// i2s_audio.c - I2S audio output for PCM5102A DAC
// Hybrid architecture: Ring Buffer (input) + DMA Double Buffer (output)

#include "i2s_audio.h"
#include "audio_i2s.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// ============================================================================
// Configuration
// ============================================================================
#define AUDIO_PIO pio0

// Ring Buffer - absorbs Bluetooth jitter (~180ms at 44.1kHz stereo)
#define RING_BUFFER_SIZE (16 * 1024) // 16KB = 4096 stereo samples
static uint32_t ring_buffer[RING_BUFFER_SIZE / sizeof(uint32_t)];
static volatile uint32_t ring_head = 0; // Write position
static volatile uint32_t ring_tail = 0; // Read position

// DMA Double Buffer - gapless output
#define DMA_BUFFER_SAMPLES 1024 // ~23ms per buffer at 44.1kHz
static uint32_t dma_buffer_a[DMA_BUFFER_SAMPLES];
static uint32_t dma_buffer_b[DMA_BUFFER_SAMPLES];

// DMA channels
static int dma_chan_a;
static int dma_chan_b;

// PIO state
static uint audio_sm;
static uint audio_offset;
static volatile bool active = false;
static uint32_t sample_freq = 44100;
static volatile uint32_t dma_irq_count = 0;

// ============================================================================
// Ring Buffer Operations (lock-free single producer, single consumer)
// ============================================================================
#define RING_MASK ((RING_BUFFER_SIZE / sizeof(uint32_t)) - 1)

static inline uint32_t ring_available(void) {
  return (ring_head - ring_tail) & RING_MASK;
}

static inline uint32_t ring_free(void) { return RING_MASK - ring_available(); }

static void ring_write(const uint32_t *data, uint32_t count) {
  for (uint32_t i = 0; i < count; i++) {
    ring_buffer[ring_head] = data[i];
    ring_head = (ring_head + 1) & RING_MASK;
  }
}

static void ring_read(uint32_t *dest, uint32_t count) {
  for (uint32_t i = 0; i < count; i++) {
    dest[i] = ring_buffer[ring_tail];
    ring_tail = (ring_tail + 1) & RING_MASK;
  }
}

static void ring_clear(void) {
  ring_head = 0;
  ring_tail = 0;
}

// ============================================================================
// DMA IRQ Handler - Pull from ring buffer into just-finished DMA buffer
// ============================================================================
static void __isr __time_critical_func(dma_irq_handler)(void) {
  uint32_t *buffer_to_fill = NULL;

  if (dma_irqn_get_channel_status(0, dma_chan_a)) {
    dma_irqn_acknowledge_channel(0, dma_chan_a);
    dma_irq_count++;
    buffer_to_fill = dma_buffer_a;
    // Reset read address for next time this channel plays
    dma_channel_set_read_addr(dma_chan_a, dma_buffer_a, false);
  }

  if (dma_irqn_get_channel_status(0, dma_chan_b)) {
    dma_irqn_acknowledge_channel(0, dma_chan_b);
    dma_irq_count++;
    buffer_to_fill = dma_buffer_b;
    dma_channel_set_read_addr(dma_chan_b, dma_buffer_b, false);
  }

  if (buffer_to_fill) {
    uint32_t available = ring_available();
    if (available >= DMA_BUFFER_SAMPLES) {
      ring_read(buffer_to_fill, DMA_BUFFER_SAMPLES);
    } else if (available > 0) {
      // Partial fill + silence
      ring_read(buffer_to_fill, available);
      memset(&buffer_to_fill[available], 0,
             (DMA_BUFFER_SAMPLES - available) * sizeof(uint32_t));
    } else {
      // Underrun - fill with silence
      memset(buffer_to_fill, 0, DMA_BUFFER_SAMPLES * sizeof(uint32_t));
    }
  }
}

// ============================================================================
// PIO Clock Configuration
// ============================================================================
static void update_pio_frequency(uint32_t freq) {
  uint32_t system_clock_frequency = clock_get_hz(clk_sys);
  uint32_t divider = system_clock_frequency * 4 / freq;

  // Disable IRQ and pause PIO while changing clock to prevent crashes
  irq_set_enabled(DMA_IRQ_0, false);
  pio_sm_set_enabled(AUDIO_PIO, audio_sm, false);

  pio_sm_set_clkdiv_int_frac(AUDIO_PIO, audio_sm, divider >> 8u,
                             divider & 0xffu);
  sample_freq = freq;

  // Re-enable PIO and IRQ
  if (active) {
    pio_sm_set_enabled(AUDIO_PIO, audio_sm, true);
  }
  irq_set_enabled(DMA_IRQ_0, true);

  printf("[I2S] Sample rate set to %u Hz\n", freq);
}

void i2s_audio_set_sample_rate(uint32_t rate) {
  if ((rate == 44100 || rate == 48000) && rate != sample_freq) {
    update_pio_frequency(rate);
  }
}

// ============================================================================
// Initialization
// ============================================================================
void i2s_audio_init(void) {
  // Clear buffers
  ring_clear();
  memset(dma_buffer_a, 0, sizeof(dma_buffer_a));
  memset(dma_buffer_b, 0, sizeof(dma_buffer_b));
  active = false;

  // Initialize GPIO pins for PIO
  pio_gpio_init(AUDIO_PIO, I2S_DATA_PIN);
  pio_gpio_init(AUDIO_PIO, I2S_BCK_PIN);
  pio_gpio_init(AUDIO_PIO, I2S_LCK_PIN);

  // Claim and configure PIO state machine
  audio_sm = pio_claim_unused_sm(AUDIO_PIO, true);
  audio_offset = pio_add_program(AUDIO_PIO, &audio_i2s_program);
  audio_i2s_program_init(AUDIO_PIO, audio_sm, audio_offset, I2S_DATA_PIN,
                         I2S_BCK_PIN);

  // Set default clock divider
  update_pio_frequency(44100);

  // ========================================================================
  // DMA Setup with Chaining
  // ========================================================================
  dma_chan_a = dma_claim_unused_channel(true);
  dma_chan_b = dma_claim_unused_channel(true);

  // Configure Channel A
  dma_channel_config cfg_a = dma_channel_get_default_config(dma_chan_a);
  channel_config_set_transfer_data_size(&cfg_a, DMA_SIZE_32);
  channel_config_set_read_increment(&cfg_a, true);
  channel_config_set_write_increment(&cfg_a, false);
  channel_config_set_dreq(&cfg_a, pio_get_dreq(AUDIO_PIO, audio_sm, true));
  channel_config_set_chain_to(&cfg_a, dma_chan_b);

  dma_channel_configure(dma_chan_a, &cfg_a, &AUDIO_PIO->txf[audio_sm],
                        dma_buffer_a, DMA_BUFFER_SAMPLES, false);

  // Configure Channel B
  dma_channel_config cfg_b = dma_channel_get_default_config(dma_chan_b);
  channel_config_set_transfer_data_size(&cfg_b, DMA_SIZE_32);
  channel_config_set_read_increment(&cfg_b, true);
  channel_config_set_write_increment(&cfg_b, false);
  channel_config_set_dreq(&cfg_b, pio_get_dreq(AUDIO_PIO, audio_sm, true));
  channel_config_set_chain_to(&cfg_b, dma_chan_a);

  dma_channel_configure(dma_chan_b, &cfg_b, &AUDIO_PIO->txf[audio_sm],
                        dma_buffer_b, DMA_BUFFER_SAMPLES, false);

  // Enable IRQs
  irq_add_shared_handler(DMA_IRQ_0, dma_irq_handler,
                         PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  dma_irqn_set_channel_enabled(0, dma_chan_a, true);
  dma_irqn_set_channel_enabled(0, dma_chan_b, true);
  irq_set_enabled(DMA_IRQ_0, true);

  // Start PIO and DMA - they run continuously, playing silence when no data
  pio_sm_set_enabled(AUDIO_PIO, audio_sm, true);
  dma_channel_start(dma_chan_a);

  printf("[I2S] Initialized and running\n");
}

// ============================================================================
// Audio Write - Push to ring buffer (never blocks)
// ============================================================================
uint32_t i2s_audio_write(const int16_t *samples, uint32_t num_samples) {
  if (!active)
    return 0;

  uint32_t pairs = num_samples / 2;
  uint32_t free = ring_free();
  if (pairs > free)
    pairs = free;
  if (pairs == 0)
    return 0;

  // Pack and write to ring buffer
  for (uint32_t i = 0; i < pairs; i++) {
    int16_t left = samples[i * 2];
    int16_t right = samples[i * 2 + 1];
    uint32_t packed = ((uint32_t)(uint16_t)left << 16) | (uint16_t)right;
    ring_buffer[ring_head] = packed;
    ring_head = (ring_head + 1) & RING_MASK;
  }

  return pairs * 2;
}

uint32_t i2s_audio_get_free_samples(void) {
  if (!active)
    return 0;
  return ring_free() * 2; // Stereo samples
}

// ============================================================================
// Start/Stop
// ============================================================================
void i2s_audio_start(void) {
  if (!active) {
    printf("[I2S] Starting...\n");

    // Disable DMA IRQ while clearing ring buffer to prevent race
    irq_set_enabled(DMA_IRQ_0, false);
    ring_clear();
    active = true;
    irq_set_enabled(DMA_IRQ_0, true);

    printf("[I2S] Started, dma_count=%u, ring=%u\n", dma_irq_count,
           ring_available());
  }
}

void i2s_audio_stop(void) {
  if (active) {
    printf("[I2S] Stopping...\n");

    // Disable DMA IRQ while clearing ring buffer to prevent race
    irq_set_enabled(DMA_IRQ_0, false);
    active = false;
    ring_clear();
    irq_set_enabled(DMA_IRQ_0, true);

    printf("[I2S] Stopped, dma_count=%u\n", dma_irq_count);
  }
}

bool i2s_audio_is_active(void) { return active; }

void i2s_audio_get_stats(uint32_t *irq_count, uint32_t *ring_level) {
  if (irq_count)
    *irq_count = dma_irq_count;
  if (ring_level)
    *ring_level = ring_available();
}

// ============================================================================
// Helper: Generate and play a tone segment
// ============================================================================
static void play_tone_segment(float freq1, float freq2, int duration_ms,
                              int amplitude, bool fade_out) {
  const float sample_rate = 44100.0f;
  const int total_samples = (int)(sample_rate * duration_ms / 1000);
  int16_t stereo_samples[200];
  int sample_idx = 0;

  while (sample_idx < total_samples) {
    int chunk_size = 100;
    if (sample_idx + chunk_size > total_samples) {
      chunk_size = total_samples - sample_idx;
    }

    for (int i = 0; i < chunk_size; i++) {
      int s = sample_idx + i;
      float envelope = 1.0f;
      if (fade_out) {
        envelope = 1.0f - ((float)s / total_samples);
        envelope = envelope * envelope;
      }

      float t = (float)s / sample_rate;
      float wave1 = sinf(2.0f * 3.14159265f * freq1 * t);
      float wave2 = (freq2 > 0) ? sinf(2.0f * 3.14159265f * freq2 * t) : 0;
      float combined = (freq2 > 0) ? (wave1 + wave2 * 0.7f) / 1.7f : wave1;

      int16_t value = (int16_t)(amplitude * envelope * combined);
      stereo_samples[i * 2] = value;
      stereo_samples[i * 2 + 1] = value;
    }

    while (i2s_audio_get_free_samples() < (uint32_t)(chunk_size * 2)) {
      tight_loop_contents();
    }
    i2s_audio_write(stereo_samples, chunk_size * 2);
    sample_idx += chunk_size;
  }
}

// ============================================================================
// Startup Tone - Gentle two-note chord with fade
// ============================================================================
void i2s_play_test_tone(void) {
  bool was_active = i2s_audio_is_active();
  if (!was_active)
    i2s_audio_start();

  // C5 (523Hz) + E5 (659Hz) = pleasant major third, quiet and short
  play_tone_segment(523.0f, 659.0f, 200, 800, true);
  sleep_ms(80);

  if (!was_active)
    i2s_audio_stop();
}

// ============================================================================
// Connected Sound - Rising two-tone (boop-boop up)
// ============================================================================
void i2s_play_connected(void) {
  bool was_active = i2s_audio_is_active();
  if (!was_active)
    i2s_audio_start();

  play_tone_segment(440.0f, 0, 60, 600, false); // A4
  sleep_ms(40);
  play_tone_segment(554.0f, 0, 80, 600, true); // C#5 (higher)
  sleep_ms(50);

  if (!was_active)
    i2s_audio_stop();
}

// ============================================================================
// Disconnected Sound - Falling two-tone (boop-boop down)
// ============================================================================
void i2s_play_disconnected(void) {
  bool was_active = i2s_audio_is_active();
  if (!was_active)
    i2s_audio_start();

  play_tone_segment(554.0f, 0, 60, 600, false); // C#5
  sleep_ms(40);
  play_tone_segment(440.0f, 0, 80, 600, true); // A4 (lower)
  sleep_ms(50);

  if (!was_active)
    i2s_audio_stop();
}

// ============================================================================
// Pairing Mode Beep - Short single beep (keep I2S running)
// ============================================================================
void i2s_play_pairing_beep(void) {
  bool was_active = i2s_audio_is_active();
  if (!was_active)
    i2s_audio_start();

  play_tone_segment(880.0f, 0, 50, 500, true); // A5 - short quiet beep
  sleep_ms(40);

  // Note: Don't stop I2S here - keep it running for smoother sound
  // It will be stopped when pairing mode ends or device connects
  if (!was_active) {
    // Actually, let's keep it running to avoid pops between beeps
    // i2s_audio_stop();
  }
}

// ============================================================================
// Direct Test (bypass DMA for debugging)
// ============================================================================
void i2s_direct_test(void) {
  pio_sm_set_enabled(AUDIO_PIO, audio_sm, true);

  for (int j = 0; j < 44100; j++) {
    int16_t value = (int16_t)(12000.0f * sinf(2.0f * 3.14159265f * j / 100));
    uint32_t sample = ((uint32_t)(uint16_t)value << 16) | (uint16_t)value;
    pio_sm_put_blocking(AUDIO_PIO, audio_sm, sample);
  }

  pio_sm_set_enabled(AUDIO_PIO, audio_sm, false);
}
