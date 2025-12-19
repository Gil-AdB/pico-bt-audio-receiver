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
    buffer_to_fill = dma_buffer_a;
    // Reset read address for next time this channel plays
    dma_channel_set_read_addr(dma_chan_a, dma_buffer_a, false);
  }

  if (dma_irqn_get_channel_status(0, dma_chan_b)) {
    dma_irqn_acknowledge_channel(0, dma_chan_b);
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
  pio_sm_set_clkdiv_int_frac(AUDIO_PIO, audio_sm, divider >> 8u,
                             divider & 0xffu);
  sample_freq = freq;
}

void i2s_audio_set_sample_rate(uint32_t rate) {
  if (rate == 44100 || rate == 48000) {
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
    active = true;

    // Clear everything
    ring_clear();
    memset(dma_buffer_a, 0, sizeof(dma_buffer_a));
    memset(dma_buffer_b, 0, sizeof(dma_buffer_b));

    // Reset DMA read addresses
    dma_channel_set_read_addr(dma_chan_a, dma_buffer_a, false);
    dma_channel_set_read_addr(dma_chan_b, dma_buffer_b, false);

    // Enable PIO
    pio_sm_set_enabled(AUDIO_PIO, audio_sm, true);

    // Start DMA chain
    dma_channel_start(dma_chan_a);
  }
}

void i2s_audio_stop(void) {
  if (active) {
    active = false;
    dma_channel_abort(dma_chan_a);
    dma_channel_abort(dma_chan_b);
    pio_sm_set_enabled(AUDIO_PIO, audio_sm, false);
  }
}

bool i2s_audio_is_active(void) { return active; }

// ============================================================================
// Test Tone - Pleasant Sine Wave
// ============================================================================
void i2s_play_test_tone(void) {
#define SINE_TABLE_SIZE 100
  static int16_t sine_table[SINE_TABLE_SIZE];
  static bool table_generated = false;

  if (!table_generated) {
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
      sine_table[i] =
          (int16_t)(12000.0f * sinf(2.0f * 3.14159265f * i / SINE_TABLE_SIZE));
    }
    table_generated = true;
  }

  i2s_audio_start();

  // Fill ring buffer with sine wave, let it play for 1 second
  int16_t stereo_samples[200];
  for (int chunk = 0; chunk < 441; chunk++) { // ~1 second at 44.1kHz
    for (int i = 0; i < 100; i++) {
      stereo_samples[i * 2] = sine_table[i];
      stereo_samples[i * 2 + 1] = sine_table[i];
    }
    while (i2s_audio_get_free_samples() < 200) {
      tight_loop_contents();
    }
    i2s_audio_write(stereo_samples, 200);
  }

  sleep_ms(100); // Let buffer drain
  i2s_audio_stop();
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
