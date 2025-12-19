// i2s_audio.c - I2S audio output for PCM5102A DAC
// Uses DMA chaining for gapless audio playback

#include "i2s_audio.h"
#include "audio_i2s.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// ============================================================================
// Configuration
// ============================================================================
#define AUDIO_PIO pio0

// Audio Buffers - ping-pong with DMA chaining
#define BUFFER_SAMPLES 1024 // ~23ms at 44.1kHz
static uint32_t buffer_a[BUFFER_SAMPLES];
static uint32_t buffer_b[BUFFER_SAMPLES];

// Write state - simple: just track position in the buffer we're filling
static volatile int write_buf_idx;  // 0 = filling A, 1 = filling B
static volatile uint32_t write_pos; // Position in current write buffer

// DMA channels - two channels chained together
static int dma_chan_a;
static int dma_chan_b;

static uint audio_sm;
static uint audio_offset;
static volatile bool active;
static uint32_t sample_freq = 44100;

// Stats
static volatile uint32_t irq_count_a = 0;
static volatile uint32_t irq_count_b = 0;

// Forward declarations
static void dma_irq_handler(void);
static void fill_buffer_with_silence(uint32_t *buf, uint32_t count);

// ============================================================================
// DMA IRQ Handler - buffer just finished, the OTHER is already playing
// ============================================================================
static void __isr __time_critical_func(dma_irq_handler)(void) {
  // Check which channel finished
  if (dma_irqn_get_channel_status(0, dma_chan_a)) {
    dma_irqn_acknowledge_channel(0, dma_chan_a);
    irq_count_a++;
    // Buffer A just finished. Buffer B is now playing (automatically chained).
    // We should now be filling Buffer A for the NEXT round.
    // Reset the DMA read address back to buffer_a for when it chains back.
    dma_channel_set_read_addr(dma_chan_a, buffer_a, false);
  }

  if (dma_irqn_get_channel_status(0, dma_chan_b)) {
    dma_irqn_acknowledge_channel(0, dma_chan_b);
    irq_count_b++;
    // Buffer B just finished. Buffer A is now playing (automatically chained).
    // Reset the DMA read address back to buffer_b for when it chains back.
    dma_channel_set_read_addr(dma_chan_b, buffer_b, false);
  }
}

static void fill_buffer_with_silence(uint32_t *buf, uint32_t count) {
  memset(buf, 0, count * sizeof(uint32_t));
}

static void update_pio_frequency(uint32_t freq) {
  uint32_t system_clock_frequency = clock_get_hz(clk_sys);
  uint32_t divider = system_clock_frequency * 4 / freq;
  pio_sm_set_clkdiv_int_frac(AUDIO_PIO, audio_sm, divider >> 8u,
                             divider & 0xffu);
  sample_freq = freq;
}

// ============================================================================
// Initialization
// ============================================================================
void i2s_audio_init(void) {
  // Clear buffers
  fill_buffer_with_silence(buffer_a, BUFFER_SAMPLES);
  fill_buffer_with_silence(buffer_b, BUFFER_SAMPLES);

  write_buf_idx = 0;
  write_pos = 0;
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

  // Set clock divider for 44.1kHz
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
  channel_config_set_chain_to(&cfg_a, dma_chan_b); // Chain to B when done

  dma_channel_configure(dma_chan_a, &cfg_a,
                        &AUDIO_PIO->txf[audio_sm], // dest: PIO TX FIFO
                        buffer_a,                  // src: buffer A
                        BUFFER_SAMPLES,            // count
                        false);                    // don't start yet

  // Configure Channel B
  dma_channel_config cfg_b = dma_channel_get_default_config(dma_chan_b);
  channel_config_set_transfer_data_size(&cfg_b, DMA_SIZE_32);
  channel_config_set_read_increment(&cfg_b, true);
  channel_config_set_write_increment(&cfg_b, false);
  channel_config_set_dreq(&cfg_b, pio_get_dreq(AUDIO_PIO, audio_sm, true));
  channel_config_set_chain_to(&cfg_b, dma_chan_a); // Chain to A when done

  dma_channel_configure(dma_chan_b, &cfg_b,
                        &AUDIO_PIO->txf[audio_sm], // dest: PIO TX FIFO
                        buffer_b,                  // src: buffer B
                        BUFFER_SAMPLES,            // count
                        false);                    // don't start yet

  // Enable IRQs for both channels
  irq_add_shared_handler(DMA_IRQ_0, dma_irq_handler,
                         PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  dma_irqn_set_channel_enabled(0, dma_chan_a, true);
  dma_irqn_set_channel_enabled(0, dma_chan_b, true);
  irq_set_enabled(DMA_IRQ_0, true);
}

// ============================================================================
// Audio Write - fill the buffer that's NOT currently playing
// ============================================================================
uint32_t i2s_audio_write(const int16_t *samples, uint32_t num_samples) {
  if (!active)
    return 0;

  // Get the buffer we're writing to
  uint32_t *buf = (write_buf_idx == 0) ? buffer_a : buffer_b;

  // Calculate space available
  uint32_t space = BUFFER_SAMPLES - write_pos;
  uint32_t pairs = num_samples / 2; // stereo pairs
  if (pairs > space)
    pairs = space;
  if (pairs == 0)
    return 0;

  // Write samples
  for (uint32_t i = 0; i < pairs; i++) {
    int16_t left = samples[i * 2];
    int16_t right = samples[i * 2 + 1];
    buf[write_pos++] = ((uint32_t)(uint16_t)left << 16) | (uint16_t)right;
  }

  // If buffer is full, switch to the other one
  if (write_pos >= BUFFER_SAMPLES) {
    write_buf_idx = 1 - write_buf_idx;
    write_pos = 0;
  }

  return pairs * 2;
}

uint32_t i2s_audio_get_free_samples(void) {
  if (!active)
    return 0;
  return (BUFFER_SAMPLES - write_pos) * 2;
}

// ============================================================================
// Start/Stop
// ============================================================================
void i2s_audio_start(void) {
  if (!active) {
    active = true;

    // Clear buffers
    fill_buffer_with_silence(buffer_a, BUFFER_SAMPLES);
    fill_buffer_with_silence(buffer_b, BUFFER_SAMPLES);

    write_buf_idx = 0;
    write_pos = 0;
    irq_count_a = 0;
    irq_count_b = 0;

    // Reset DMA read addresses
    dma_channel_set_read_addr(dma_chan_a, buffer_a, false);
    dma_channel_set_read_addr(dma_chan_b, buffer_b, false);

    // Enable PIO state machine
    pio_sm_set_enabled(AUDIO_PIO, audio_sm, true);

    // Start DMA chain by triggering channel A
    dma_channel_start(dma_chan_a);
  }
}

void i2s_audio_stop(void) {
  if (active) {
    active = false;

    // Abort both DMA channels
    dma_channel_abort(dma_chan_a);
    dma_channel_abort(dma_chan_b);

    // Disable PIO
    pio_sm_set_enabled(AUDIO_PIO, audio_sm, false);
  }
}

bool i2s_audio_is_active(void) { return active; }

// ============================================================================
// Test Tone - Pleasant Sine Wave
// ============================================================================
void i2s_play_test_tone(void) {
// Generate 440Hz sine wave lookup table
#define SINE_TABLE_SIZE 100
  static int16_t sine_table[SINE_TABLE_SIZE];
  static bool table_generated = false;

  if (!table_generated) {
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
      sine_table[i] =
          (int16_t)(16000.0f * sinf(2.0f * 3.14159265f * i / SINE_TABLE_SIZE));
    }
    table_generated = true;
  }

  // Pre-fill both buffers with sine wave
  for (uint32_t i = 0; i < BUFFER_SAMPLES; i++) {
    int16_t value = sine_table[i % SINE_TABLE_SIZE];
    buffer_a[i] = ((uint32_t)(uint16_t)value << 16) | (uint16_t)value;
    buffer_b[i] = buffer_a[i];
  }

  i2s_audio_start();

  // Play for 1 second (sine wave already in both buffers, DMA will loop)
  sleep_ms(1000);

  i2s_audio_stop();
}

// ============================================================================
// Direct Test (bypass DMA for debugging)
// ============================================================================
void i2s_direct_test(void) {
  // Enable PIO
  pio_sm_set_enabled(AUDIO_PIO, audio_sm, true);

  // Generate simple tone directly to PIO
  for (int j = 0; j < 44100; j++) { // 1 second
    int16_t value = ((j % 100) < 50) ? 16000 : -16000;
    uint32_t sample = ((uint32_t)(uint16_t)value << 16) | (uint16_t)value;
    pio_sm_put_blocking(AUDIO_PIO, audio_sm, sample);
  }

  pio_sm_set_enabled(AUDIO_PIO, audio_sm, false);
}
