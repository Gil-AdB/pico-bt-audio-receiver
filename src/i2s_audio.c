// i2s_audio.c - I2S audio output using PIO and DMA
// For PCM5102A DAC module

#include "i2s_audio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

// ============================================================================
// PIO Program for I2S output
// ============================================================================
// This generates: BCK (bit clock), LRCK (word select), DIN (data)
// I2S format: MSB first, data valid on rising edge of BCK
// LRCK: LOW = Left channel, HIGH = Right channel

// PIO assembly equivalent:
// .side_set 2          ; BCK and LRCK as sideset
// loop:
//   out pins, 1  side 0b10 [0]   ; shift out bit, BCK high, LRCK from bit
//              side 0b00 [0]   ; BCK low
//
// Simplified: shift 16 bits per channel, toggle LRCK between L/R

#define AUDIO_PIO_WRAP_TARGET 0
#define AUDIO_PIO_WRAP 3

static const uint16_t audio_pio_program_instructions[] = {
    // Left channel (16 bits): LRCK = 0
    //         side  delay
    0x7001, // out pins, 1   side 0b10 [0]  ; data out, BCK high
    0x5001, // out pins, 1   side 0b00 [0]  ; data out, BCK low
    // Right channel (16 bits): LRCK = 1
    0x7801, // out pins, 1   side 0b11 [0]  ; data out, BCK high, LRCK high
    0x5801, // out pins, 1   side 0b01 [0]  ; data out, BCK low, LRCK high
};

static const struct pio_program audio_pio_program = {
    .instructions = audio_pio_program_instructions,
    .length = 4,
    .origin = -1,
};

// ============================================================================
// Audio Buffers
// ============================================================================
#define AUDIO_BUFFER_SAMPLES 1024
static uint32_t audio_buffer_a[AUDIO_BUFFER_SAMPLES]; // L+R packed
static uint32_t audio_buffer_b[AUDIO_BUFFER_SAMPLES];
static volatile uint32_t *current_write_buffer = audio_buffer_a;
static volatile uint32_t write_index = 0;

static PIO pio_hw = pio0;
static uint pio_sm = 0;
static int dma_chan_a = -1;
static int dma_chan_b = -1;
static volatile bool audio_running = false;

// DMA interrupt handler for double-buffering
static void dma_irq_handler(void) {
  if (dma_channel_get_irq0_status(dma_chan_a)) {
    dma_channel_acknowledge_irq0(dma_chan_a);
    // Chain to buffer B
    dma_channel_set_read_addr(dma_chan_a, audio_buffer_a, false);
  }
  if (dma_channel_get_irq0_status(dma_chan_b)) {
    dma_channel_acknowledge_irq0(dma_chan_b);
    // Chain to buffer A
    dma_channel_set_read_addr(dma_chan_b, audio_buffer_b, false);
  }
}

void i2s_audio_init(void) {
  printf("[I2S] Initializing...\n");

  // Clear buffers
  memset(audio_buffer_a, 0, sizeof(audio_buffer_a));
  memset(audio_buffer_b, 0, sizeof(audio_buffer_b));

  // Configure GPIO pins
  gpio_init(I2S_DATA_PIN);
  gpio_init(I2S_BCK_PIN);
  gpio_init(I2S_LCK_PIN);
  gpio_set_dir(I2S_DATA_PIN, GPIO_OUT);
  gpio_set_dir(I2S_BCK_PIN, GPIO_OUT);
  gpio_set_dir(I2S_LCK_PIN, GPIO_OUT);

  // Load PIO program
  pio_sm = pio_claim_unused_sm(pio_hw, true);
  uint offset = pio_add_program(pio_hw, &audio_pio_program);

  // Configure state machine
  pio_sm_config c = pio_get_default_sm_config();

  // OUT pin for data
  sm_config_set_out_pins(&c, I2S_DATA_PIN, 1);
  pio_gpio_init(pio_hw, I2S_DATA_PIN);
  pio_sm_set_consecutive_pindirs(pio_hw, pio_sm, I2S_DATA_PIN, 1, true);

  // Side-set for BCK (bit 0) and LRCK (bit 1)
  sm_config_set_sideset_pins(&c, I2S_BCK_PIN);
  sm_config_set_sideset(&c, 2, false, false);
  pio_gpio_init(pio_hw, I2S_BCK_PIN);
  pio_gpio_init(pio_hw, I2S_LCK_PIN);
  pio_sm_set_consecutive_pindirs(pio_hw, pio_sm, I2S_BCK_PIN, 2, true);

  // Shift config: shift out MSB first, autopull at 32 bits
  sm_config_set_out_shift(&c, false, true, 32);

  // Program wrap
  sm_config_set_wrap(&c, offset + AUDIO_PIO_WRAP_TARGET,
                     offset + AUDIO_PIO_WRAP);

  // Clock: sample_rate * 32 bits * 2 (for BCK toggle per instruction)
  float freq = AUDIO_SAMPLE_RATE * 32 * 2;
  float div = (float)clock_get_hz(clk_sys) / freq;
  sm_config_set_clkdiv(&c, div);

  printf("[I2S] Clock div: %.2f (sys=%lu, target=%d)\n", div,
         clock_get_hz(clk_sys), (int)freq);

  pio_sm_init(pio_hw, pio_sm, offset, &c);

  // Configure DMA channels for ping-pong
  dma_chan_a = dma_claim_unused_channel(true);
  dma_chan_b = dma_claim_unused_channel(true);

  // DMA channel A
  dma_channel_config dma_cfg_a = dma_channel_get_default_config(dma_chan_a);
  channel_config_set_transfer_data_size(&dma_cfg_a, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_cfg_a, true);
  channel_config_set_write_increment(&dma_cfg_a, false);
  channel_config_set_dreq(&dma_cfg_a, pio_get_dreq(pio_hw, pio_sm, true));
  channel_config_set_chain_to(&dma_cfg_a, dma_chan_b);

  dma_channel_configure(dma_chan_a, &dma_cfg_a, &pio_hw->txf[pio_sm],
                        audio_buffer_a, AUDIO_BUFFER_SAMPLES, false);

  // DMA channel B
  dma_channel_config dma_cfg_b = dma_channel_get_default_config(dma_chan_b);
  channel_config_set_transfer_data_size(&dma_cfg_b, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_cfg_b, true);
  channel_config_set_write_increment(&dma_cfg_b, false);
  channel_config_set_dreq(&dma_cfg_b, pio_get_dreq(pio_hw, pio_sm, true));
  channel_config_set_chain_to(&dma_cfg_b, dma_chan_a);

  dma_channel_configure(dma_chan_b, &dma_cfg_b, &pio_hw->txf[pio_sm],
                        audio_buffer_b, AUDIO_BUFFER_SAMPLES, false);

  printf("[I2S] Initialized: %d Hz, pins BCK=%d LCK=%d DIN=%d\n",
         AUDIO_SAMPLE_RATE, I2S_BCK_PIN, I2S_LCK_PIN, I2S_DATA_PIN);
}

uint32_t i2s_audio_write(const int16_t *samples, uint32_t num_samples) {
  // Pack stereo 16-bit samples into 32-bit words
  uint32_t written = 0;
  uint32_t *buf = (uint32_t *)current_write_buffer;

  while (written < num_samples / 2 && write_index < AUDIO_BUFFER_SAMPLES) {
    int16_t left = samples[written * 2];
    int16_t right = samples[written * 2 + 1];
    buf[write_index++] = ((uint32_t)(uint16_t)left << 16) | (uint16_t)right;
    written++;
  }

  // Buffer full? Switch
  if (write_index >= AUDIO_BUFFER_SAMPLES) {
    current_write_buffer = (current_write_buffer == audio_buffer_a)
                               ? audio_buffer_b
                               : audio_buffer_a;
    write_index = 0;
  }

  return written * 2;
}

uint32_t i2s_audio_get_free_samples(void) {
  return (AUDIO_BUFFER_SAMPLES - write_index) * 2;
}

void i2s_audio_start(void) {
  if (!audio_running) {
    audio_running = true;
    dma_channel_start(dma_chan_a);
    pio_sm_set_enabled(pio_hw, pio_sm, true);
    printf("[I2S] Started\n");
  }
}

void i2s_audio_stop(void) {
  if (audio_running) {
    audio_running = false;
    pio_sm_set_enabled(pio_hw, pio_sm, false);
    dma_channel_abort(dma_chan_a);
    dma_channel_abort(dma_chan_b);
    printf("[I2S] Stopped\n");
  }
}
