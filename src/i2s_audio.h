// i2s_audio.h - I2S audio output driver
#ifndef I2S_AUDIO_H
#define I2S_AUDIO_H

#include <stdbool.h>
#include <stdint.h>

// I2S GPIO pins for PCM5102
#define I2S_DATA_PIN 20
#define I2S_BCK_PIN 18
#define I2S_LCK_PIN 19

// Audio format
#define AUDIO_SAMPLE_RATE 44100
#define AUDIO_CHANNELS 2
#define AUDIO_BITS 16

// Initialize I2S output
void i2s_audio_init(void);

// Write PCM samples (interleaved stereo, 16-bit signed)
// Returns number of samples written
uint32_t i2s_audio_write(const int16_t *samples, uint32_t num_samples);

// Get available buffer space
uint32_t i2s_audio_get_free_samples(void);

// Start/stop playback
void i2s_audio_start(void);
void i2s_audio_stop(void);

// Play test tone (440Hz for 2 seconds)
void i2s_play_test_tone(void);

// Direct PIO test (no DMA) - for debugging GPIO/PIO issues
void i2s_direct_test(void);

#endif // I2S_AUDIO_H
