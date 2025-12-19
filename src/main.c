// main.c - Pico BT Audio Receiver Entry Point
#include "bt_audio_sink.h"
#include "btstack.h"
#include "i2s_audio.h"
#include "pico/btstack_cyw43.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <stdio.h>

int main() {
  stdio_init_all();

  // Wait for USB serial to connect
  printf("\n\nWaiting 3 seconds for serial...\n");
  for (int i = 3; i > 0; i--) {
    printf("%d...\n", i);
    sleep_ms(1000);
  }

  printf("\n");
  printf("========================================\n");
  printf("   Pico BT Audio Receiver\n");
  printf("========================================\n");

  // Initialize CYW43 (WiFi/BT chip)
  printf("[MAIN] Initializing CYW43...\n");
  if (cyw43_arch_init()) {
    printf("[ERROR] Failed to initialize CYW43!\n");
    while (1) {
      tight_loop_contents();
    }
  }
  printf("[MAIN] CYW43 initialized\n");

  // Initialize BTstack with TLV storage for persistent pairing
  // This uses the built-in btstack_cyw43 helper which sets up:
  // - btstack_memory
  // - btstack_run_loop (async context)
  // - TLV flash storage for link keys
  // - Link key database
  printf("[MAIN] Initializing BTstack with TLV storage...\n");
  if (!btstack_cyw43_init(cyw43_arch_async_context())) {
    printf("[ERROR] Failed to initialize BTstack!\n");
    while (1) {
      tight_loop_contents();
    }
  }
  printf("[MAIN] BTstack initialized with link key persistence\n");

  // Initialize A2DP Sink (after BTstack is initialized)
  bt_audio_sink_init();

  // Play test tone to verify I2S audio output (using DMA)
  printf("[MAIN] Playing I2S test tone...\\n");
  i2s_play_test_tone();

  // Turn on Bluetooth
  hci_power_control(HCI_POWER_ON);

  printf("Bluetooth powered on, waiting for connections...\n");

  // LED heartbeat
  uint32_t last_led_time = 0;
  bool led_state = false;

  // Main loop
  while (1) {
    // Process BTstack
    btstack_run_loop_execute();

    // LED heartbeat (blink pattern indicates status)
    uint32_t now = to_ms_since_boot(get_absolute_time());
    uint32_t blink_rate = bt_audio_sink_is_connected() ? 1000 : 250;

    if (now - last_led_time >= blink_rate) {
      led_state = !led_state;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
      last_led_time = now;
    }
  }

  return 0;
}
