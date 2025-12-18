// main.c - Pico BT Audio Receiver Entry Point
#include "bt_audio_sink.h"
#include "btstack.h"
#include "pico/btstack_hci_transport_cyw43.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <stdio.h>

int main() {
  stdio_init_all();

  // Wait for USB serial to connect (for debugging)
  sleep_ms(3000);

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

  // Initialize BTstack
  hci_init(hci_transport_cyw43_instance(), NULL);

  // Initialize A2DP Sink
  bt_audio_sink_init();

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
