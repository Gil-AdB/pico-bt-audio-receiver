// main.c - Pico BT Audio Receiver Entry Point
#include "bt_audio_sink.h"
#include "btstack.h"
#include "btstack_run_loop.h"
#include "i2s_audio.h"
#include "pico/btstack_cyw43.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <stdio.h>

// LED heartbeat state
static uint32_t last_led_time = 0;
static bool led_state = false;

// BTstack timer for LED heartbeat (since btstack_run_loop_execute is blocking)
static btstack_timer_source_t led_timer;

static void led_heartbeat_handler(btstack_timer_source_t *ts) {
  (void)ts;

  // Toggle LED
  led_state = !led_state;
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);

  // Set next timeout based on connection status
  uint32_t blink_rate = bt_audio_sink_is_connected() ? 1000 : 250;
  btstack_run_loop_set_timer(&led_timer, blink_rate);
  btstack_run_loop_add_timer(&led_timer);
}

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

  // Play test tone to verify I2S audio output
  printf("[MAIN] Playing I2S test tone...\n");
  i2s_play_test_tone();

  // Setup LED heartbeat timer (runs inside BTstack run loop)
  btstack_run_loop_set_timer_handler(&led_timer, led_heartbeat_handler);
  btstack_run_loop_set_timer(&led_timer, 250);
  btstack_run_loop_add_timer(&led_timer);

  // Turn on Bluetooth
  hci_power_control(HCI_POWER_ON);

  printf("Bluetooth powered on, waiting for connections...\n");
  printf("[LED] Fast blink = discoverable, Slow blink = connected\n");

  // Run BTstack event loop (this blocks forever)
  btstack_run_loop_execute();

  return 0;
}
