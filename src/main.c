// main.c - Pico BT Audio Receiver Entry Point
#include "bt_audio_sink.h"
#include "btstack.h"
#include "btstack_run_loop.h"
#include "i2s_audio.h"
#include "pico/btstack_cyw43.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <stdio.h>

// ============================================================================
// Hard Fault Handler - prints crash info
// ============================================================================
void __attribute__((naked)) HardFault_Handler(void) {
  __asm volatile("mrs r0, msp\n" // Get Main Stack Pointer
                 "b hard_fault_handler_c\n");
}

void hard_fault_handler_c(uint32_t *stack) {
  // Stack frame: R0, R1, R2, R3, R12, LR, PC, xPSR
  uint32_t r0 = stack[0];
  uint32_t r1 = stack[1];
  uint32_t r2 = stack[2];
  uint32_t r3 = stack[3];
  uint32_t r12 = stack[4];
  uint32_t lr = stack[5];
  uint32_t pc = stack[6];
  uint32_t psr = stack[7];

  printf("\n\n*** HARD FAULT ***\n");
  printf("PC  = 0x%08lX  (crash location)\n", pc);
  printf("LR  = 0x%08lX  (return address)\n", lr);
  printf("PSR = 0x%08lX\n", psr);
  printf("R0  = 0x%08lX  R1  = 0x%08lX\n", r0, r1);
  printf("R2  = 0x%08lX  R3  = 0x%08lX\n", r2, r3);
  printf("R12 = 0x%08lX\n", r12);
  printf("*** Use: arm-none-eabi-addr2line -e build/pico_bt_audio.elf 0x%08lX "
         "***\n",
         pc);

  // Blink LED rapidly to indicate fault
  while (1) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    for (volatile int i = 0; i < 100000; i++)
      ;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    for (volatile int i = 0; i < 100000; i++)
      ;
  }
}

// BOOTSEL button is directly readable via QSPI CS pin
#define BOOTSEL_PIN_READ() (!gpio_get(PICO_DEFAULT_BOOTSEL_GPIO_PIN))

// Pairing mode state
static bool pairing_mode = false;
static uint32_t pairing_mode_start_time = 0;
static uint32_t bootsel_press_start = 0;
static bool bootsel_was_pressed = false;

// LED state
static bool led_state = false;

// BTstack timers
static btstack_timer_source_t led_timer;
static btstack_timer_source_t pairing_timer;
static btstack_timer_source_t bootsel_timer;

// Forward declarations
static void check_bootsel_button(btstack_timer_source_t *ts);
static void pairing_mode_tick(btstack_timer_source_t *ts);
static void enter_pairing_mode(void);
static void exit_pairing_mode(void);

// ============================================================================
// LED heartbeat - different patterns for different states
// ============================================================================
static void led_heartbeat_handler(btstack_timer_source_t *ts) {
  (void)ts;

  led_state = !led_state;
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);

  uint32_t blink_rate;
  if (bt_audio_sink_is_connected()) {
    blink_rate = 1000; // Slow blink when connected
  } else if (pairing_mode) {
    blink_rate = 100; // Very fast blink in pairing mode
  } else {
    blink_rate = 500; // Medium blink when connectable but not discoverable
  }

  btstack_run_loop_set_timer(&led_timer, blink_rate);
  btstack_run_loop_add_timer(&led_timer);
}

// ============================================================================
// BOOTSEL button checker - runs every 100ms
// ============================================================================
static void check_bootsel_button(btstack_timer_source_t *ts) {
  (void)ts;

  // On Pico W, we can't directly read BOOTSEL, so we use an alternative
  // approach The BOOTSEL button grounds the flash CS pin when pressed For
  // simplicity, we'll check if the user manually triggers pairing via USB
  // command OR we use the built-in method

  // Alternative: Use GPIO 9 if user has connected a button there
  // For now, we'll just reschedule the timer
  // TODO: Implement proper BOOTSEL detection or use external button

  btstack_run_loop_set_timer(&bootsel_timer, 100);
  btstack_run_loop_add_timer(&bootsel_timer);
}

// ============================================================================
// Pairing mode tick - beeps and timeout
// ============================================================================
static void pairing_mode_tick(btstack_timer_source_t *ts) {
  (void)ts;

  if (!pairing_mode)
    return;

  // Check timeout (45 seconds)
  uint32_t elapsed =
      to_ms_since_boot(get_absolute_time()) - pairing_mode_start_time;
  if (elapsed > 45000) {
    printf("[PAIRING] Timeout after 45 seconds\n");
    exit_pairing_mode();
    return;
  }

  // Check if device connected
  if (bt_audio_sink_is_connected()) {
    printf("[PAIRING] Device connected, exiting pairing mode\n");
    exit_pairing_mode();
    return;
  }

  // Play beep
  i2s_play_pairing_beep();

  // Schedule next tick (every 2 seconds)
  btstack_run_loop_set_timer(&pairing_timer, 2000);
  btstack_run_loop_add_timer(&pairing_timer);
}

// ============================================================================
// Enter/Exit pairing mode
// ============================================================================
static void enter_pairing_mode(void) {
  if (pairing_mode)
    return;

  printf("[PAIRING] Entering pairing mode (45 second timeout)\n");
  pairing_mode = true;
  pairing_mode_start_time = to_ms_since_boot(get_absolute_time());

  // Make discoverable AND connectable
  gap_discoverable_control(1);
  gap_connectable_control(1);

  // Start pairing mode tick timer
  btstack_run_loop_set_timer_handler(&pairing_timer, pairing_mode_tick);
  btstack_run_loop_set_timer(&pairing_timer, 500); // First beep soon
  btstack_run_loop_add_timer(&pairing_timer);
}

static void exit_pairing_mode(void) {
  if (!pairing_mode)
    return;

  printf("[PAIRING] Exiting pairing mode\n");
  pairing_mode = false;

  // Stay connectable but not discoverable (for known devices)
  gap_discoverable_control(0);
  gap_connectable_control(1);

  // Stop pairing timer
  btstack_run_loop_remove_timer(&pairing_timer);
}

// ============================================================================
// Main
// ============================================================================
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

  // Initialize A2DP Sink
  bt_audio_sink_init();

  // Play startup tone
  printf("[MAIN] Playing startup tone...\n");
  i2s_play_test_tone();

  // Setup LED heartbeat timer
  btstack_run_loop_set_timer_handler(&led_timer, led_heartbeat_handler);
  btstack_run_loop_set_timer(&led_timer, 250);
  btstack_run_loop_add_timer(&led_timer);

  // Turn on Bluetooth
  hci_power_control(HCI_POWER_ON);

  // Keep device discoverable and connectable for auto-reconnect
  // macOS needs the device to be visible for auto-reconnect to work
  gap_discoverable_control(1);
  gap_connectable_control(1);

  // Schedule auto-reconnect to last connected device
  bt_audio_sink_schedule_reconnect();

  printf("Bluetooth powered on - discoverable and connectable\n");
  printf("[LED] Fast blink=pairing, Medium=waiting, Slow=connected\n");

  // Run BTstack event loop (this blocks forever)
  btstack_run_loop_execute();

  return 0;
}
