/*
 * board.h — what differs between the boards (board_config.h picks one). Everything not named here is the same pin
 * on both: PPM in / telemetry out PB01, BIND PB00, RANGE PB02, F/S PB03, protocol jumpers PC00..PC03, PPM out PC03,
 * LED_A PC04, LED_B PC05, SBUS out PA03, receiver battery sense PA04, buzzer PD02.
 *
 * Seeed XIAO MG24 (EFR32MG24B220F1536IM48; pins from Seeed's wiki and Zephyr's board files, 26 Nov 2026):
 *   pad  D0   D1   D2   D3   D4   D5   D6   D7   D8   D9   D10  D13  D14  D15  D16  D17  D18
 *   pin  PC00 PC01 PC02 PC03 PC04 PC05 PC06 PC07 PA03 PA04 PA05 PB02 PB03 PB00 PB01 PA00 PD02
 *   on the board itself: PA08 / PA09 = the USB serial bridge (chip TX / RX), PA07 = the yellow LED (low = on),
 *   PB05 = power of the antenna switch (high = on), PB04 = antenna (low = ceramic, high = U.FL), PD04 = battery
 *   sense, PA03 PA04 PA05 PA06 = the 4 MB flash chip (PA06 = its chip select, held high here: the chip stays off
 *   the three shared pads).
 *   Differences from the module build:
 *     - log and command line ("cfg ...") on the USB cable (EUSART0 on PA08 / PA09), 115200 8N1;
 *     - the signal input (MULTI frames from a radio or an encoder, or PPM) on D7 = PC07 in place of PA06. The
 *       command line no longer shares its pin with the frames, so it keeps listening in every mode;
 *     - the yellow LED follows LED_B (PC05); LEDs on D4 / D5 work as on the module;
 *     - no 459 MHz radio, no TX panel, no bind plug on servo connector 8, no alarm level on PA07;
 *     - receiver roles: SBUS on D8 (PA03) and PPM on D3 (PC03) in every output layout; no PWM servo outputs
 *       (the module's servo pins are the LED, the antenna switch, the battery sense and the USB bridge here);
 *     - crystal trim (CTUNE) 95 in place of the module's 70 (config/sl_clock_manager_oscillator_config.h).
 */
#ifndef BOARD_H
#define BOARD_H
#include "board_config.h"
#include "sl_gpio.h"

#if BOARD == BOARD_XIAO_MG24
#define BOARD_NAME          "Seeed XIAO MG24"
#define BOARD_IN_PORT       SL_GPIO_PORT_C     /* the signal input: MULTI serial / PPM */
#define BOARD_IN_PIN        7u
#define BOARD_IN_NAME       "D7 (PC07)"
#define BOARD_LOG_SHARES_IN 0                  /* the command line has its own pin (USB) */
#define BOARD_HAS_SX1278    0
#define BOARD_HAS_PANEL     0
#define BOARD_HAS_BIND_PLUG 0
#define BOARD_HAS_ALARM_PIN 0
#define BOARD_SERVO_MASK    0x00u              /* PWM servo outputs that have a pin (bit 0 = servo 1) */
#define BOARD_FIXED_SBUS_PPM 1                 /* SBUS on PA03 and PPM on PC03 whatever the output layout */
#else
#define BOARD_NAME          "RF-BM-MG24 module"
#define BOARD_IN_PORT       SL_GPIO_PORT_A
#define BOARD_IN_PIN        6u
#define BOARD_IN_NAME       "J3 RX (PA06)"
#define BOARD_LOG_SHARES_IN 1                  /* the command line listens on the signal input pin */
#define BOARD_HAS_SX1278    1
#define BOARD_HAS_PANEL     1
#define BOARD_HAS_BIND_PLUG 1
#define BOARD_HAS_ALARM_PIN 1
#define BOARD_SERVO_MASK    0xFFu
#define BOARD_FIXED_SBUS_PPM 0
#endif

/* first thing at power-up (app_init) */
static inline void board_early_init(void)
{
#if BOARD == BOARD_XIAO_MG24
  static const sl_gpio_t sw_pwr = { .port = SL_GPIO_PORT_B, .pin = 5 }, sw_sel = { .port = SL_GPIO_PORT_B, .pin = 4 },
                         fl_cs = { .port = SL_GPIO_PORT_A, .pin = 6 }, led = { .port = SL_GPIO_PORT_A, .pin = 7 };
  sl_gpio_set_pin_mode(&sw_sel, SL_GPIO_MODE_PUSH_PULL, XIAO_ANTENNA_UFL ? 1 : 0);   /* the antenna, */
  sl_gpio_set_pin_mode(&sw_pwr, SL_GPIO_MODE_PUSH_PULL, 1);                           /* ... then the switch on: no RF without it */
  sl_gpio_set_pin_mode(&fl_cs,  SL_GPIO_MODE_PUSH_PULL, 1);                           /* flash chip deselected */
  sl_gpio_set_pin_mode(&led,    SL_GPIO_MODE_PUSH_PULL, 1);                           /* LED off (low = on) */
#endif
}

/* every main-loop pass: the XIAO's own LED shows LED_B (PC05) */
static inline void board_service(void)
{
#if BOARD == BOARD_XIAO_MG24
  static const sl_gpio_t led_b = { .port = SL_GPIO_PORT_C, .pin = 5 }, led = { .port = SL_GPIO_PORT_A, .pin = 7 };
  bool on = false;
  sl_gpio_get_pin_output(&led_b, &on);
  if (on) sl_gpio_clear_pin(&led); else sl_gpio_set_pin(&led);
#endif
}
#endif
