/*
 * servo_pins.h — servo outputs 1..8 of every receiver role, DUAL build (branch dual-serial).
 *
 * The dual build has the SX1278 (RA-01) on PC06 PC07 PC08 PC09 and PD03, the pins the single build uses for
 * servos 2 and 5..8. Here those five servos move to module pads the Multi_Mode carrier leaves unconnected;
 * servos 1, 3 and 4 stay where they were. A receiver board for the dual build must route these to its header.
 *
 *   servo   1     2     3     4     5     6     7     8
 *   pin     PA07  PB04  PD04  PD05  PA00  PA08  PA09  PB05
 *   module  14    2     20    19    9     15    16    1
 *
 * PB04 is the SX1278's DIO0 on the schematic; the dual build does not use DIO0 (it reads the SX1278 over SPI).
 * Unchanged: PPM PC03, SBUS PA03, RLINK battery sense PA04.
 */
#ifndef SERVO_PINS_H
#define SERVO_PINS_H
#include "sl_gpio.h"

#define SERVO_PINS_INIT { \
  { .port = SL_GPIO_PORT_A, .pin = 7 }, { .port = SL_GPIO_PORT_B, .pin = 4 }, \
  { .port = SL_GPIO_PORT_D, .pin = 4 }, { .port = SL_GPIO_PORT_D, .pin = 5 }, \
  { .port = SL_GPIO_PORT_A, .pin = 0 }, { .port = SL_GPIO_PORT_A, .pin = 8 }, \
  { .port = SL_GPIO_PORT_A, .pin = 9 }, { .port = SL_GPIO_PORT_B, .pin = 5 } }

#endif /* SERVO_PINS_H */
