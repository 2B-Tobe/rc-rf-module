/*
 * servo_out.h — the one servo / PPM output stage of every receiver role (DUAL build).   v1.1
 *
 * 8 servo pulses (servo_pins.h) and the PPM frame on PC03, made by the compare outputs of TIMER2, TIMER3 and
 * TIMER4 (1 MHz): the pin edges are hardware events at the programmed times; the one interrupt (TIMER2 CC0,
 * highest priority) runs after each PPM edge and arms the next edges 300..2200 us ahead. Neither RAIL's timer
 * nor interrupt latency can move an edge. The timing is the one the roles always had: 22.5 ms frame, 300 us PPM
 * pulses, servo n high from PPM pulse n to PPM pulse n+1 (= channel n's width), ~44 Hz.
 *
 * A role hands over the array it decodes into (servo_out_source) and switches the pulses on and off
 * (servo_out_enable); the pulses read the array live, as the roles' own callbacks did.
 * Dual receiver: app_process.c can point the output at another array (servo_out_override): the 459 MHz
 * receiver's channels while the 2.4 GHz link is away. The pulses run as long as either the role has them on
 * or an override is set, so a "no pulses" failsafe of the 2.4 GHz role does not silence the 459 MHz data.
 * sbus_out.c takes its channels from servo_out_current() so SBUS follows the same choice.
 *
 * v1.2 output layouts (servo_out_layout, before servo_out_init): servo connector 6 (PA08) can carry SBUS and
 * connector 7 (PA09) the PPM frame instead of servo 6 / 7, so a receiver needs no pads beyond its 8 servo
 * connectors. Layout 0 is the one every receiver had: 8 servos, PPM on PC03, SBUS on PA03.
 * servo_out_hold: a connector is kept an input (the bind plug on connector 8 at power-up) until it is let go.
 *
 * #included into app_process.c before sbus_out.c and the receiver roles.
 */
#ifndef SERVO_OUT_H
#define SERVO_OUT_H
#include <stdint.h>
#include <stdbool.h>

#define RXOUT_SBUS  1u                                      /* layout bit: SBUS on connector 6 (PA08) */
#define RXOUT_PPM   2u                                      /* layout bit: PPM on connector 7 (PA09) */
#define RXOUT_MAX   3u                                      /* 0 = 8 PWM, 1 = 5 + SBUS, 2 = 5 + PPM, 3 = 5 + SBUS + PPM */
void servo_out_layout(uint8_t layout);                     /* before servo_out_init; 0 = as before */
void servo_out_hold(uint8_t servo, bool hold);             /* servo 0..7: keep its connector an input (true) / output it */
const char *servo_out_layout_name(uint8_t layout);         /* "8 PWM", "5 PWM + SBUS", ... */
void servo_out_init(void);                                 /* pins and TIMER2; safe to call more than once */
void servo_out_source(const volatile uint16_t *us8);       /* the role's channels 1..8 in us */
void servo_out_enable(bool on);                            /* the role's pulses on / off (off: servos low, PPM high) */
void servo_out_override(const volatile uint16_t *us8);     /* dual: output these instead; NULL = the role's again */
const volatile uint16_t *servo_out_current(void);          /* the array being output right now (NULL = none) */
bool servo_out_overridden(void);
void servo_out_report(void);                               /* log line: frames, late interrupts, channel jumps (every 2 s) */
#endif
