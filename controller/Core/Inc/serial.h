/*
 * serial.h - the serial link to the RF module: text out on USART1 (PA9), the module's answer in on LINK_RX_PIN.
 *
 * LINK_RX_PIN  8   PA8: a software receiver. An edge interrupt catches the start bit, then TIM2 samples the
 *                  line in the middle of every bit (72 MHz, 625 ticks per bit at 115200). The level the pin rests
 *                  at before a command is taken as idle, so an inverting input stage works as well as a direct
 *                  one, and slow edges do not matter. Default: PA8 is a free timer pin on this board.
 *             10   PA10: USART1's own receiver, shared with the DEC key. Needs a series resistor (1 kOhm) in the
 *                  wire from the module so the key can pull the line low. Keys are read over three scans, so the
 *                  serial traffic never looks like a key press.
 *              3   PA3: USART2's receiver, a pin of its own.
 */
#ifndef SERIAL_H
#define SERIAL_H
#include <stdint.h>
#include <stdbool.h>

#ifndef LINK_RX_PIN
#define LINK_RX_PIN 8
#endif

void     serial_init(void);
void     serial_send(const char *text);     /* blocking, 115200 8N1 on PA9 */
void     serial_listen(void);               /* clear the receive buffer and arm the receiver for an answer */
int      serial_read(void);                 /* next received byte, -1 = none */
uint32_t serial_frames(void);               /* bytes started since serial_listen (diagnostic) */
uint32_t serial_errors(void);               /* bytes without a stop bit since serial_listen (diagnostic) */
bool     serial_inverted(void);             /* PA8 receiver: the line idles low (inverting input stage) */
#endif
