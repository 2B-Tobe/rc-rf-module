/*
 * sbus_out.h — Futaba-style SBUS output, shared by the receiver roles.
 * 100 kbaud 8E2, inverted (idle low), 25-byte frame on PA03 (EUSART1: free in every receiver role, since
 * the encoder's serial input exists in the transmitter role only). The log stays on EUSART0 / PA05.
 * Channels 1-8 come from the protocol, 9-16 sit at centre; the flags byte
 * carries frame-lost and failsafe. Bytes are pushed from the main loop, so
 * nothing blocks: sbus_send() queues a frame, sbus_service() feeds the FIFO.
 */
#ifndef SBUS_OUT_H
#define SBUS_OUT_H
#include <stdint.h>
#include <stdbool.h>

#ifndef SBUS_ENABLE
#define SBUS_ENABLE 1
#endif
#define SBUS_PORT SL_GPIO_PORT_A
#define SBUS_PIN  3

void sbus_init(void);                                   /* RX roles only, once at start */
void sbus_set_pin(uint8_t port, uint8_t pin);           /* before sbus_init: another pin (servo connector 6 = PA08) */
bool sbus_send(const uint16_t us[8], bool frame_lost, bool failsafe);  /* false = still busy */
void sbus_service(void);                                /* call every main-loop pass */
bool sbus_idle(void);                                   /* true = no frame going out */
extern volatile int32_t sbus_frames;                    /* frames handed to the UART */
#endif
