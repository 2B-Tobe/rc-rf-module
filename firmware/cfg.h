/*
 * cfg.h — the module's stored setup: role, protocol and option, so a board needs no protocol jumpers.   v1.0
 *
 * One record in its own flash page (0x08042000, next to the bind store), written over the log UART (J3: PA06
 * in, PA05 out) with the "cfg" command — from PuTTY today, from the programmer box later. A firmware update
 * leaves the page alone, so a board is set up once.
 *
 *   cfg                      show the stored setup and what is running now
 *   cfg tx <proto> [dual]    transmitter; proto nrf24 rlink uk d8 d16 dsmx jeti txtest (or Prot Sub numbers);
 *                            dual = 459 MHz link beside it (PPM input; with the encoder its Opt decides)
 *   cfg rx <proto> [layout]  receiver; proto nrf24 rlink uk d8 d16 dsmx dsmcap jeti; layout 8pwm (default) 5sbus
 *                            5ppm 5both: SBUS on servo connector 6, PPM on connector 7 (or 0..3)
 *   cfg clear                erase: the jumpers decide again, as before
 *   cfg bind                 restart with BIND counted as held for 8 s (bind without the jumper; a receiver that
 *                            links inside those 8 s also stores its failsafe positions, as with the jumper)
 *   The module restarts after a change.
 *
 * What the stored setup means at power-up (see proto_start in app_process.c):
 *   nothing stored   exactly as before: F/S = receiver, RANGE at power-up = encoder serial, PC00..PC02 = protocol
 *   TX stored        F/S low  = serial control (everything from the MULTI frames, as the bay mode before)
 *                    F/S high = whichever comes first: the encoder's MULTI frames (encoder control) or PPM
 *                               (protocol and single/dual from the setup) - no jumper needed for either
 *   RX stored        F/S low  = as before: PC00..PC02 choose the protocol
 *                    F/S high = protocol from the setup
 *   BIND and RANGE keep their meaning in every case (bind at power-up, hold BIND = store failsafe; range check).
 *
 * Protocol numbers are the MULTI frame's Prot / Sub, so one table serves the frames and the setup.
 * #included into app_process.c.
 */
#ifndef CFG_H
#define CFG_H
#include <stdint.h>
#include <stdbool.h>

#define CFG_ROLE_TX 1u
#define CFG_ROLE_RX 2u

typedef struct {
  uint8_t role;      /* CFG_ROLE_TX / CFG_ROLE_RX */
  uint8_t proto;     /* Prot */
  uint8_t sub;       /* Sub */
  uint8_t option;    /* transmitter with PPM input: 0 = single (2.4 GHz only), 1 = dual (459 MHz beside) */
  uint8_t outputs;   /* receiver: output layout, servo_out.h RXOUT_* (0 = 8 PWM, as before) */
} cfg_t;

bool cfg_load(cfg_t *c);                      /* true = a valid setup is stored */
bool cfg_save(const cfg_t *c);
bool cfg_clear(void);
bool cfg_bind_pending(void);                  /* a "cfg bind" asked for a bind at this power-up ... */
void cfg_bind_consume(void);                  /* ... take it (one-shot) */
const char *cfg_proto_name(uint8_t role, uint8_t proto, uint8_t sub);
bool cfg_owner_known(const uint8_t id[6]);    /* a TX module allowed to set this receiver up over the air (rx_setup.c) */
bool cfg_owner_add(const uint8_t id[6]);      /* pairing: up to 3, a 4th pushes the oldest out */
uint8_t cfg_owner_count(void);   /* "RLINK", "UK receiver", ... NULL = not in this module */
void cfg_cli_init(void);                      /* registers the "cfg" command on the log UART's command line */
#endif
