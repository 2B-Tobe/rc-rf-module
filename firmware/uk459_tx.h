/*
 * uk459_tx.h — second radio for the EFR32MG24 module: an SX1276/78 (RA-01 / RA-02) on the pins the carrier
 * board reserves for it, sending Tobe's UK 459 MHz hopping link beside the 2.4 GHz transmitter (uk459_tx.c, v1.4, DUAL build).
 *
 * On the air it is the GFSK_Tx_UK459_HOP V9 transmitter (ATmega328 + SX1278), register for register, so the
 * existing GFSK_Rx_UK459_HOP V7 receivers and satellites work unchanged -- after one bind to the module's own ID.
 *
 * Wiring (Multi_Mode schematic labels; RA-01 pin names):
 *   PC06 SCK -> SCK      PC07 SDI -> MOSI     PC08 SDO <- MISO     PC09 NSS-nSEL -> NSS
 *   PD03 RESET-SDN -> RESET (pulsed low at power-up, then left open)      3.3 V and GND.
 *   DIO0, DIO1, DIO2 stay unconnected: "packet sent" is read over SPI. (PB04, PD04, PD05 are servo outputs
 *   2, 3, 4 in this build, servo_pins.h.)
 *
 * Roles:
 *   nRF24, FrSky D8, FrSky D16, DSMX transmitter                      -> the UK link sends all the time
 *   RLINK transmitter                                                 -> the UK link sends only while RLINK's link
 *     alarm is on -- 2 beeps (weak link) or continuous (link lost) -- unconditionally until RLINK's RSSI / link
 *     quality is back, plus UK_GATE_HOLD_MS. Failsafe stays with the receiver (only when both links are lost).
 *     Wake-up matched to the V7 receiver, which parks on hop position 0 after 1.2 s without packets:
 *       silent < 0.6 s          -> carry on the hop schedule (the receiver is still hopping roughly in step)
 *       silent >= 1.25 s        -> restart at hop 0 at once: data within a packet, beacon sync within a dwell
 *       in between              -> wait until the receiver has parked (at most 0.65 s), then restart at hop 0
 *     (RLINK's 3-beep RX-battery alarm does not wake it.)
 *   DSM capture, TX test, receiver roles                              -> SX1278 asleep (receiver roles: until the
 *     dual receiver is added; their servo outputs are on other pins in this build, servo_pins.h)
 *   No SX1278 answering (version register not 0x12)                    -> 2.4 GHz only, and the log says so.
 *
 * Bind:  every power-up sends UK_AUTO_BIND_MS (1.5 s) of bind packets at bind power on 458.5375 MHz, as V9 does;
 *        BIND held at power-up keeps them going while BIND is held and UK_AUTO_BIND_MS after release. The bind
 *        packet carries the ID and the hop lane; the receiver stores both. The 2.4 GHz role binds by its own rules
 *        at the same time, so one BIND power-up binds both receivers when both are waiting in bind.
 * Lane:  PC03 (J1 PROTOCOL_03) at power-up: open = even lane (20 channels), to GND = odd lane (19 channels).
 *        Taught at bind: after changing it, rebind the UK receiver.
 * RANGE: low = range power on this link too, followed at any time, like the 2.4 GHz side.
 * Encoder control (v1.4, RANGE jumper at power-up, app_process.c / bay_serial.c): the channels, BIND and RANGE come
 *        from the serial frames for this link too -- their BIND binds it, and keeps it binding while BIND is
 *        on; their RANGE is its range check. The UK ID then includes the frames' Rx No (per-model
 *        binding): bind each model's UK receiver once. Rx No 0, or no encoder, keeps the ID as before. The lane
 *        stays on the PC03 jumper.
 * Failsafe: set on the 2.4 GHz side; no failsafe-capture packet here. Without PPM for 500 ms no data packets go
 *        out, so the UK receiver falls into its own failsafe, as with V9.
 *
 * Bind packets go out in every transmitter role, RLINK included: bind is at power-up, not in flight.
 *
 * UART PA05: "UK boot uk459_tx v1.4 ... (model n) ...", "UK data ...", "UK RANGE on/off ...", "UK on/standby ..." (RLINK),
 *   every 10 s "UK s tx.. bcn.. ..".
 */
#ifndef UK459_TX_H
#define UK459_TX_H

#include <stdbool.h>
#include <stdint.h>
#include "sl_rail.h"

bool uk_probe(sl_rail_handle_t rh);   /* power-up, every role: reset the SX and read its version; true = fitted */
void uk_start(const volatile uint16_t *ch_us, bool bind_held, bool range_low);   /* transmitter roles */
void uk_sleep(void);                  /* fitted, but this role does not use it */
/* every main-loop pass; want_data = false keeps the data packets back (RLINK with a good link) */
void uk_service(uint32_t now, bool ppm_fresh, bool bind_low, bool range_low, bool want_data);

#define UK_DIAG_N  10u
extern volatile int32_t uk_diag[UK_DIAG_N];
#define UK_VERSION  0   /* SX version register read at power-up (0x12 = SX1276/77/78/79)            */
#define UK_PHASE    1   /* 0 off, 1 bind, 2 data                                                       */
#define UK_TX       2   /* packets sent                                                                */
#define UK_BEACON   3   /* sync beacons sent (one per hop)                                             */
#define UK_HOP_LATE 4   /* hops where "packet sent" was not seen by the next 40 ms slot (retuned anyway) */
#define UK_NO_PPM   5   /* 40 ms slots without a data packet because there was no PPM                  */
#define UK_HOP      6   /* current hop index                                                           */
#define UK_RANGE    7   /* 1 while RANGE is low                                                        */
#define UK_ACTIVE   8   /* 1 while data packets go out, 0 while held back (RLINK link good)             */
#define UK_QUIET    9   /* 40 ms slots held back, schedule kept running                                */

#endif /* UK459_TX_H */
