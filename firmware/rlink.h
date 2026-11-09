/*
 * rlink.h — RLINK, resilient two-way 2.4 GHz RC link (EFR32MG24).
 * #included into app_process.c like the other protocols.
 * Specified in "RLINK Protocol Specification" and "RLINK System Specification".
 *
 * Boot jumpers:  PC02 low                = RLINK transmitter
 *                PC02 + F/S low          = RLINK receiver
 *                + BIND held on both     = private bind (RX first)
 *                TX + RANGE low          = range check, fixed -10 dBm
 *                RX + RANGE low          = bench test: RX ignores copy A on RF 12..20
 *   TX outputs:  PD02 buzzer beeps alarms; PA07 high = alarm (for the encoder);
 *                bay UART PA05 115200: "RL q.. a.. b.. u.. p.. m.. s.. i.. l.. v.." ~5.7x/s
 *                beeps: 2 = weak link, 3 = RX battery low, continuous = land now
 *                (RX battery 4-cell/5 V: 4.6 / 4.4 V, 2S LiPo: 7.2 / 6.8 V, picked from the first reading)
 *   Every transmission uses listen-before-talk: busy channel -> packet skipped.
 *   RX outputs:  PPM PC03, 8 servo pulses (servo_pins.h), SBUS on PA03 (100 kbaud 8E2, inverted)
 *   RX input:    battery on PA04 via 47k/10k divider from Vbatt (none fitted -> no sensor)
 *   RX:          failsafe preset = hold BIND 0.3 s while linked
 */
#ifndef RLINK_H
#define RLINK_H
#include <stdint.h>
#include <stdbool.h>
#include "sl_rail.h"

void rlink_start(sl_rail_handle_t rh, const volatile uint16_t *ch_us, bool rx_role, bool range_low, bool bind_held);
void rlink_action(void);
void rlink_on_events(sl_rail_events_t events);
void rlink_rx_process(void);
void rlink_input(bool ok);   /* TX: call every loop with "encoder PPM present" */
void rlink_range(bool low);  /* TX: call every loop with the RANGE input (low = -10 dBm range check) */
void rlink_bind(bool held);  /* TX: call every loop; true (BIND on in the serial frames) keeps the bind going.
                                The bind at power-up lasts RL_BIND_MS (1.5 s) at least, and then as long as this
                                stays true. Jumper mode passes false: 1.5 s, as before. */

/* TX, for the encoder's telemetry: microseconds since the last telemetry packet from the receiver, and whether
   one has been heard at all since this role started. */
uint32_t rlink_tele_age_us(uint32_t now);
bool     rlink_ever_linked(void);

#define RL_DIAG_N 58u
extern volatile int32_t rlink_diag[RL_DIAG_N];
#define R_ROLE   0
#define R_INLOST 1   /* TX: 1 = encoder PPM lost / RX: frames received flagged "TX input lost" */
#define R_SEQ    2
#define R_FTX    3
#define R_TXOK   4
#define R_TXER   5
#define R_BIND_SEEN 6
#define R_FRX    7
#define R_CRX    8
#define R_FLOST  9
#define R_CRCB  10
#define R_CH0   11
#define R_CH1   12
#define R_LQ    13
#define R_RSSI  14
#define R_WORST 15
#define R_GOT_A 16
#define R_SAVED_B 17
#define R_LATE  18
#define R_PWR   19
#define R_SYNC_NOW 20   /* sync word the radio is actually using right now        */
#define R_SYNC_ST  21   /* status of the last sync-word write (0 = OK)             */
#define R_SYNC_FIX 22   /* times the sync word had been reset and was re-applied   */
#define R_BIND_TX  23   /* TX: bind packets sent                                   */
#define R_STATE    24   /* bits: 1 rxbind, 2 acq, 4 saved bind, 8 provisional,
                           16 TX bind burst, 32 BIND pin reads low, 64 LBT refused (sent plain)                */
#define R_RSSI_B   25   /* RX: RSSI of the last copy-B packet, dBm ([14] = copy A)  */
/* R2 telemetry */
#define R_T_ASK    26   /* TX: telemetry windows opened / RX: telemetry replies sent */
#define R_T_GOT    27   /* TX: telemetry packets received                           */
#define R_T_LQ     28   /* TX: RX link quality, %                                   */
#define R_T_RSSIA  29   /* TX: RSSI at the RX, copy A, dBm                          */
#define R_T_RSSIB  30   /* TX: RSSI at the RX, copy B, dBm                          */
#define R_T_SAVEDB 31   /* TX: RX frames saved by copy B (16-bit, wraps)            */
#define R_T_LOST   32   /* TX: RX frames lost (16-bit, wraps)                       */
#define R_T_CRCB   33   /* TX: RX CRC errors (16-bit, wraps)                        */
#define R_T_FLAGS  34   /* TX: bit0 RX in failsafe, bit1 provisional, bits 8-15 worst dropout /10 ms */
#define R_T_RSSI_UP 35  /* TX: RSSI of the telemetry packet at the TX, dBm          */
/* R3a channel blacklisting (both roles) */
#define R_MAP_VER  36   /* active channel map version (0 = nothing blacklisted)     */
#define R_MAP_NA   37   /* blacklisted A channels                                   */
#define R_MAP_NB   38   /* blacklisted B channels                                   */
#define R_MAP_SW   39   /* map switches done                                        */
#define R_MAP_RST  40   /* map resets (sides disagreed -> back to empty map)        */
#define R_MAP_A_LO 41   /* blacklist A, bit i = RF channel i+2  (bits 0..31)        */
#define R_MAP_A_HI 42   /*                                      (bits 32..38)       */
#define R_MAP_B_LO 43   /* blacklist B, bit i = RF channel i+42 (bits 0..31)        */
#define R_MAP_B_HI 44   /*                                      (bits 32..38)       */
#define R_MAP_PEND 45   /* RX: proposed version waiting / TX: version being announced */
#define R_T_MAPS   46   /* TX: map proposals received                               */
/* R3b automatic TX power (TX) */
#define R_PC_REQ   47   /* power the controller asks for, ddBm ([19] = what RAIL set) */
#define R_PC_UPS   48   /* jumps to full power (weak signal, low LQ, telemetry lost) */
#define R_PC_DOWNS 49   /* 2 dB steps down                                          */
#define R_PC_RSSI  50   /* RSSI at the RX the controller last acted on, dBm         */
/* R3c alarms (TX) */
#define R_ALARM    51   /* 0 ok, 1 warning (weak link), 2 critical (no telemetry / RX failsafe) */
/* R4 listen-before-talk (both roles) */
#define R_LBT_BUSY 52   /* packets skipped because the channel was busy          */
#define R_SBUS     54   /* RX: SBUS frames sent on PA03                         */
#define R_LBT_TL   53   /* busy threshold used for the last transmission, dBm   */
#define R_VBAT     55   /* RX battery, mV (RX: measured / TX: from telemetry), 0 = no sensor */
#define R_VB_ALM   56   /* TX: RX battery alarm 0/1 low/2 critical / RX: 0 no sensor, 1 probing, 2 measuring */
#endif
