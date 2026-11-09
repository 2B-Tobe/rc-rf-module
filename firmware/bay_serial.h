/*
 * bay_serial.h — encoder / EdgeTX serial control for the transmitter roles (bay_serial.c, v1.5).
 *
 * A transmitter or encoder with Multiprotocol serial output sends a "MULTI" frame every 7 to 22 ms:
 * 100000 baud, 8 data bits, even parity, 2 stop bits, NOT inverted (EdgeTX: inverted, below), 26 or 27 bytes:
 *   [0]     0x55 = protocol bit 5 clear, 0x54 = protocol bit 5 set
 *   [1]     protocol bits 0..4 | 0x20 range check | 0x40 autobind | 0x80 bind
 *   [2]     RX number (bits 0..3) | sub protocol << 4 (bits 4..6) | 0x80 low power
 *   [3]     option (signed)
 *   [4..25] 16 channels x 11 bits, little-endian; 204 / 1024 / 1844 = 1000 / 1500 / 2000 us
 *   [26]    the Multiprotocol "V2" frame, 27 bytes: protocol bits 6..7 in bits 6..7, RX number bits 4..5 in
 *           bits 4..5, bit 2 (reserved in Multiprotocol) = the sender listens for the 16-byte telemetry
 *           frame on the PPM pin (v1.3, below), bit 3 = RxSetup (v1.4, rx_setup.h) with the receiver's output
 *           layout in bits 0..1, the rest 0. Bits 2 and 3 are this module's extensions, read only from a
 *           normal-polarity sender; from EdgeTX byte 26 is read as in Multiprotocol.
 * A 26-byte frame is the classic one, protocols 0..63; the 27th byte carries protocols 64 and up and the bits
 * above. Every frame carries Prot, Sub, Rx No, Opt and BIND / RANGE.
 *
 * Wiring: the sender's serial output -> module PA06 (pad 13; J3 "RX" on the Multi_Mode carrier, through its 1 k),
 * common GND. Both run at 3.3 V. PA06 is the log UART's RX: it is taken over here, so the log adapter's TX wire
 * must come off J3 RX (keep its RX on J3 TX = PA05 and GND; the log is unchanged). The (unused) CLI stops
 * listening in the transmitter roles.
 *
 * The module uses EUSART1 for this (RX on PA06); EUSART0 stays the log UART on PA05.
 *
 * Telemetry back (v1.3): in serial mode the module answers every good frame THAT ASKS FOR IT (byte 26 bit 2)
 * with a 16-byte telemetry frame on PB01 - the PPM input, idle while the sender sends serial frames, and back to
 * the sender through the bay. EUSART1's TX is routed there (same 100000 8E2), so the two directions are
 * independent; the PPM capture on PB01 is switched off in serial mode. A sender that does not ask never sees
 * the pin driven; the line is taken only after the first asking frame
 * and let go (back to an input) when the frames stop, so the module never drives PB01 against a PPM output.
 *   [0]  0xA5 header
 *   [1]  flags: 0x01 link (the receiver has been heard lately), 0x02 bound / data phase, 0x04 binding,
 *        0x08 range check, 0x10 RX battery sensor present, 0x20 UK 459 MHz link running beside this mode (dual),
 *        0x40 UK link sending right now (the 2.4 GHz link needs it),
 *        0x80 receiver reports failsafe
 *   [2]  protocol number the module runs (as the encoder sent it)
 *   [3]  sub protocol (bits 0..3) | link alarm 0 ok 1 weak 2 lost (bits 4..5) | RX battery alarm 0 ok 1 low
 *        2 critical (bits 6..7)
 *   [4]  TX power, dBm, signed
 *   [5]  signal at the receiver: RLINK dBm signed; FrSky the receiver's RSSI figure 0..100; 0x7F = none
 *   [6]  signal at the module (the receiver's telemetry), dBm signed; 0x7F = none
 *   [7]  link quality %, 0..100; 0xFF = none
 *   [8]  RX battery, 10 mV units, little-endian low byte (0 = no sensor)
 *   [9]  RX battery high byte
 *   [10] frames lost at the receiver, low byte of a running count (RLINK), else 0
 *   [11] bay version (13 = v1.3)
 *   [12] bit0: [5] is in dBm (RLINK) rather than a figure; bit1: [6] valid; bit2: the module does not have the
 *        Prot / Sub the encoder asked for (RF off)
 *   [13] UK hop index (dual), else 0
 *   [14] sequence, one up per reply
 *   [15] CRC-8, polynomial 0x31, init 0, over bytes 0..14
 * EdgeTX (v1.5, etx_multi.h): an EdgeTX radio with the module in its JR bay, type MULTI, sends the same frames
 * INVERTED (idle low) on bay pin 1, every 7 ms, always 27 bytes or more, and listens on bay pin 5 (S.Port) for the
 * Multiprotocol telemetry ("MP" frames). The module tells the two apart by the polarity alone: read the wrong way
 * up, either stream gives only framing / parity errors, never a good frame. bay_detect tries normal, then inverted,
 * every BAY_POL_TRY_MS until frames check out. The frame rate decides too (v1.5a): frames under 12 ms apart are
 * EdgeTX whatever their polarity (a radio whose bay hardware inverts the line once more), 22 ms the encoder. The
 * first 2 frames after a silence are held back until the rate is known; the frames then carry f->edgetx. Byte 26 means something else there:
 * bits 6..7 protocol bits 6..7, bits 4..5 RX number bits 4..5, bit 3 telemetry inverted, bit 1 telemetry off,
 * bit 0 channel mapping off. The same two pins carry it: bay pin 1 -> J3 RX (PA06), bay pin 5 -> PB01.
 *
 * A frame counts when all its bytes arrive without a parity, framing or overrun error, the header is 0x55 / 0x54,
 * and a gap of more than BAY_GAP_US separates it from the previous frame. The 26th byte completes a frame unless
 * a 27th follows within BAY_V2_WAIT_US (one byte time plus margin): the receiver waits that long for it.
 */
#ifndef BAY_SERIAL_H
#define BAY_SERIAL_H

#include <stdbool.h>
#include <stdint.h>
#include "sl_rail.h"

typedef struct {
  uint8_t  proto;        /* 0..255 ("Prot"; 64 and up come in the 27-byte frame)        */
  uint8_t  sub;          /* 0..7  ("Sub")                                           */
  uint8_t  rxnum;        /* 0..15 ("Rx No"; EdgeTX 0..63)                           */
  int8_t   option;       /* -128..127 ("Opt")                                       */
  bool     bind, range, autobind, lowpower;
  bool     failsafe;     /* a failsafe frame (header 0x56 / 0x57): us[] are the positions the receiver shall store */
  bool     telem;        /* the encoder listens for telemetry on its PPM pin (27-byte frame, byte 26 bit 2) */
  bool     rxsetup;      /* RxSetup: send receiver setup packets (byte 26 bit 3, rx_setup.h) */
  uint8_t  rxlayout;     /* ... the receiver's output layout (byte 26 bits 0..1) */
  bool     edgetx;       /* the frame came inverted: an EdgeTX radio (proto / sub / option are then translated, below) */
  uint8_t  wproto, wsub; /* protocol and sub protocol as sent (EdgeTX: the Multiprotocol numbers; encoder: = proto, sub) */
  bool     scan;         /* EdgeTX protocol 0: the radio asks for the protocol list, entry scan_idx (0xFF = the count) */
  uint8_t  scan_idx;
  bool     tel_inv;      /* EdgeTX byte 26 bit 3: telemetry goes back inverted (S.Port style, idle low) */
  uint16_t us[16];       /* channel pulse widths, us (exact inverse of the encoder) */
  uint32_t t_us;         /* RAIL time the frame completed                           */
  uint32_t seq;          /* 1, 2, 3 ... per good frame                              */
} bay_frame_t;

/* Transmitter roles, at power-up: EUSART1 RX on PA06, 100000 8E2. on_frame runs in the RX interrupt
 * (priority BAY_IRQ_PRIO) for every good frame. */
void bay_init(sl_rail_handle_t rh, void (*on_frame)(const bay_frame_t *f));
/* Latest good frame; false while none has arrived. */
bool bay_get(bay_frame_t *f);
/* Frame bytes -> fields (also used by the host test). n = 26 or 27 bytes; etx = the frame came inverted (EdgeTX). */
void bay_decode(const uint8_t *b, uint8_t n, bool etx, bay_frame_t *f);
/* Polarity of the bay serial: false = normal (encoder), true = inverted (EdgeTX). Both directions follow rx_inv
 * unless bay_tx_polarity sets the telemetry's own. A change restarts the receiver: a frame under way is dropped. */
void bay_set_inverted(bool rx_inv);
bool bay_inverted(void);
void bay_tx_polarity(bool tx_inv);

/* ---- telemetry back to the encoder ---- */
#define BAY_TELEM_VERSION  13u
#define BAY_NO_RSSI        0x7F
#define BAY_NO_LQ          0xFF
typedef struct {
  bool     link, bound, binding, range, vbat_sensor, uk_link, uk_active, failsafe, unsupported;
  uint8_t  proto, sub;
  uint8_t  link_alarm;     /* 0 ok, 1 weak, 2 lost */
  uint8_t  vbat_alarm;     /* 0 ok, 1 low, 2 critical */
  int8_t   power_dbm;
  int8_t   rssi_rx;        /* at the receiver: dBm (rssi_rx_dbm) or the receiver's own figure; BAY_NO_RSSI = none */
  bool     rssi_rx_dbm;
  int8_t   rssi_tx;        /* at the module, dBm; valid only with rssi_tx_valid */
  bool     rssi_tx_valid;
  uint8_t  lq;             /* 0..100, BAY_NO_LQ = none */
  uint16_t vbat_10mv;      /* 0 = no sensor */
  uint8_t  lost;           /* receiver frames lost, running count */
  uint8_t  uk_hop;
} bay_telem_t;

/* Encoder mode: EUSART1's TX onto PB01 (push-pull, idle high; inverted: idle low), for an encoder that asks; the
 * PPM capture on that pin must be off. bay_tx_end gives the pin back (an input pulled to its idle level) when the
 * frames stop. */
void bay_tx_begin(void);
void bay_tx_end(void);
bool bay_tx_on(void);
/* One telemetry frame to the encoder; returns false while the previous one is still leaving. */
bool bay_send_telemetry(const bay_telem_t *t);
/* Any bytes (up to BAY_TXBUF) out on PB01, from the TX interrupt; false while the previous ones are still leaving. */
#define BAY_TXBUF  64u
bool bay_send_raw(const uint8_t *p, uint8_t n);
bool bay_tx_busy(void);
uint8_t bay_crc8(const uint8_t *p, uint8_t n);

#define BAY_DIAG_N  9u
extern volatile int32_t bay_diag[BAY_DIAG_N];
#define BAY_OK       0   /* good frames                                                        */
#define BAY_HDR      1   /* frames with a first byte other than 0x55 / 0x54 (SBUS = 0x0F)      */
#define BAY_LASTHDR  2   /* that first byte                                                    */
#define BAY_ERR      3   /* frames dropped for a parity or framing error                       */
#define BAY_OVF      4   /* RX FIFO overruns                                                    */
#define BAY_SHORT    5   /* frames cut short by a gap                                           */
#define BAY_EXTRA    6   /* bytes after the 27th before the gap (a longer frame format)         */
#define BAY_BYTES    7   /* bytes received                                                      */
#define BAY_GOOD     8   /* good frames incl. the 2 held back after a silence (polarity search)  */

#endif /* BAY_SERIAL_H */
