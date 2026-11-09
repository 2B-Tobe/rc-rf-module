/*
 * etx_multi.h — the module as a Multiprotocol ("MULTI", 4-in-1) module for an EdgeTX radio (etx_multi.c, v1.1).
 *
 * Wiring (the same two module pins as the MiTo encoder):
 *   JR bay pin 1 (MULTI frames, inverted)    -> J3 RX (PA06, through its 1 k)
 *   JR bay pin 5 (S.Port, telemetry in)      -> PB01 (the PPM pin, Schottky bridged)
 *   JR bay pin 3 (battery) / pin 4 (GND)     -> the module supply / GND
 * In EdgeTX: Model setup -> External RF -> MULTI. The radio's protocol list (colour radios scan the module for it)
 * then shows what this module has, under the Multiprotocol numbers, so a model made for a real 4-in-1 runs
 * unchanged:
 *   FrSkyD   (3)   D8                      -> FrSky D8
 *   DSM      (6)   DSMX_1F / DSMX_2F       -> DSMX 22 ms / 11 ms (11 ms not in the EU build; DSM2 not in this module)
 *   FrSkyX   (15)  CH_16 / CH_8 / EU_16    -> D16 ACCST v1 FCC / FCC 8 channels / EU-LBT (module Sub 2 / 5 / 0)
 *   FrSkyX2  (64)  CH_16 / -    / EU_16    -> D16 ACCST v2 FCC / EU-LBT (module Sub 3 / 1)
 *   MiTo     (120) nRF24 / RLINK / UK459   -> Tobe's own links (the encoder's Prot 0 Sub 0 / 1 / 2)
 * Single / dual band (the 459 MHz link beside 2.4 GHz, where the RA-01 is fitted), per model in EdgeTX (v1.1):
 *   MiTo: Option value 0 = single, any other = dual;  FrSkyD / FrSkyX / FrSkyX2: Option value 1 = dual, else single;
 *   DSM: "Max throw" on = dual (EdgeTX fills the DSM option byte itself; this module has no max-throw mode).
 * Dual only helps a receiver that is a MiTo board with the RA-01 (a FrSky or Spektrum receiver hears 2.4 GHz only).
 * 120 is free in Multiprotocol and EdgeTX (their numbers end near 110); EdgeTX keeps 0 for the scan.
 * Anything else (DSM2, Cloned, EU_8, ...) shows "Protocol invalid" and the RF stays off, as with the encoder.
 *
 * Back to the radio, on PB01 at 100000 8E2 (inverted, unless the radio clears byte 26 bit 3), "MP" frames:
 *   type 0x01 status, every ETX_STATUS_MS (EdgeTX drops it after 2 s): flags, version 1.3.4.<bay version>,
 *        channel order AETR, next / previous protocol, protocol and sub protocol names;
 *   type 0x11 protocol list entry, one per scan request (protocol 0, the entry index in the option byte);
 *   type 0x03 FrSky hub link frame (D8): A1 (the receiver's battery), RSSI;
 *   type 0x02 S.Port frames (D16, RLINK): RSSI 0xF101 (with TX RSSI and TX LQI packed in, as Multiprotocol
 *        does) and RxBt 0xF104, which EdgeTX finds as sensors RSSI, TRSS, TQly and RxBt ("Discover new").
 * RLINK's RSSI sensor is the receiver's dBm + 130, so EdgeTX's usual alarms (45 low, 42 critical) fall at -85 /
 * -88 dBm, where RLINK itself warns. nRF24 and DSMX have no telemetry: only the status.
 * Not carried over from EdgeTX: the failsafe frames (EdgeTX would send one every 7 s and the receivers here store
 * the CURRENT positions on that command, which would mean the sticks of the moment), channel mapping off, low power,
 * autobind, RF tune (the FrSky option line is the dual setting here). The module answers "no failsafe" so EdgeTX does not ask for one.
 */
#ifndef ETX_MULTI_H
#define ETX_MULTI_H

#include <stdbool.h>
#include <stdint.h>
#include "bay_serial.h"

#define ETX_PROTO_MITO   120u     /* Tobe's links in the EdgeTX protocol list                        */
#define ETX_NONE         0xFFu    /* translated protocol: not in this module (cfg_map refuses it)    */
#define ETX_STATUS_MS    500u
#define ETX_TEL_MS       100u     /* link telemetry this often (alternating RSSI / RxBt on S.Port)   */
#define ETX_VERSION_PATCH 15u     /* "V1.3.4.15" in the radio: the module's bay version 1.5          */

/* Multiprotocol numbers -> this module's Prot / Sub / Opt (the encoder's), in place; wproto / wsub keep them. */
void etx_translate(bay_frame_t *f);

/* What the running role knows, for the telemetry back to the radio. */
#define ETX_TEL_NONE   0u         /* nRF24, DSMX, UK only: status only */
#define ETX_TEL_HUB    1u         /* FrSky D8: hub link frame          */
#define ETX_TEL_SPORT  2u         /* D16, RLINK: S.Port frames         */
/* Every new frame from the radio, main loop: answers a scan, sends the status and the link telemetry in turn
 * (one frame at a time, while the previous one is not still leaving). valid = the module runs that protocol. */
void etx_service(const bay_frame_t *f, uint32_t now_us, bool valid, uint8_t tel_kind, const bay_telem_t *t);

/* Builders (also used by the host test): the whole "MP" frame into b, returns its length. */
uint8_t etx_build_status(uint8_t *b, const bay_frame_t *f, bool valid, bool binding);
uint8_t etx_build_scan(uint8_t *b, uint8_t idx);
uint8_t etx_build_hub(uint8_t *b, const bay_telem_t *t);
uint8_t etx_build_sport(uint8_t *b, uint16_t id, uint32_t value);
uint8_t etx_rssi_figure(const bay_telem_t *t);

#endif /* ETX_MULTI_H */
