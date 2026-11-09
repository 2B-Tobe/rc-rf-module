/*
 * dsm_cap.h — DSM (Spektrum / CYRF6936) raw chip capture role for the EFR32MG24 module.  v3.29
 *
 * Not a receiver: it finds the channel a DSM transmitter is on with an RSSI sweep, centres
 * on the carrier in 125 kHz steps, then alternates two ways of recording the 1 Mchip/s DSSS
 * signal and prints the captures as hex on the bay UART (PuTTY log = the capture,
 * dsm_analyse.py decodes):
 *   packet phase  dsm_raw PHY in packet mode, fixed 1024-byte frames = 8192 hard chips per
 *                 frame if the modem's timing detector locks on the CYRF preamble ("DCP")
 *   soft phase    RAIL demod-data source: 8-bit soft samples (6 MS/s), free-running 4096-sample
 *                 snapshots, the two strongest printed ("DCS"), from the lock and from +-125 kHz
 * Channels in the log are MHz channels with a fraction: "38.5" = 2438.5 MHz (f= gives kHz).
 * Purpose: learn the exact DSM air format from a real transmitter before writing an
 * EFR32 DSM receiver/transmitter.
 *
 * Boot jumpers:  PROTOCOL_00 + PROTOCOL_01 + PROTOCOL_02 (PC00 + PC01 + PC02) fitted (PC00 + PC01 alone is the DSMX transmitter, + F/S the receiver).
 *   (v3.29: PC02 selects the role, so the carrier-detect output is LED_A only)
 *   PC03 (J1 PROTOCOL_03)  DOUT: modem DOUT via PRS (experimental, DC_PRS_DOUT)
 *   PA03 (J4 TEST_A)       DCLK: modem DCLK via PRS (experimental, DC_PRS_DOUT)
 *   LED_A = CD, LED_B: toggles once per hunt sweep, steady when locked
 *   BIND fitted at boot -> bind sniffer: fixed on 2414 MHz, DCK/DCD cycle (v3.31); BIND while hunting -> drop the lock
 *   RANGE low at boot -> data test: census, then frames on the busiest channel (v3.27)
 *   F/S low at boot -> the DSMX receiver role instead (dsm_rx.c); the any-burst hunt option is gone
 *   UART PA05 115200: "DC hunt", "DC sweep ...", "DC lock ch=..", "DC phase packet|soft",
 *                     "DC peak ch=..: .. dBm, mid .., sat ..%, modulation ..%"   (1 MHz, then 125 kHz steps)
 *                     "DC P ch=.. r=.. d=..% b=.. max=.. tim=.. pre=.. sync=.. pkt=.."  1x/s
 *                     "DC S ch=.. r=.. b=.. snaps=.. strongest=.."                     1x/s
 *                     "DCK ch=.. rssi=.. sync=.. sel=.. order=.. inv=.. q=../32 bytes=<16 bytes> sum1=OK sum2=OK"  (v3.20:
 *                     a frame received by the modem and despread; needs the 0011-preamble PHY)
 *                     "DCP ch=.. f=.. rssi=.. sync=.. len=1020" + hex + "DCE"   (packet phase frames)
 *                     "DCX ch=.. rssi=.. sop=.. q=../64 crc=OK bytes=<19 bytes> | ch1=1244 ..."  (v3.28: a DSMX
 *                     data packet decoded in the firmware, data test, 5/s at most)
 *                     "DCT ch=.. n=.. t0=..: 125010/A1/-24 127990/B2/-24 ..."  (v3.28: every frame of the last
 *                     second: microseconds since the previous frame / kind / RSSI; kind A = upper frame
 *                     (channels 0 7 3 8 9 11), B = lower frame (1 5 2 4 6 10), then 1 or 2 = first or
 *                     second packet of the 4 ms pair (by CRC seed); x = CRC failed, n = no SOP)
 *                     "DCS ch=.. f=.. rssi=.. len=4096 sat=.. tr=.. mid=.. fs=.." + hex + "DCE"
 *                     "DCW ..." same, a pinned snapshot (mid = p10/p90 midpoint of the samples)
 *                     snapshots are the two taken at the highest RSSI in the phase (v3.16)
 *                     "DCD ch=.. f=.. rssi=.. len=32768 dec=2 pre=3072 fs=.. ovf=.." + hex + "DCE"  (v3.19:
 *                     one triggered burst, 1 ms before the RSSI edge and 9.6 ms after, every 2nd
 *                     sample = 3 per chip; dsm_analyse.py recovers the chip timing and the bytes)
 *
 * Over SWD: dsm_cap_diag[] (int32) and dsm_cap_peak[126] (int8 dBm, all-time max per channel)
 */
#ifndef DSM_CAP_H
#define DSM_CAP_H
#include <stdint.h>
#include <stdbool.h>
#include "sl_rail.h"

void dsm_cap_start(sl_rail_handle_t rh, bool fixed, bool any_burst);   /* fixed = RANGE low, any_burst = F/S low */
void dsm_cap_service(bool bind_held);     /* every main-loop pass */
void dsm_cap_rx_process(void);            /* a packet is held in RAIL (packet phase frames)          */
void dsm_cap_on_events(sl_rail_events_t events);   /* from the RAIL event callback (ISR) */

#define DC_CHANNELS 126u
#define DC_DIAG_N   24u
extern volatile int32_t dsm_cap_diag[DC_DIAG_N];
extern volatile int8_t  dsm_cap_peak[DC_CHANNELS];   /* all-time max RSSI per channel, dBm  */
extern volatile int8_t  dsm_cap_scan[DC_CHANNELS];   /* max RSSI per channel, last sweep    */
#define DC_STATE     0   /* 0 init, 1 hunt, 2 confirm, 3 locked, 4 peak search           */
#define DC_SUPPORT   1   /* sl_rail_supports_rx_raw_data()                                 */
#define DC_CFG_ERR   2   /* 1 chans 2 rx-data-config 4 fifo 8 start-rx 16 fixed-length    */
#define DC_CH        3   /* quarter channel the radio is on (2400 + n/4 MHz)               */
#define DC_LOCK_RSSI 4   /* RSSI that produced the lock, dBm                               */
#define DC_RSSI      5   /* latest RSSI reading, dBm                                       */
#define DC_BURSTS    6   /* carrier-detect rising edges while locked                       */
#define DC_SWEEPS    7   /* hunt sweeps completed                                          */
#define DC_BEST_CH   8   /* best channel of the last sweep                                 */
#define DC_BEST_RSSI 9   /* its RSSI, dBm                                                  */
#define DC_2ND_CH   10   /* runner-up channel of the last sweep                            */
#define DC_2ND_RSSI 11   /* its RSSI, dBm                                                  */
#define DC_FLOOR    12   /* lowest channel maximum of the last sweep (noise floor), dBm    */
#define DC_INVALID  13   /* RSSI reads that returned SL_RAIL_RSSI_INVALID                  */
#define DC_DUTY     14   /* carrier-detect duty over the last status period, percent       */
#define DC_RELOCKS  15   /* times the lock was dropped (silence or BIND)                   */
#define DC_CD       16   /* carrier-detect output right now                                */
#define DC_THRESH   17   /* detect threshold in use, dBm                                   */
#define DC_FIFO_SZ  18   /* RX FIFO size RAIL granted, bytes                               */
#define DC_OVERFLOW 19   /* RX FIFO overflows (each soft snapshot ends with one)           */
#define DC_CAPTURED 20   /* burst captures completed (v3.18)                               */
#define DC_SENT     21   /* captures printed on the UART                                   */
#define DC_READS    22   /* RSSI reads total (loop rate check)                             */
#define DC_LONGEST  23   /* longest carrier-detect burst seen, microseconds                */
#endif
