/*
 * dsm_tx.h — DSMX transmitter role for the EFR32MG24 module (dsm_tx.c, v1.14).
 *
 * The encoder PPM goes out as DSMX for Spektrum-compatible receivers (tested: Spektrum AR610, OrangeRX,
 * Lemon RX DSMX, and the module's own dsm_rx role). The chip stream the CYRF6936 would send is built in
 * software on the dsm_raw PHY (1 Mchip/s GFSK, 300 kHz deviation): bind = 64-chip SDR symbols; data =
 * SOP + 8DR symbols + CRC, two packets 4.01 ms apart on consecutive channels of the ID's 23-channel list.
 * Frame rate chosen at bind and remembered (flash slot 4): 22 ms, 7 channels, bind type A2, or 11 ms,
 * 8 channels, type B2. Before any bind is stored: DTX_MODE_22MS (1 = 22 ms).
 * EU lock (v1.14): DTX_EU_LOCK 1, the default, allows 22 ms only. BIND + RANGE then binds at 22 ms as well, and a
 * rate of 11 ms stored by an older build runs at 22 ms (rebind that receiver once). Set it in app_process.c.
 *
 * Boot jumpers: PC00 + PC01 fitted, PC02 and F/S open.
 *   BIND held at boot         -> bind at 22 ms: 100 packets on 2414 MHz, 10.545 ms apart, then up to 4.2 s
 *                                listening for the receiver's answer; then data.
 *   11 ms bind (8 channels)   -> serial frames: Sub 3 at the bind, only with DTX_EU_LOCK 0.
 *                                (RANGE held at power-up now selects encoder control, app_process.c.)
 *   No BIND                   -> data at once, at the rate of the last bind (the ID is fixed per module, from
 *                                the chip's EUI64, so a receiver stays bound).
 *   RANGE low                 -> -10 dBm for a range check, followed at any time (v1.12).
 * Power: flight DTX_TX_POWER_DDBM (180 = 18 dBm) at 22 ms, DTX_TX_POWER_11MS_DDBM (150) at 11 ms,
 *   bind DTX_BIND_POWER_DDBM (0 dBm), range DTX_RANGE_POWER_DDBM (-10 dBm).
 * Input: the encoder PPM on PB01 (app_process.c); DSM channel ids 0..7 = throttle aileron elevator rudder gear
 *   aux1 aux2 aux3 from PPM channels 3 1 2 4 5 6 7 8 (AETR in, DTX_IN_MAP). 1500 us -> 1024.
 *   No PPM for 100 ms -> centre sticks with throttle 1000 us (bench use); LED_B blinks slowly.
 * LEDs: LED_B bind = fast blink, data = steady, no PPM = slow blink; LED_A toggles per 20 packets.
 * UART PA05 115200: "DT boot dsm_tx v1.14 ... (chosen at this bind / stored at the last bind)", "DT bind at .. ms ...",
 *   "DT bound by receiver ..." or
 *   "DT no answer ...", "DT RANGE on/off: power ...", and once a second "DT s2 tx.. late.. err.. ppm.. ch ..".
 */
#ifndef DSM_TX_H
#define DSM_TX_H
#include <stdint.h>
#include <stdbool.h>
#include "sl_rail.h"

void dsm_tx_start(sl_rail_handle_t rh, const volatile uint16_t *ch_us, bool bind_held, bool range_low);
void dsm_tx_input(bool ppm_present);     /* every main-loop pass, before dsm_tx_action */
void dsm_tx_range(bool range_low);       /* every main-loop pass: the RANGE input (v1.12) */
void dsm_tx_action(void);                /* every main-loop pass */
void dsm_tx_on_events(sl_rail_events_t events);   /* from the RAIL event callback */
void dsm_tx_rx_process(void);            /* a received packet is held in RAIL (bind answer) */

#define RX_SLOT_DSMTX 4u                /* rx_store slot: hop[0] = 22 or 11, the rate of the last bind */
#define DTX_DIAG_N 16u
extern volatile int32_t dsm_tx_diag[DTX_DIAG_N];
#define DT_STATE     0   /* 0 bind, 1 waiting for the answer, 2 data                      */
#define DT_TX        1   /* packets sent                                                  */
#define DT_LATE      2   /* packets launched late (main loop was busy)                    */
#define DT_TXERR     3   /* start_tx refused                                              */
#define DT_HOP       4   /* hop index of the last packet                                  */
#define DT_ID        5   /* id0..id3 as one word                                          */
#define DT_ANSWERS   6   /* bind answers received                                         */
#define DT_PPM       7   /* 1 = PPM present                                               */
#define DT_CFG_ERR   8   /* 1 chans 2 fixlen 4 fifo 8 startrx 32 sync 64 starttx         */
#endif
