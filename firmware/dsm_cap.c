/* dsm_cap.c — see dsm_cap.h.  #included into app_process.c.   v3.32 (BIND at boot = bind sniffer on 2414, trigger -70 dBm; lock at -65 dBm; PC02 selects the role)
 *
 * v3.15: the radio tunes on a 125 kHz grid (a RAM copy of the dsm_raw channel table with
 *        125 kHz spacing: grid numbers q, f = 2400 MHz + q x 125 kHz).  v3.14 saw the Lemon's
 *        carrier pinned low from ch 39 and pinned high from ch 38 although the RSSI peaked
 *        sharply on 39: the demod's linear range is narrower than the offset.  The peak search
 *        now ends with 125 kHz steps and picks the LO whose demod output is centred (p10/p90
 *        midpoint nearest 0); the soft phase then looks from one step below and above as well.
 * v3.16: snapshots are ranked by the RSSI read while they filled, not by saturation (the run of
 *        24 Sep showed the discriminator swinging rail to rail on noise and staying small and
 *        centred on a strong on-frequency signal); coarse peak readings are reused by the fine
 *        steps on the same frequency; 80 ms per step.
 * v3.17: the strongest snapshot of the peak search is printed before the lock (peak=1 in its
 *        header), so a signal that lasts only a second still leaves a capture.
 * v3.18: run 17 decoded: the Lemon bind is 64-chip SDR with Deviation's pn[0][8], 1 Mchip/s, on
 *        2408.000 MHz.  New burst phase right after the lock: the demod samples stream through
 *        the RAIL FIFO into a 64 kB ring, an RSSI edge triggers, one whole packet (1 ms before
 *        the edge, 9.6 ms after) is kept and printed: raw samples for the first 5 ms ("DCR")
 *        and hard chips for all of it ("DCH").  The coarse peak search locks at once when its
 *        winner is centred.  Neighbour views and the packet phase are out of the sequence.
 * v3.19: run 18's two captures decoded to the complete 16-byte bind packet (both checksums
 *        right).  The capture is now printed as the whole window at 3 samples per chip
 *        ("DCD"); the hard chips sliced in the firmware slipped (rate error) and are gone.
 * v3.20: run 19: the frame is preamble 0011 x 27 then 128 symbols, no SOP, no CRC.  The packet
 *        phase is now a receiver test: sync words = first 32 chips of the code / its inverse
 *        (dual sync), 1020-byte frames despread in software to the 16 bind bytes ("DCK" lines).
 *        It needs the PHY regenerated with the 0011 preamble (the current one has 0101).
 * v3.22: run 21 reported dual sync unavailable, so sync0 alternates between the code and its
 *        inverse every 250 ms in the packet phase (this Lemon's packets start with the inverse).
 * v3.23: run 22 with the regenerated PHY: tim=88 pre=45 - the modem now detects the Lemon's
 *        preamble - but sync=0 (v3.21 was flashed, code only armed).  The packet phase tries
 *        inverse, code, and both bit-reversed as sync word 0, 250 ms each, stays with the
 *        first that syncs, and reports per slot.
 * v3.24: a pinned coarse winner locks straight on the neighbouring MHz channel the sign points to.
 * v3.25: RUN 26 RECEIVED THE BIND PACKETS THROUGH THE MODEM (sync/pkt 39/38, 47/46, 44/43 on three
 *        binds, every frame despread to the 16 bytes with both sums right).  New data test on the
 *        RANGE jumper: one DSMX channel of this id, sync on the preamble, frames printed raw.
 * v3.26: the data test lost its preamble sync to the slot rotation after 250 ms (run 27); fixed,
 *        and the test channel moved from 47 (inside the WiFi) to 9.
 * v3.27: run 28 on ch 9: not one preamble detect in 60 s, so the computed channel set is in doubt.
 *        The data test now starts with a census: every channel 3..79 for 400 ms in packet mode
 *        with the preamble sync, two passes, counting preamble detects; the busiest channel
 *        is then used for the frame capture.
 * v3.28: run 29 (census + 65 frames on 2474 MHz) decoded the DSMX data packet completely - SOP,
 *        8DR symbols, channel words, CRC - so the data test now decodes every frame in the
 *        firmware ("DCX" lines, 5/s at most) and logs the time of every frame ("DCT" lines,
 *        one per second: delta t in us, frame kind, RSSI) to learn the hop timing.  Run 29 also
 *        showed weak (-96 dBm) frames of this Lemon coded with pn row 0 on the row-1 channel:
 *        they decode (same id, same channel values) so they are its packets from another
 *        channel of the set leaking in, not noise; DCT tags them "n" when the SOP search fails.
 *        Cross-checked against the Multi-module source (DSM.ino, DSM_cyrf6936.ino): pncodes,
 *        sop_col, data columns, pn_row, the channel list and the two CRC seeds all agree; Multi
 *        sends each frame twice per 11 ms - CH1 on hop n with seed ~(id0<<8|id1), CH2 on hop
 *        n+1 4.01 ms later with seed id0<<8|id1 - lower and upper frames alternating, so one
 *        channel comes round every 126.5 ms (alternately 125.0 and 128.0 ms).  DCT prints the
 *        seed as 1/2 after the kind so the log shows whether the Lemon does the same.
 *
 * Why: two attempts at a DSM receiver on the EFR32 heard nothing, so instead of
 * guessing the physical layer again we record it.  The CYRF6936 in a DSM transmitter
 * sends 64-chip DSSS symbols at 1 Mchip/s (GFSK, 2400 + n MHz).  This role finds the
 * transmitter's channel with an RSSI sweep, then alternates two capture methods on it:
 *
 *   phase P "packet": the dsm_raw PHY (2GFSK 1 Mbps, preamble 0011 x16, sync 0x3333) in
 *            normal packet mode with a fixed length of 1024 bytes = 8192 chips.  If the
 *            modem's timing detector locks on the CYRF preamble, every bind burst arrives
 *            as one packet of hard chips, sampled by the hardware at exactly 1 bit per chip.
 *            The timing / preamble / sync / packet event counters say how far it gets.
 *   phase S "soft":   RAIL's demod-data source, which on the MG24 delivers 8-bit soft
 *            samples at several MS/s (v2 found that).  Too fast to stream, so on every
 *            carrier-detect the RX is restarted and the 4096-byte FIFO fills once and
 *            freezes (overflow terminates RX): a snapshot of ~1 ms from inside the burst,
 *            no assumptions about preamble or timing.  The sample rate is measured too.
 *
 * Both print their captures as hex on the UART; dsm_analyse.py --uart <log> decodes them.
 * (RAIL direct mode - the chip stream on a GPIO - is not supported on the MG24.)
 *
 * Sequence:  hunt (sweep of all 126 channels, DC_DWELL_US each; the winner is the busiest channel -
 *            the strongest channel that was busy at all, or the busiest one with F/S fitted)
 *         -> confirm (listen on the best channel for DC_CONFIRM_US)
 *         -> locked: S for DC_SOFT_US, then P for DC_PKT_US, then hunt again; CD from the RSSI;
 *            re-hunt after DC_SILENCE_US without carrier or when BIND is held
 * RANGE jumper low at boot: no hunt, lock on DC_FIXED_CH for good.
 */
#include "dsm_cap.h"
#include "rail_config.h"
#include "sl_gpio.h"
#include "em_device.h"
#include <string.h>
#if defined(SL_CATALOG_APP_LOG_PRESENT)
#include "app_log.h"
#endif

extern const RAIL_ChannelConfig_t dsm_raw_channelConfig;   /* PHY: 2GFSK 1 Mbps, 320 kHz dev, 1 MHz channels 0..125 */

/* tuning grid: the same PHY on a RAM channel table with DC_STEP_KHZ spacing.  Everything the radio
 * is told uses grid numbers q (f = 2400 MHz + q x DC_STEP_KHZ); the hunt tables stay per MHz channel. */
#ifndef DC_STEP_KHZ
#define DC_STEP_KHZ       125u                        /* must divide 1000 */
#endif
#define DC_QPC            (1000u / DC_STEP_KHZ)       /* grid steps per MHz channel */
#define DC_QCHANNELS      (DC_CHANNELS * DC_QPC)      /* 1008 tuning steps, 2400.000 .. 2525.875 MHz */
static inline uint32_t dc_khz(uint16_t q)  { return 2400000u + DC_STEP_KHZ * q; }
static inline uint8_t  dc_q2ch(uint16_t q) { uint16_t c = (uint16_t)((q + DC_QPC / 2u) / DC_QPC); return (uint8_t)(c < DC_CHANNELS ? c : DC_CHANNELS - 1u); }
static const char *dc_chs(uint16_t q)                 /* "38", "38.125", "38.5", "38.875" for the log */
{
  static char buf[2][12]; static uint8_t which;
  char *b = buf[which ^= 1u];
  unsigned c = q / DC_QPC, i = 0; char tmp[4]; unsigned t = 0;
  do { tmp[t++] = (char)('0' + c % 10u); c /= 10u; } while (c && t < 3u);
  while (t) b[i++] = tmp[--t];
  unsigned k = (q % DC_QPC) * DC_STEP_KHZ;            /* fraction in kHz, 0..999: three digits, trailing zeros dropped */
  if (k) {
    b[i++] = '.'; b[i++] = (char)('0' + k / 100u); b[i++] = (char)('0' + (k / 10u) % 10u); b[i++] = (char)('0' + k % 10u);
    while (b[i - 1] == '0') i--;
  }
  b[i] = '\0';
  return b;
}

#ifndef DC_FIXED_CH
#define DC_FIXED_CH       13u        /* RANGE low at boot: capture here without hunting (0x0D = Deviation's bind ch) */
#endif
#ifndef DC_FIXED_Q
#define DC_FIXED_Q        (DC_FIXED_CH * DC_QPC)      /* ... as a grid number; e.g. 308 = 2438.5 MHz at 125 kHz steps */
#endif
#ifndef DC_THRESH_DBM
#define DC_THRESH_DBM     (-75)      /* RSSI above this = carrier (a DSM TX a metre away reads -50 .. -30)      */
#endif
#ifndef DC_PRS_DOUT
#define DC_PRS_DOUT       1          /* also route the modem's DOUT/DCLK PRS signals to PC03/PA03 (experiment)  */
#endif
#define DC_DWELL_US       1500u      /* per channel while hunting: 126 x 1.5 ms = 190 ms per sweep               */
#define DC_ACTIVE_DUTY    4u         /* a channel must be at least this busy (%) in its dwell to be a candidate    */
#define DC_LONG_RUN_US    2000u      /* default hunt: and show one burst at least this long (DSM bind = 8 ms;
                                        Bluetooth 0.6 ms).  F/S fitted: any burst length (strongest active)       */
#define DC_RUN_GAP_US     300u       /* RSSI reads closer than this belong to the same burst                      */
#define DC_RECENT_DECAY   4          /* dB per sweep: a channel stays a candidate for a few sweeps after a burst   */
#define DC_CONFIRM_DUTY   3u         /* and this busy during the confirm listen                                   */
#define DC_SETTLE_US      250u       /* after start_rx before RSSI is trusted                                    */
#define DC_CONFIRM_US     600000u    /* listen on the sweep winner this long before locking                      */
#define DC_SILENCE_US     5000000u   /* locked, no carrier this long -> hunt again (a released bind button)       */
#define DC_CD_HOLD_US     150u       /* CD stays up this long after the last above-threshold read                */
#define DC_BIND_US        50000u     /* BIND held this long while locked -> hunt again                           */
#define DC_UART_US        1000000u   /* status line interval while locked                                        */
#define DC_MIN_OVER_FLOOR 12         /* sweep winner must beat the sweep's noise floor by this many dB           */
#define DC_DIAG_NONE      (-128)

#define DC_SOFT_US        1200000u   /* soft phase length (first after a lock: the data that matters)            */
#define DC_PKT_US         2000000u   /* packet phase length                                                      */
#define DC_STRONG_DBM     (-65)      /* a dwell reading this strong locks at once, no confirm (v3.30: was -45; the longest burst wins) */
#ifndef DC_SKIP_LO
#define DC_SKIP_LO        200u       /* channels DC_SKIP_LO..DC_SKIP_HI never lock instantly (a known local device) */
#define DC_SKIP_HI        200u       /*   e.g. 69 / 73 for the mouse dongle on 2470 MHz; 200 = none                */
#endif
#define DC_FIFO_BYTES     4096u      /* RAIL RX FIFO; in the soft phase this is the snapshot length              */
#define DC_PKT_BYTES      1020u      /* packet phase fixed frame: 8192 - 32 (sync) chips = one 16-byte bind packet */
/* v3.20 packet phase = receiver test: sync word 0 = first 32 chips of the bind code pn[0][8], sync
 * word 1 = its inverse (dual sync), 1020-byte frames despread in software.  Works once the PHY has
 * the 0011 preamble (the generated dsm_raw PHY has 0101: timing detect never fires on the Lemon). */
#define DC_CODE64         0x86AE895EB154A1D7ull   /* pn[0][8] D7 A1 54 B1 5E 89 AE 86, chip i = bit i (LSB first on air) */
#define DC_SYNC0          0xB154A1D7ul            /* first 32 chips of the code (symbol = bit 1), bit 0 first on air */
#define DC_SYNC1          0x4EAB5E28ul            /* first 32 chips of the inverse (bit 0)         */
/* without dual sync the packet phase tries these as sync word 0 in turn, DC_SYNC_ALT_US each, and
 * stays with the first one that produces sync detects: inverse (this Lemon's packets start with
 * it), code, then both with the bit order reversed in case RAIL's "LSB first on air" is not what
 * this PHY does.  sync word 1 is set to the partner (code <-> inverse) for the day dual sync works. */
/* v3.25 data test (RANGE jumper): sit on one of this Lemon's 23 DSMX channels (Deviation's
 * calc_dsmx_channel on id 37 22 1A 2D: 47 61 57 33 59 31 35 23 73 39 25 9 5 37 49 7 17 3 71 69 15 53 55;
 * every hop sighting so far was on or next to one of them), sync on the preamble itself
 * (1001 x 8 = 0x99999999, so the frame starts inside the preamble whatever the first symbol is)
 * and print the first DC_DATA_PRINT bytes of every frame for offline decoding: 64-chip SOP
 * pncodes[(ch-2)%5][5], then 8DR data symbols (16 bytes + CRC) with data code column 2/3. */
#ifndef DC_DATA_CH
#define DC_DATA_CH        9u         /* 2409 MHz: in the id's set, and clear of the WiFi seen on 35..50 (run 27 on 47 was buried in it) */
#endif
#define DC_SYNC_PRE       0x99999999ul
#define DC_DATA_BYTES     200u       /* frame: preamble rest + SOP 64 + 4 + ~SOP 64 + 19 x 64 = ~1400 chips      */
#define DC_DATA_PRINT     200u       /* bytes of each frame printed (1600 chips)                                  */
/* v3.28: the DSMX data packet, decoded from run 29 (Deviation's DSM source + the captures):
 *   RF channel = DSMX channel + 1 (the Lemon's 73 is on 2474 MHz); pn_row = (dsmx_ch - 2) % 5 = (rf - 3) % 5
 *   sop_col = (id0 + id1 + id2 + 2) & 7, data_col = 7 - sop_col   (this id: 5 and 2)
 *   preamble 1001.. | SOP = pn[row][sop_col] 64 chips | 4 chips | ~SOP 64 chips | 19 symbols of 64 chips
 *   8DR symbol for byte v: code = v & 0x40 ? pn[row][data_col + 1] : pn[row][data_col], chip i of the symbol =
 *   code chip (i + (v & 0x3F) - 1) mod 64, all chips inverted when bit 7 of v is clear
 *   bytes: 0x10 (length), id[2], id[3] + model, 7 words (bit 15: second frame type on word 0,
 *   bits 14..11 channel, bits 10..0 value; the channel order is Deviation's ch_map12), CRC-16
 *   (poly 0x8005 reflected, init = bit-reversed seed, seed = ~(id0<<8|id1) on one packet and
 *   its complement on the next, xor 0xFFFF, low byte first) over the 17 bytes before it. */
#define DC_ID0 0x37u
#define DC_ID1 0x22u
#define DC_ID2 0x1Au
#define DC_ID3 0x2Du
#define DC_SOP_COL  ((DC_ID0 + DC_ID1 + DC_ID2 + 2u) & 7u)
#define DC_DATA_COL (7u - DC_SOP_COL)
#define DC_CRC_SEED ((uint16_t)~((DC_ID0 << 8) | DC_ID1))
#define DC_DATA_SYMS 19u
static const uint8_t dc_pn[5][9][8] = {
  { { 0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8 }, { 0x88, 0x17, 0x13, 0x3B, 0x2D, 0xBF, 0x06, 0xD6 }, { 0xF1, 0x94, 0x30, 0x21, 0xA1, 0x1C, 0x88, 0xA9 },
    { 0xD0, 0xD2, 0x8E, 0xBC, 0x82, 0x2F, 0xE3, 0xB4 }, { 0x8C, 0xFA, 0x47, 0x9B, 0x83, 0xA5, 0x66, 0xD0 }, { 0x07, 0xBD, 0x9F, 0x26, 0xC8, 0x31, 0x0F, 0xB8 },
    { 0xEF, 0x03, 0x95, 0x89, 0xB4, 0x71, 0x61, 0x9D }, { 0x40, 0xBA, 0x97, 0xD5, 0x86, 0x4F, 0xCC, 0xD1 }, { 0xD7, 0xA1, 0x54, 0xB1, 0x5E, 0x89, 0xAE, 0x86 } },
  { { 0x83, 0xF7, 0xA8, 0x2D, 0x7A, 0x44, 0x64, 0xD3 }, { 0x3F, 0x2C, 0x4E, 0xAA, 0x71, 0x48, 0x7A, 0xC9 }, { 0x17, 0xFF, 0x9E, 0x21, 0x36, 0x90, 0xC7, 0x82 },
    { 0xBC, 0x5D, 0x9A, 0x5B, 0xEE, 0x7F, 0x42, 0xEB }, { 0x24, 0xF5, 0xDD, 0xF8, 0x7A, 0x77, 0x74, 0xE7 }, { 0x3D, 0x70, 0x7C, 0x94, 0xDC, 0x84, 0xAD, 0x95 },
    { 0x1E, 0x6A, 0xF0, 0x37, 0x52, 0x7B, 0x11, 0xD4 }, { 0x62, 0xF5, 0x2B, 0xAA, 0xFC, 0x33, 0xBF, 0xAF }, { 0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97 } },
  { { 0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97 }, { 0x8E, 0x4A, 0xD0, 0xA9, 0xA7, 0xFF, 0x20, 0xCA }, { 0x4C, 0x97, 0x9D, 0xBF, 0xB8, 0x3D, 0xB5, 0xBE },
    { 0x0C, 0x5D, 0x24, 0x30, 0x9F, 0xCA, 0x6D, 0xBD }, { 0x50, 0x14, 0x33, 0xDE, 0xF1, 0x78, 0x95, 0xAD }, { 0x0C, 0x3C, 0xFA, 0xF9, 0xF0, 0xF2, 0x10, 0xC9 },
    { 0xF4, 0xDA, 0x06, 0xDB, 0xBF, 0x4E, 0x6F, 0xB3 }, { 0x9E, 0x08, 0xD1, 0xAE, 0x59, 0x5E, 0xE8, 0xF0 }, { 0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E } },
  { { 0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E }, { 0x80, 0x69, 0x26, 0x80, 0x08, 0xF8, 0x49, 0xE7 }, { 0x7D, 0x2D, 0x49, 0x54, 0xD0, 0x80, 0x40, 0xC1 },
    { 0xB6, 0xF2, 0xE6, 0x1B, 0x80, 0x5A, 0x36, 0xB4 }, { 0x42, 0xAE, 0x9C, 0x1C, 0xDA, 0x67, 0x05, 0xF6 }, { 0x9B, 0x75, 0xF7, 0xE0, 0x14, 0x8D, 0xB5, 0x80 },
    { 0xBF, 0x54, 0x98, 0xB9, 0xB7, 0x30, 0x5A, 0x88 }, { 0x35, 0xD1, 0xFC, 0x97, 0x23, 0xD4, 0xC9, 0x88 }, { 0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93 } },
  { { 0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93 }, { 0xDC, 0x68, 0x08, 0x99, 0x97, 0xAE, 0xAF, 0x8C }, { 0xC3, 0x0E, 0x01, 0x16, 0x0E, 0x32, 0x06, 0xBA },
    { 0xE0, 0x83, 0x01, 0xFA, 0xAB, 0x3E, 0x8F, 0xAC }, { 0x5C, 0xD5, 0x9C, 0xB8, 0x46, 0x9C, 0x7D, 0x84 }, { 0xF1, 0xC6, 0xFE, 0x5C, 0x9D, 0xA5, 0x4F, 0xB7 },
    { 0x58, 0xB5, 0xB3, 0xDD, 0x0E, 0x28, 0xF1, 0xB0 }, { 0x5F, 0x30, 0x3B, 0x56, 0x96, 0x45, 0xF4, 0xA1 }, { 0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8 } }
};
#define DC_DATA_PRINT_US  1000000u
/* v3.27 census: before sitting on a channel, the data test visits every channel 3..79 for DC_CENSUS_US
 * in packet mode with the preamble sync and counts the preamble detects; the channels with detects
 * are this Lemon's hopping set as measured (DSMX visits each of its 23 channels every 253 ms) */
#define DC_SNIFF_CH      14u         /* v3.31 bind sniffer channel: the transmitter role binds on 2414 MHz          */
#define DC_CENSUS_LO      3u
#define DC_CENSUS_HI      79u
#define DC_CENSUS_US      400000u
#define DC_CENSUS_PASSES  2u
#define DC_SYNC_VARIANTS  4u
static const uint32_t dc_sync_tab[DC_SYNC_VARIANTS][2] = {
  { DC_SYNC1, DC_SYNC0 }, { DC_SYNC0, DC_SYNC1 }, { 0x147AD572ul, 0xEB852A8Dul }, { 0xEB852A8Dul, 0x147AD572ul } };
static const char *dc_sync_name[DC_SYNC_VARIANTS] = { "inverse", "code", "inverse-bitrev", "code-bitrev" };
#define DC_PKT_SEND_US    2000000u   /* at most one packet printed per this interval (2 kB hex = 180 ms)        */
#define DC_SNAP_SAT       100        /* |sample| >= this counts as saturated (signal present)                     */
#define DC_SNAP_APART_US  50000u     /* the two printed snapshots come from moments at least this far apart       */
#define DC_RATE_US        150000u    /* soft phase: continuous-drain window to measure the sample rate           */

/* burst phase (v3.18): the demod samples stream through the 4 kB RAIL FIFO into a 64 kB ring
 * (10.9 ms at 6 MS/s) drained by the main loop; an RSSI rising edge marks the start of a burst
 * and the capture stops DC_RING_BYTES - DC_RING_PRE samples later, so the ring holds 1 ms before
 * the edge (preamble) and 9.6 ms after it: one whole 8.4 ms DSM bind packet.  Printed as raw
 * the whole window, every 2nd sample (3 per chip); the PC recovers the chip timing.              */
#define DC_RING_BYTES     65536u
#define DC_RING_PRE       6144u      /* samples kept before the trigger: ~1 ms                                   */
#define DC_DEC            2u         /* the window is printed every DC_DEC-th sample: 3 samples per chip          */
#define DC_BURST_US       2500000u   /* burst phase: wait this long for an edge                                  */
#define DC_TRIG_BELOW     15         /* trigger level = lock RSSI - this (never below DC_THRESH_DBM)             */
#define DC_NEIGHBOUR_VIEWS 0         /* 1 = soft phases from +-1 grid step as well (v3.15 - v3.17)               */

volatile int32_t dsm_cap_diag[DC_DIAG_N];
volatile int8_t  dsm_cap_peak[DC_CHANNELS];
volatile int8_t  dsm_cap_scan[DC_CHANNELS];

static sl_rail_handle_t dc_rail;
/* v3.29: PC02 is now the jumper that selects this role, so the carrier-detect output is LED_A only */
static sl_gpio_t dc_led_a  = { .port = SL_GPIO_PORT_C, .pin = 4 };
static sl_gpio_t dc_led_b  = { .port = SL_GPIO_PORT_C, .pin = 5 };
#if DC_PRS_DOUT
static sl_gpio_t dc_dout_pin = { .port = SL_GPIO_PORT_C, .pin = 3 }; /* J1 PROTOCOL_03: PRS ch 6 = MODEML DOUT   */
static sl_gpio_t dc_dclk_pin = { .port = SL_GPIO_PORT_A, .pin = 3 }; /* J4 TEST_A:      PRS ch 0 = MODEML DCLK   */
#endif

static SL_RAIL_DECLARE_FIFO_BUFFER(dc_fifo, DC_FIFO_BYTES);
static uint8_t  dc_snap[2][DC_FIFO_BYTES];      /* best two soft snapshots of the current soft phase */
static uint16_t dc_snap_len[2], dc_snap_sat[2], dc_snap_tr[2]; /* length, score (% saturated), zero crossings */
static int16_t  dc_snap_mid_[2];                 /* p10/p90 midpoint (carrier offset indicator) */
static uint32_t dc_snap_walls;                   /* snapshots rejected as pinned this phase */
static int16_t  dc_snap_rssi[2];
static uint32_t dc_snap_time[2];
static bool     dc_snap_full[2];
static uint32_t dc_snap_t0;                     /* time of the last restart */
static int16_t  dc_snap_tag;                    /* RSSI read while this snapshot was filling */
static bool     dc_snap_tagged;
static uint32_t dc_snaps;                       /* snapshots taken this phase */
static int16_t  dc_snap_maxrssi;                /* strongest snapshot tag this phase, dBm */
static volatile uint32_t dc_ovf_us;             /* time of the last overflow (ISR) */
static uint32_t dc_rate_t0;                     /* sample-rate measurement: fill time of a 2048- then a 4096-byte FIFO */
static uint32_t dc_rate_t2, dc_rate_t4;         /*   (the RX warm-up cancels in the difference) */
static uint8_t  dc_rate_step;
static bool     dc_rate_done;
static uint32_t dc_rate_kss;                    /* measured kS/s (0 = unknown) */

static uint8_t  dc_ring[DC_RING_BYTES];         /* burst phase capture ring */
static uint32_t dc_b_wr, dc_b_filled, dc_b_post, dc_b_trig_pos, dc_b_ovf;
static bool     dc_b_armed, dc_b_trig, dc_b_done;
static int16_t  dc_b_level, dc_b_rssi;
static uint8_t  dc_pkt[DC_PKT_BYTES];           /* last packet-phase frame */
static uint16_t dc_pkt_len;
static int16_t  dc_pkt_rssi;
static bool     dc_pkt_full;
static uint32_t dc_pkt_send_us;
static uint8_t  dc_pkt_sync;                     /* which sync word the frame matched (0 = code, 1 = inverse) */
static uint8_t  dc_dec[16];                      /* despread bytes of the last frame */
static uint8_t  dc_dec_q, dc_dec_order, dc_dec_inv;   /* worst symbol |score| (0..32), chip order that won (0 LSB, 1 MSB), bytes inverted to pass the sums */
static bool     dc_dec_full, dc_dual, dc_dual_hw;
static int32_t  dc_dual_st;                      /* status of the dual-sync rx-option call */
static uint8_t  dc_sync_sel;                     /* without dual sync: which dc_sync_tab entry is sync0 now */
static uint32_t dc_sync_sel_us, dc_sync_slot_sync, dc_sync_slot_pkt;   /* slot start; sync/packet counts at the slot start */
static bool     dc_sync_locked;                  /* a slot produced sync detects: stay with it */
#define DC_SYNC_ALT_US 250000u                   /* ... alternated this often in the packet phase */
static uint32_t dc_dec_n, dc_dec_ok;             /* frames despread / with both checksums right, this phase */

static volatile uint32_t dc_ev_tim, dc_ev_timlost, dc_ev_pre, dc_ev_prelost, dc_ev_sync, dc_ev_pkt, dc_ev_abort, dc_ev_ferr;
static volatile bool dc_overflow;

static uint8_t  dc_state;            /* 0 init, 1 hunt, 2 confirm, 3 locked */
static uint8_t  dc_ph;               /* locked: 0 = packet phase, 1 = soft phase */
static uint32_t dc_ph_since;
static bool     dc_fixed, dc_cd, dc_bind_seen, dc_soft_mode, dc_rssi_hunt;
static bool     dc_data_mode;                    /* RANGE jumper: data test on DC_DATA_CH (v3.25) */
static bool     dc_census;                       /* data test: census pass running */
static uint64_t dc_sym[256];                     /* 8DR symbol table for the current row: chip i = bit i */
static uint64_t dc_sop64;                        /* the row's SOP code */
static uint8_t  dc_sym_row = 0xFF;
static uint8_t  dc_dx[DC_DATA_SYMS];             /* the last decoded data packet */
static uint8_t  dc_dx_q, dc_dx_seed;   /* worst symbol match (64 = perfect), which seed passed (0 seed, 1 ~seed, 2 = crc failed) */
static uint16_t dc_dx_sop;                       /* chip position of the SOP in the frame */
static int8_t   dc_dx_rssi;
static bool     dc_dx_full;
static uint32_t dc_dx_n, dc_dx_crc_ok, dc_dx_nosop;
/* packet timing (v3.28): every frame the modem delivers gets an entry (sync-end time, RSSI, what
 * the decoder made of it); printed once a second as a "DCT" line, deltas in microseconds */
#define DC_DT_N 32u
static struct { uint32_t t; int8_t rssi; char kind, pos; } dc_dt[DC_DT_N];
static uint8_t  dc_dt_n, dc_dt_drop;
static uint32_t dc_dt_last;                      /* time of the previous entry (across lines) */
static bool     dc_dt_have;
static uint8_t  dc_census_ch, dc_census_pass;
static uint32_t dc_census_since;
static uint16_t dc_census_pre[DC_CHANNELS], dc_census_pkt[DC_CHANNELS];
static uint8_t  dc_duty[DC_CHANNELS];            /* % of RSSI reads above threshold, last sweep */
static uint16_t dc_run[DC_CHANNELS];             /* longest burst seen in the dwell, us (capped at 65535) */
static uint16_t dc_h_reads, dc_h_hits;           /* per-channel counters while hunting / confirming */
static uint32_t dc_h_run_start, dc_h_run_last, dc_h_run_max;   /* burst-length tracking from RSSI reads */
static uint32_t dc_cd_sum_us, dc_cd_n;           /* sum / count of burst lengths in the status period (mean burst) */
static uint8_t  dc_cand[3], dc_cand_n, dc_cand_i;  /* confirm candidates: strongest active channels, in order */
/* peak search around a strong sweep reading c, two stages in soft mode:
 *   coarse: c-1, c, c+1, c+2 in 1 MHz steps -> the strongest (the sweep runs upward, c may be the low skirt)
 *   fine:   +-4 grid steps (+-500 kHz) around it -> among the steps within DC_PEAK_WITHIN dB of the
 *           strongest, the one whose demod output is centred.  Per step the snapshot taken while the
 *           RSSI was highest is kept: its p10/p90 midpoint says where the carrier sits relative to the
 *           LO (0 = centred, -128/+127 = pinned); the table printed is the discriminator's response */
#define DC_PEAK_COARSE   4u
#define DC_PEAK_FINE     9u
#define DC_PEAK_MAX      9u
#define DC_PEAK_DWELL_US 80000u      /* per step: ~80 snapshots, several inside bursts even at 21/s           */
#define DC_PEAK_WITHIN   3           /* dB: fine steps this close to the strongest compete on centring       */
static uint16_t dc_pk_first, dc_pk_step, dc_pk_n, dc_pk_i;   /* the current list of steps: first q, q per step, count, index */
static uint8_t  dc_pk_stage;                     /* 1 coarse, 2 fine */
static uint8_t  dc_pk_c;                         /* the sweep channel that started it (fallback) */
static int16_t  dc_pk_rssi, dc_pk_tag;           /* max RSSI tag this step; the tag of the snapshot now filling */
static int16_t  dc_pk_mid;                       /* midpoint of the snapshot with the max tag */
static uint16_t dc_pk_score, dc_pk_sat;          /* best modulation score this step; sat % of the max-tag snapshot */
static int16_t  dc_pk_r[DC_PEAK_MAX], dc_pk_m[DC_PEAK_MAX];   /* per step: max RSSI tag, midpoint */
static uint16_t dc_pk_cq[DC_PEAK_COARSE]; static int16_t dc_pk_cr[DC_PEAK_COARSE], dc_pk_cm[DC_PEAK_COARSE];   /* the coarse stage's results, reused by the fine steps that coincide */
static uint8_t  dc_pk_snap[DC_FIFO_BYTES];       /* the snapshot taken at the highest RSSI during the whole peak search: printed */
static uint16_t dc_pk_snap_len, dc_pk_snap_q, dc_pk_snap_sat, dc_pk_snap_tr; /*   before the lock, so a signal that lasts */
static int16_t  dc_pk_snap_rssi, dc_pk_snap_mid; static bool dc_pk_snap_full;  /*   only a second still leaves a capture   */
static bool     dc_strong_seen;                  /* a reading >= DC_STRONG_DBM in this sweep */
static uint8_t  dc_neighbour_tries;              /* soft phases done from the +-1 grid step neighbours */
static uint32_t dc_pk_snap_t0;
static bool     dc_pk_ready;
static int8_t   dc_recent[DC_CHANNELS];          /* max RSSI per channel over the last few sweeps (decays DC_RECENT_DECAY dB/sweep) */
static RAIL_ChannelConfigEntry_t dc_grid_entry;  /* the dsm_raw channel entry on the fine grid */
static RAIL_ChannelConfig_t      dc_grid_cfg;

static void dc_run_hit(uint32_t now)
{
  if (dc_h_run_last == 0u || (now - dc_h_run_last) > DC_RUN_GAP_US) dc_h_run_start = now;
  dc_h_run_last = now;
  uint32_t len = now - dc_h_run_start;
  if (len > dc_h_run_max) dc_h_run_max = len;
}
static void dc_run_reset(void) { dc_h_run_start = 0; dc_h_run_last = 0; dc_h_run_max = 0; }
static uint16_t dc_q;                /* quarter channel the radio is on */
static uint16_t dc_lock_q;           /* the locked (centre) quarter channel; dc_q leaves it for the neighbour views */
static int16_t  dc_max;              /* max RSSI on the current hunt channel, dBm */
static uint32_t dc_rx_since, dc_phase_since, dc_cd_last, dc_cd_start, dc_uart_us, dc_bind_since;
static uint32_t dc_cd_high_us, dc_cd_win_since;
static uint8_t  dc_hunt_ch;

static inline uint32_t dc_now(void) { return (uint32_t)sl_rail_get_time(dc_rail); }

/* part of the current CD burst that falls inside the current duty window */
static uint32_t dc_cd_since_window(uint32_t now)
{
  uint32_t from = ((int32_t)(dc_cd_start - dc_cd_win_since) > 0) ? dc_cd_start : dc_cd_win_since;
  return now - from;
}

static void dc_cd_set(bool on)
{
  if (on == dc_cd) return;
  dc_cd = on;
  dsm_cap_diag[DC_CD] = on ? 1 : 0;
  if (on) sl_gpio_set_pin(&dc_led_a);
  else    sl_gpio_clear_pin(&dc_led_a);
}

/* RSSI in whole dBm, or DC_DIAG_NONE when RAIL has no fresh reading yet */
static int16_t dc_rssi(void)
{
  int16_t q = sl_rail_get_rssi(dc_rail, SL_RAIL_GET_RSSI_NO_WAIT);
  dsm_cap_diag[DC_READS]++;
  if (q == SL_RAIL_RSSI_INVALID) { dsm_cap_diag[DC_INVALID]++; return DC_DIAG_NONE; }
  int16_t d = (int16_t)(q / 4);
  dsm_cap_diag[DC_RSSI] = d;
  return d;
}

static void dc_sync_set(uint8_t sel);
/* ---- data source: packet mode (hard chips, fixed 1024 bytes) or soft samples to the FIFO ---- */
static void dc_source(bool soft)
{
  sl_rail_idle(dc_rail, SL_RAIL_IDLE, true);
  sl_rail_reset_fifo(dc_rail, false, true);
  sl_rail_rx_data_config_t rdc;
  if (soft) { rdc.rx_source = SL_RAIL_RX_DATA_SOURCE_DEMOD_DATA;  rdc.rx_method = SL_RAIL_DATA_METHOD_FIFO_MODE; }
  else      { rdc.rx_source = SL_RAIL_RX_DATA_SOURCE_PACKET_DATA; rdc.rx_method = SL_RAIL_DATA_METHOD_PACKET_MODE; }
  if (sl_rail_config_rx_data(dc_rail, &rdc) != SL_RAIL_STATUS_NO_ERROR) dsm_cap_diag[DC_CFG_ERR] |= 2;
  if (!soft) {
    uint16_t want = dc_data_mode ? (uint16_t)DC_DATA_BYTES : (uint16_t)DC_PKT_BYTES;
    if (sl_rail_set_fixed_length(dc_rail, want) != want) dsm_cap_diag[DC_CFG_ERR] |= 16;
    dc_dual_hw = sl_rail_supports_dual_sync_words(dc_rail);
    dc_dual_st = (int32_t)sl_rail_config_rx_options(dc_rail, SL_RAIL_RX_OPTION_ENABLE_DUAL_SYNC, SL_RAIL_RX_OPTION_ENABLE_DUAL_SYNC);
    dc_dual = dc_dual_hw && (dc_dual_st == (int32_t)SL_RAIL_STATUS_NO_ERROR);
    if (dc_data_mode) {                                      /* data test: the preamble itself is the sync word */
      sl_rail_sync_word_config_t sc = { .sync_word_bits = 32u, .sync_word_0 = DC_SYNC_PRE, .sync_word_1 = DC_SYNC_PRE };
      if (sl_rail_config_sync_words(dc_rail, &sc) != SL_RAIL_STATUS_NO_ERROR) dsm_cap_diag[DC_CFG_ERR] |= 32;
      dc_sync_locked = true;                                 /* no rotation */
    } else { dc_sync_sel = 0; dc_sync_set(dc_sync_sel); }   /* receiver test: sync on the first 32 chips of the first symbol */
  }
  dc_soft_mode = soft;
}

/* sync word 0 = the code (sel 0) or its inverse (sel 1); sync word 1 = the other one, used only
 * when dual sync works.  Without dual sync the packet phase alternates sel every DC_SYNC_ALT_US,
 * since a Lemon whose id has bit 0 clear starts every bind packet with the inverse. */
static void dc_sync_set(uint8_t sel)
{
  sl_rail_sync_word_config_t sc = { .sync_word_bits = 32u, .sync_word_0 = dc_sync_tab[sel][0], .sync_word_1 = dc_sync_tab[sel][1] };
  if (sl_rail_config_sync_words(dc_rail, &sc) != SL_RAIL_STATUS_NO_ERROR) dsm_cap_diag[DC_CFG_ERR] |= 32;
  dc_sync_sel = sel; dc_sync_sel_us = dc_now(); dc_sync_slot_sync = dc_ev_sync; dc_sync_slot_pkt = dc_ev_pkt;
}

/* despread a 1020-byte frame: the sync took the first 32 chips of symbol 0 (its value = which sync word),
 * then 32 chips + 127 x 64 chips follow.  Each symbol is compared with the code: +32 = code = bit 1,
 * -32 = inverse = bit 0.  Both chip orders inside a byte are tried, the better one wins. */
static inline uint8_t dc_chip(const uint8_t *d, uint32_t p, bool msb) { return (uint8_t)((d[p >> 3] >> (msb ? (7u - (p & 7u)) : (p & 7u))) & 1u); }
static void dc_despread(const uint8_t *d, uint8_t sync_id)
{
  uint8_t best_bytes[16]; uint8_t best_q = 0; uint8_t best_order = 0; bool have = false;
  for (uint8_t order = 0; order < 2u; order++) {
    uint8_t bytes[16] = { 0 }; uint8_t q = 32;
    for (uint32_t sym = 0; sym < 128u; sym++) {
      int score, a; bool one;
      if (sym == 0u) {                                       /* symbol 0: its first 32 chips were the sync word, the frame holds the rest */
        uint64_t hi = 0; for (uint32_t i = 0; i < 32u; i++) if (dc_chip(d, i, order)) hi |= (1ull << i);
        uint64_t expect = (DC_CODE64 >> 32) & 0xFFFFFFFFull; if (sync_id) expect ^= 0xFFFFFFFFull;
        uint32_t m = 32u - (uint32_t)__builtin_popcountll(hi ^ expect);   /* 32 = the second half agrees with the sync */
        one = (sync_id == 0u);
        a = (m > 16u) ? (int)(2u * (m - 16u)) : 0;
      } else {
        uint64_t w = 0; uint32_t p0 = 32u + 64u * (sym - 1u);
        for (uint32_t i = 0; i < 64u; i++) if (dc_chip(d, p0 + i, order)) w |= (1ull << i);
        score = 64 - (int)__builtin_popcountll(w ^ DC_CODE64) - 32;   /* +32 code (bit 1), -32 inverse (bit 0) */
        one = (score > 0); a = (score < 0) ? -score : score;
      }
      if (a < q) q = (uint8_t)a;
      if (one) bytes[sym >> 3] |= (uint8_t)(1u << (sym & 7u));         /* LSB first, as on air */
    }
    if (!have || q > best_q) { memcpy(best_bytes, bytes, 16); best_q = q; best_order = order; have = true; }
  }
  memcpy(dc_dec, best_bytes, 16); dc_dec_q = best_q; dc_dec_order = best_order; dc_dec_full = true; dc_dec_n++;
  /* the bind packet's two sums; if they fail as received, try all bits inverted (an FSK symbol
   * map opposite to the Lemon's turns every chip round: the sync words swap roles, the bytes invert) */
  dc_dec_inv = 0;
  for (uint8_t inv = 0; inv < 2u; inv++) {
    uint8_t b[16]; for (int i = 0; i < 16; i++) b[i] = inv ? (uint8_t)~dc_dec[i] : dc_dec[i];
    uint32_t sum = 384u - 0x10u; for (int i = 0; i < 8; i++) sum += b[i];
    bool ok1 = (b[8] == ((sum >> 8) & 0xFFu)) && (b[9] == (sum & 0xFFu));
    for (int i = 8; i < 14; i++) sum += b[i];
    bool ok2 = (b[14] == ((sum >> 8) & 0xFFu)) && (b[15] == (sum & 0xFFu));
    if (ok1 && ok2) { if (inv) { memcpy(dc_dec, b, 16); dc_dec_inv = 1; } dc_dec_ok++; break; }
  }
}

static void dc_rx_fifo(uint16_t q, uint16_t fifo_bytes)
{
  sl_rail_idle(dc_rail, SL_RAIL_IDLE, true);
  uint16_t sz = fifo_bytes;
  (void)sl_rail_set_rx_fifo(dc_rail, dc_fifo, &sz);
  sl_rail_reset_fifo(dc_rail, false, true);
  dc_overflow = false;
  if (sl_rail_start_rx(dc_rail, q, NULL) != SL_RAIL_STATUS_NO_ERROR) dsm_cap_diag[DC_CFG_ERR] |= 8;
  dc_q = q; dsm_cap_diag[DC_CH] = q;
  dc_rx_since = dc_now();
  dc_max = DC_DIAG_NONE;
}

static void dc_rx(uint16_t q)
{
  sl_rail_idle(dc_rail, SL_RAIL_IDLE, true);
  sl_rail_reset_fifo(dc_rail, false, true);                  /* raw modes want an aligned, empty RX buffer */
  dc_overflow = false;
  if (sl_rail_start_rx(dc_rail, q, NULL) != SL_RAIL_STATUS_NO_ERROR) dsm_cap_diag[DC_CFG_ERR] |= 8;
  dc_q = q; dsm_cap_diag[DC_CH] = q;
  dc_rx_since = dc_now();
  dc_max = DC_DIAG_NONE;
}

static void dc_enter_hunt(void)
{
  if (dc_soft_mode) dc_source(false);                        /* hunt in packet mode: RSSI only, nothing to drain */
  dc_state = 1; dsm_cap_diag[DC_STATE] = 1;
  dc_hunt_ch = 0; dc_strong_seen = false;
  dc_h_reads = 0; dc_h_hits = 0; dc_run_reset();
  memset(dc_duty, 0, sizeof(dc_duty)); memset(dc_run, 0, sizeof(dc_run));
  dc_cd_set(false);
  dc_rx(0);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("DC hunt\r\n");
#endif
}

static void dc_ring_reset(void) { dc_b_wr = 0; dc_b_filled = 0; dc_b_post = 0; dc_b_trig = false; dc_b_armed = false; }

/* phases while locked: 2 = burst capture (first), 1 = soft snapshots, 0 = packet mode (unused since v3.18) */
static void dc_enter_phase(uint8_t ph)
{
  dc_ph = ph; dc_ph_since = dc_now();
  dc_cd_set(false);
  if (ph == 0u) dc_ev_tim = dc_ev_timlost = dc_ev_pre = dc_ev_prelost = dc_ev_sync = dc_ev_pkt = dc_ev_abort = dc_ev_ferr = 0;   /* before the slot start is recorded */
  dc_source(ph != 0u);
  if (ph == 2u) {                                            /* burst phase: stream into the ring, wait for an RSSI edge */
    int16_t lv = (int16_t)(dsm_cap_diag[DC_LOCK_RSSI] - DC_TRIG_BELOW);
    dc_b_level = (lv < DC_THRESH_DBM) ? (int16_t)DC_THRESH_DBM : lv;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("DC phase burst ch=%s: trigger at %d dBm\r\n", dc_chs(dc_q), (int)dc_b_level);   /* before RX starts: a print stalls the drain */
#endif
    sl_rail_idle(dc_rail, SL_RAIL_IDLE, true);
    uint16_t sz = DC_FIFO_BYTES; (void)sl_rail_set_rx_fifo(dc_rail, dc_fifo, &sz);   /* the rate measurement may have left 2 kB */
    dc_rx(dc_q);
    if (!dc_rate_done) { dc_rate_step = 0; dc_rx_fifo(dc_q, 2048u); dc_rate_t0 = dc_rx_since; }
    dc_ring_reset(); dc_b_done = false; dc_b_ovf = 0;
    return;
  }
  dc_rx(dc_q);
  if (ph == 1u) {                                     /* soft phase: the first one measures the sample rate by draining */
    if (!dc_rate_done) { dc_rate_step = 0; dc_rx_fifo(dc_q, 2048u); dc_rate_t0 = dc_rx_since; }
    else { dc_snap_t0 = dc_now(); dc_snap_tag = DC_DIAG_NONE; dc_snap_tagged = false; }
    dc_snap_full[0] = dc_snap_full[1] = false; dc_snaps = 0; dc_snap_maxrssi = -128; dc_snap_walls = 0;
  } else {
    dc_ev_tim = dc_ev_timlost = dc_ev_pre = dc_ev_prelost = dc_ev_sync = dc_ev_pkt = dc_ev_abort = dc_ev_ferr = 0;
    dc_pkt_full = false; dc_dec_full = false; dc_dec_n = 0; dc_dec_ok = 0; dc_sync_locked = dc_data_mode;   /* data test: the preamble sync stays */
  }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (ph == 0u && dc_data_mode) app_log("DC phase packet ch=%s: sync=preamble fixed=%u bytes cfg_err=%ld\r\n", dc_chs(dc_q), (unsigned)DC_DATA_BYTES, (long)dsm_cap_diag[DC_CFG_ERR]);
  else if (ph == 0u) app_log("DC phase packet ch=%s: sync0=%08lX sync1=%08lX dual=%d (hw=%d st=%ld)%s fixed=%u bytes cfg_err=%ld\r\n", dc_chs(dc_q),
                        (unsigned long)DC_SYNC0, (unsigned long)DC_SYNC1, dc_dual ? 1 : 0, dc_dual_hw ? 1 : 0, (long)dc_dual_st,
                        dc_dual ? "" : " trying inverse/code/bit-reversed as sync0, 250 ms each", (unsigned)DC_PKT_BYTES, (long)dsm_cap_diag[DC_CFG_ERR]);
  else app_log("DC phase %s ch=%s\r\n", ph ? "soft" : "packet", dc_chs(dc_q));
#endif
}

static void dc_enter_lock(uint16_t q, int16_t rssi)
{
  dc_state = 3; dsm_cap_diag[DC_STATE] = 3;
  dsm_cap_diag[DC_LOCK_RSSI] = rssi;
  dc_q = q; dc_lock_q = q; dc_neighbour_tries = 0;
  uint32_t now = dc_now();
  dc_cd_last = now; dc_cd_win_since = now; dc_cd_high_us = 0; dc_pkt_send_us = now;
  dsm_cap_diag[DC_LONGEST] = 0; dc_cd_sum_us = 0; dc_cd_n = 0;
  sl_gpio_set_pin(&dc_led_b);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("DC lock ch=%s (%lu kHz) %d dBm\r\n", dc_chs(q), (unsigned long)dc_khz(q), (int)rssi);
#endif
  dc_enter_phase(0);                                         /* receiver test first (2 s), then the burst capture, then soft */
}

void dsm_cap_start(sl_rail_handle_t rh, bool fixed, bool rssi_hunt)
{
  dc_rail = rh;
  dc_fixed = fixed;
  dc_rssi_hunt = rssi_hunt;
  memset((void *)dsm_cap_diag, 0, sizeof(dsm_cap_diag));
  for (uint8_t i = 0; i < DC_CHANNELS; i++) { dsm_cap_peak[i] = DC_DIAG_NONE; dsm_cap_scan[i] = DC_DIAG_NONE; dc_recent[i] = DC_DIAG_NONE; }
  dsm_cap_diag[DC_THRESH]  = DC_THRESH_DBM;
  dsm_cap_diag[DC_SUPPORT] = sl_rail_supports_rx_raw_data(rh) ? 1 : 0;

  sl_gpio_set_pin_mode(&dc_led_a, SL_GPIO_MODE_PUSH_PULL, 0);
  sl_gpio_set_pin_mode(&dc_led_b, SL_GPIO_MODE_PUSH_PULL, 0);

  sl_rail_idle(rh, SL_RAIL_IDLE_ABORT, true);
  /* the dsm_raw PHY on the fine grid: same modem config, RAM copy of the channel entry */
  dc_grid_entry = dsm_raw_channelConfig.configs[0];
  dc_grid_entry.baseFrequency = 2400000000u;
  dc_grid_entry.channelSpacing = DC_STEP_KHZ * 1000u;
  dc_grid_entry.physicalChannelOffset = 0;
  dc_grid_entry.channelNumberStart = 0;
  dc_grid_entry.channelNumberEnd = (uint16_t)(DC_QCHANNELS - 1u);
  dc_grid_cfg = dsm_raw_channelConfig;
  dc_grid_cfg.configs = &dc_grid_entry;
  dc_grid_cfg.length = 1u;
  if (sl_rail_config_channels(rh, (const sl_rail_channel_config_t *)&dc_grid_cfg, NULL) != SL_RAIL_STATUS_NO_ERROR) dsm_cap_diag[DC_CFG_ERR] |= 1;
  sl_rail_prepare_channel(rh, DC_FIXED_Q);                           /* load the PHY now */

  uint16_t fsz = DC_FIFO_BYTES;                                      /* bigger RX FIFO: the stock one is 256 bytes */
  if (sl_rail_set_rx_fifo(rh, dc_fifo, &fsz) != SL_RAIL_STATUS_NO_ERROR) dsm_cap_diag[DC_CFG_ERR] |= 4;
  dsm_cap_diag[DC_FIFO_SZ] = fsz;

  /* count what the modem's front end does with the signal */
  sl_rail_events_t ev = SL_RAIL_EVENT_RX_TIMING_DETECT | SL_RAIL_EVENT_RX_TIMING_LOST | SL_RAIL_EVENT_RX_PREAMBLE_DETECT
                      | SL_RAIL_EVENT_RX_PREAMBLE_LOST | SL_RAIL_EVENT_RX_SYNC_0_DETECT | SL_RAIL_EVENT_RX_SYNC_1_DETECT
                      | SL_RAIL_EVENT_RX_PACKET_RECEIVED | SL_RAIL_EVENT_RX_PACKET_ABORTED | SL_RAIL_EVENT_RX_FRAME_ERROR
                      | SL_RAIL_EVENT_RX_FIFO_OVERFLOW;
  sl_rail_config_events(rh, ev, ev);
  dc_soft_mode = true;                                               /* force the packet-mode setup below */
  dc_source(false);

#if DC_PRS_DOUT
  /* experiment: the modem's DOUT / DCLK exist as PRS producers on the MG24 even though RAIL
   * has no direct mode here.  Async channel 6 -> PC03, channel 0 -> PA03.  Harmless if silent. */
  CMU->CLKEN0_SET = CMU_CLKEN0_PRS;
  PRS->ASYNC_CH[6].CTRL = PRS_ASYNC_CH_CTRL_FNSEL_A | PRS_ASYNC_CH_CTRL_SOURCESEL_MODEML | PRS_ASYNC_CH_CTRL_SIGSEL_MODEMLDOUT;
  PRS->ASYNC_CH[0].CTRL = PRS_ASYNC_CH_CTRL_FNSEL_A | PRS_ASYNC_CH_CTRL_SOURCESEL_MODEML | PRS_ASYNC_CH_CTRL_SIGSEL_MODEMLDCLK;
  GPIO->PRSROUTE[0].ASYNCH6ROUTE = ((uint32_t)SL_GPIO_PORT_C << _GPIO_PRS_ASYNCH6ROUTE_PORT_SHIFT) | (3u << _GPIO_PRS_ASYNCH6ROUTE_PIN_SHIFT);
  GPIO->PRSROUTE[0].ASYNCH0ROUTE = ((uint32_t)SL_GPIO_PORT_A << _GPIO_PRS_ASYNCH0ROUTE_PORT_SHIFT) | (3u << _GPIO_PRS_ASYNCH0ROUTE_PIN_SHIFT);
  GPIO->PRSROUTE[0].ROUTEEN |= GPIO_PRS_ROUTEEN_ASYNCH6PEN | GPIO_PRS_ROUTEEN_ASYNCH0PEN;
  sl_gpio_set_pin_mode(&dc_dout_pin, SL_GPIO_MODE_PUSH_PULL, 0);
  sl_gpio_set_pin_mode(&dc_dclk_pin, SL_GPIO_MODE_PUSH_PULL, 0);
#endif

#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("DC boot v3.32 raw_ok=%ld cfg_err=%ld fifo=%u thresh=%d grid=%ukHz fixed=%d hunt=%s\r\n", (long)dsm_cap_diag[DC_SUPPORT],
          (long)dsm_cap_diag[DC_CFG_ERR], (unsigned)fsz, DC_THRESH_DBM, (unsigned)DC_STEP_KHZ, fixed ? 1 : 0, rssi_hunt ? "strongest-any" : "strongest-long-burst");
#endif
  dc_uart_us = dc_now();
  dc_data_mode = fixed;
  if (rssi_hunt) {                                                   /* v3.31: BIND at boot = bind sniffer: sit on the bind channel */
    dc_fixed = true; dc_data_mode = false; dc_rssi_hunt = false;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("DC bind sniffer: fixed on ch=%u (%u MHz), receiver test (DCK) / burst capture (DCD) cycling; power the transmitter with BIND\r\n",
            (unsigned)DC_SNIFF_CH, 2400u + (unsigned)DC_SNIFF_CH);
#endif
    dc_enter_lock((uint16_t)(DC_SNIFF_CH * DC_QPC), -55);           /* burst trigger = this - DC_TRIG_BELOW = -70 dBm */
    return;
  }
  if (fixed) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("DC data test: ch=%u (%lu kHz), sync=preamble %08lX, frames of %u bytes, first %u printed\r\n", (unsigned)DC_DATA_CH,
            (unsigned long)dc_khz((uint16_t)(DC_DATA_CH * DC_QPC)), (unsigned long)DC_SYNC_PRE, (unsigned)DC_DATA_BYTES, (unsigned)DC_DATA_PRINT);
#endif
    dc_census = true; dc_census_ch = DC_CENSUS_LO; dc_census_pass = 0;
    memset(dc_census_pre, 0, sizeof(dc_census_pre)); memset(dc_census_pkt, 0, sizeof(dc_census_pkt));
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("DC census: channels %u..%u, %lu ms each, %u passes: which channels carry frames with the 1001 preamble\r\n",
            (unsigned)DC_CENSUS_LO, (unsigned)DC_CENSUS_HI, (unsigned long)(DC_CENSUS_US / 1000u), (unsigned)DC_CENSUS_PASSES);
#endif
    dc_enter_lock((uint16_t)(DC_CENSUS_LO * DC_QPC), -40);
    dc_census_since = dc_now();
  } else dc_enter_hunt();
}

static void dc_pk_step_reset(void) { dc_pk_score = 0; dc_pk_sat = 0; dc_pk_rssi = DC_DIAG_NONE; dc_pk_mid = 0; dc_pk_tag = DC_DIAG_NONE; dc_pk_ready = false; }

/* run a list of peak-search steps: q = first + i * step, i < n */
static void dc_pk_begin(uint8_t stage, uint16_t first, uint16_t step, uint16_t n)
{
  dc_pk_stage = stage; dc_pk_first = first; dc_pk_step = step; dc_pk_n = n; dc_pk_i = 0;
  dc_pk_step_reset();
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("DC peak search %s: %s .. %s in %u kHz steps\r\n", stage == 1u ? "coarse" : "fine", dc_chs(first),
          dc_chs((uint16_t)(first + (n - 1u) * step)), (unsigned)(step * DC_STEP_KHZ));
#endif
  dc_rx(first); dc_pk_snap_t0 = dc_now();
}

static void dc_start_peak_search(uint8_t c, int16_t r)
{
  uint16_t q = (uint16_t)c * DC_QPC;
  uint16_t first = (q >= DC_QPC) ? (uint16_t)(q - DC_QPC) : 0u;      /* c-1 .. c+2 */
  if (first + DC_PEAK_COARSE * DC_QPC > DC_QCHANNELS) first = (uint16_t)(DC_QCHANNELS - DC_PEAK_COARSE * DC_QPC);
  dc_pk_c = c; dc_pk_snap_full = false; dc_pk_snap_rssi = DC_DIAG_NONE;
  dc_state = 4; dsm_cap_diag[DC_STATE] = 4;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("DC strong ch=%u %d dBm: peak search\r\n", (unsigned)c, (int)r);
#endif
  dc_source(true);                                                   /* judge each step by its demod output */
  dc_pk_begin(1u, first, (uint16_t)DC_QPC, DC_PEAK_COARSE);
}

/* end of a hunt sweep: pick the winner */
static void dc_sweep_done(void)
{
  int16_t best = DC_DIAG_NONE, second = DC_DIAG_NONE, floor = 127;
  uint8_t bch = 0, sch = 0;
  for (uint8_t i = 0; i < DC_CHANNELS; i++) {
    int16_t v = dsm_cap_scan[i];
    if (v == DC_DIAG_NONE) continue;
    if (v < floor) floor = v;
    if (v > best) { second = best; sch = bch; best = v; bch = i; }
    else if (v > second) { second = v; sch = i; }
  }
  /* the three busiest channels (duty hunt) */
  uint8_t top[3] = { 0, 0, 0 }; uint8_t topd[3] = { 0, 0, 0 };
  for (uint8_t i = 0; i < DC_CHANNELS; i++) {
    uint8_t d = dc_duty[i];
    if (d == 0u) continue;
    for (uint8_t k = 0; k < 3; k++) {
      if (d > topd[k] || (d == topd[k] && dsm_cap_scan[i] > dsm_cap_scan[top[k]])) {
        for (uint8_t m = 2; m > k; m--) { top[m] = top[m - 1]; topd[m] = topd[m - 1]; }
        top[k] = i; topd[k] = d; break;
      }
    }
  }
  dsm_cap_diag[DC_SWEEPS]++;
  dsm_cap_diag[DC_BEST_CH] = bch; dsm_cap_diag[DC_BEST_RSSI] = best;
  dsm_cap_diag[DC_2ND_CH]  = sch; dsm_cap_diag[DC_2ND_RSSI]  = second;
  dsm_cap_diag[DC_FLOOR]   = (floor == 127) ? DC_DIAG_NONE : floor;
  sl_gpio_toggle_pin(&dc_led_b);
  bool quiet = (dsm_cap_diag[DC_SWEEPS] % 5) != 0 && best < DC_STRONG_DBM;   /* 5 sweeps/s: print every 5th, or a strong one */
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (!quiet) app_log("DC sweep %ld: busy ch=%u %u%% %d dBm, ch=%u %u%% %d, ch=%u %u%% %d | strongest ch=%u %d, floor %d\r\n",
            (long)dsm_cap_diag[DC_SWEEPS], (unsigned)top[0], (unsigned)topd[0], (int)dsm_cap_scan[top[0]],
            (unsigned)top[1], (unsigned)topd[1], (int)dsm_cap_scan[top[1]], (unsigned)top[2], (unsigned)topd[2], (int)dsm_cap_scan[top[2]],
            (unsigned)bch, (int)best, (int)dsm_cap_diag[DC_FLOOR]);
#endif
  /* candidates: the three strongest channels that were at least DC_ACTIVE_DUTY busy; each is
   * confirmed in turn for DC_CONFIRM_US, long enough to see whole bursts (the 4 ms dwell is not) */
  /* a 4 ms dwell overlaps a 34/s bind burst in only ~40 % of the sweeps, so rank by the peak of
   * the last few sweeps: dc_recent decays DC_RECENT_DECAY dB per sweep and is refreshed by any hit */
  for (uint8_t i = 0; i < DC_CHANNELS; i++) {
    int16_t v = dc_recent[i]; if (v > DC_DIAG_NONE + DC_RECENT_DECAY) v = (int16_t)(v - DC_RECENT_DECAY); else v = DC_DIAG_NONE;
    if (dc_duty[i] >= DC_ACTIVE_DUTY && dsm_cap_scan[i] != DC_DIAG_NONE && dsm_cap_scan[i] > v) v = dsm_cap_scan[i];
    dc_recent[i] = (int8_t)v;
  }
  dc_cand_n = 0;
  for (uint8_t k = 0; k < 3; k++) {
    int16_t b = DC_DIAG_NONE; uint8_t bc = 0; bool found = false;
    for (uint8_t i = 0; i < DC_CHANNELS; i++) {
      if (dc_recent[i] < DC_THRESH_DBM) continue;
      bool used = false; for (uint8_t m = 0; m < dc_cand_n; m++) if (dc_cand[m] == i) used = true;
      if (used) continue;
      if (dc_recent[i] > b) { b = dc_recent[i]; bc = i; found = true; }
    }
    if (!found) break;
    dc_cand[dc_cand_n++] = bc;
  }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (!quiet) {
    app_log("DC sweep %ld: candidates", (long)dsm_cap_diag[DC_SWEEPS]);
    for (uint8_t m = 0; m < dc_cand_n; m++) app_log(" ch=%u %d(%d) dBm %u%% %u us", (unsigned)dc_cand[m], (int)dc_recent[dc_cand[m]], (int)dsm_cap_scan[dc_cand[m]], (unsigned)dc_duty[dc_cand[m]], (unsigned)dc_run[dc_cand[m]]);
    app_log("\r\n");
  }
#endif
  /* default mode: only the instant lock on a strong reading (>= DC_STRONG_DBM) is used; the
   * candidate/confirm path is for F/S mode (a weak or distant transmitter), WiFi passes it too often */
  /* strong channels (>= DC_STRONG_DBM, the transmitter within a metre): with several - a mouse on
   * the desk is as strong as the transmitter - take the one with the longest burst in its dwell
   * (DSM bind 8 ms, a mouse 0.5 ms), then the strongest, and go to the peak search */
  if (dc_strong_seen && !dc_rssi_hunt) {
    uint8_t sc = 0; int16_t sr = DC_DIAG_NONE; uint16_t srun = 0; bool found = false;
    for (uint8_t i = 0; i < DC_CHANNELS; i++) {
      if (dsm_cap_scan[i] == DC_DIAG_NONE || dsm_cap_scan[i] < DC_STRONG_DBM) continue;
      if (i >= DC_SKIP_LO && i <= DC_SKIP_HI) continue;
      bool longer = (dc_run[i] >= 1000u) && (srun < 1000u);
      bool same   = ((dc_run[i] >= 1000u) == (srun >= 1000u));
      if (!found || longer || (same && dsm_cap_scan[i] > sr)) { sc = i; sr = dsm_cap_scan[i]; srun = dc_run[i]; found = true; }
    }
    dc_strong_seen = false;
    if (found) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("DC sweep %ld: strong ch=%u %d dBm burst %u us\r\n", (long)dsm_cap_diag[DC_SWEEPS], (unsigned)sc, (int)sr, (unsigned)srun);
#endif
      dc_start_peak_search(sc, sr);
      return;
    }
  }
  bool go = dc_rssi_hunt && (dc_cand_n > 0u); uint8_t ch = dc_cand[0]; dc_cand_i = 0;
  (void)top; (void)topd; (void)floor;
  if (go) {
    dc_state = 2; dsm_cap_diag[DC_STATE] = 2;
    dc_phase_since = dc_now();
    dc_h_reads = 0; dc_h_hits = 0; dc_run_reset();
    dc_rx((uint16_t)ch * DC_QPC);
  } else {
    dc_hunt_ch = 0;
    dc_rx(0);
  }
}

#if defined(SL_CATALOG_APP_LOG_PRESENT)
static void dc_send_hex(const uint8_t *d, uint16_t len)
{
  static const char hx[] = "0123456789ABCDEF";
  static char line[130];
  for (uint16_t i = 0; i < len; i += 64u) {
    uint16_t n = (uint16_t)(len - i); if (n > 64u) n = 64u;
    for (uint16_t k = 0; k < n; k++) { line[2 * k] = hx[d[i + k] >> 4]; line[2 * k + 1] = hx[d[i + k] & 0x0Fu]; }
    line[2 * n] = '\0';
    app_log("%s\r\n", line);
  }
  app_log("DCE\r\n");
  dsm_cap_diag[DC_SENT]++;
}
#endif

void dsm_cap_on_events(sl_rail_events_t events)
{
  if (events & SL_RAIL_EVENT_RX_FIFO_OVERFLOW) { dc_overflow = true; dc_ovf_us = (uint32_t)sl_rail_get_time(dc_rail); dsm_cap_diag[DC_OVERFLOW]++; }
  if (events & SL_RAIL_EVENT_RX_TIMING_DETECT)   dc_ev_tim++;
  if (events & SL_RAIL_EVENT_RX_TIMING_LOST)     dc_ev_timlost++;
  if (events & SL_RAIL_EVENT_RX_PREAMBLE_DETECT) dc_ev_pre++;
  if (events & SL_RAIL_EVENT_RX_PREAMBLE_LOST)   dc_ev_prelost++;
  if (events & (SL_RAIL_EVENT_RX_SYNC_0_DETECT | SL_RAIL_EVENT_RX_SYNC_1_DETECT)) dc_ev_sync++;
  if (events & SL_RAIL_EVENT_RX_PACKET_RECEIVED) dc_ev_pkt++;
  if (events & SL_RAIL_EVENT_RX_PACKET_ABORTED)  dc_ev_abort++;
  if (events & SL_RAIL_EVENT_RX_FRAME_ERROR)     dc_ev_ferr++;
}

/* ---- DSMX data packet decoder (v3.28) ---- */
static uint64_t dc_code64(const uint8_t *c) { uint64_t w = 0; for (int i = 0; i < 8; i++) w |= (uint64_t)c[i] << (8 * i); return w; }   /* chip i = bit i */
static inline uint64_t dc_rotr64(uint64_t w, unsigned n) { n &= 63u; return n ? ((w >> n) | (w << (64u - n))) : w; }
static void dc_sym_build(uint8_t row)
{
  uint64_t a = dc_code64(dc_pn[row][DC_DATA_COL]), b = dc_code64(dc_pn[row][DC_DATA_COL + 1u]);
  for (unsigned v = 0; v < 256u; v++) {
    uint64_t w = dc_rotr64((v & 0x40u) ? b : a, (v - 1u) & 63u);   /* symbol chip i = code chip (i + (v & 63) - 1) mod 64 */
    dc_sym[v] = (v & 0x80u) ? w : ~w;
  }
  dc_sop64 = dc_code64(dc_pn[row][DC_SOP_COL]);
  dc_sym_row = row;
}
static uint16_t dc_crc16(const uint8_t *d, uint32_t n, uint16_t seed)
{
  uint16_t crc = 0; for (int i = 0; i < 16; i++) if (seed & (1u << i)) crc |= (uint16_t)(0x8000u >> i);   /* init = bit-reversed seed */
  for (uint32_t k = 0; k < n; k++) { crc ^= d[k]; for (int i = 0; i < 8; i++) crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1); }
  return (uint16_t)(crc ^ 0xFFFFu);
}
/* 64 chips from bit position p of the frame (chip i = bit i & 7 of byte i >> 3) */
static inline uint64_t dc_get64(const uint64_t *w, uint32_t p)
{
  uint32_t i = p >> 6, b = p & 63u;
  return b ? ((w[i] >> b) | (w[i + 1] << (64u - b))) : w[i];
}
static void dc_dt_add(uint32_t t, int8_t rssi, char kind, char pos)
{
  if (dc_dt_n < DC_DT_N) { dc_dt[dc_dt_n].t = t; dc_dt[dc_dt_n].rssi = rssi; dc_dt[dc_dt_n].kind = kind; dc_dt[dc_dt_n].pos = pos; dc_dt_n++; }
  else dc_dt_drop++;
}
/* kind in the DCT line: A = upper frame (byte 3 bit 7 set: channels 0 7 3 8 9 11), B = lower frame
 * (channels 1 5 2 4 6 10), x = symbols decoded but the CRC failed, n = no SOP found (a weak frame
 * from another channel of the hop set); then 1 or 2 = which CRC seed checked = first or second
 * packet of the 4 ms pair (Multi-module: CH1 uses ~(id0<<8|id1), CH2 uses id0<<8|id1) */
static void dc_decode_data(const uint8_t *frame, uint16_t n, uint8_t rf_ch, uint32_t t_us, int8_t rssi)
{
  static uint64_t w[DC_DATA_BYTES / 8u + 2u];
  uint8_t row = (uint8_t)((rf_ch + 5u - 3u) % 5u);
  if (row != dc_sym_row) dc_sym_build(row);
  memset(w, 0, sizeof(w)); memcpy(w, frame, n);
  uint32_t bits = (uint32_t)n * 8u;
  uint32_t sop = 0; bool found = false;
  for (uint32_t p = 0; p + 64u <= bits && p < 400u; p++) {
    if (__builtin_popcountll(dc_get64(w, p) ^ dc_sop64) <= 4) { sop = p; found = true; break; }
  }
  dc_dx_n++;
  uint32_t p0 = sop + 64u + 4u + 64u;
  if (!found || p0 + 64u * DC_DATA_SYMS > bits) { dc_dx_nosop++; dc_dt_add(t_us, rssi, 'n', 0); return; }
  uint8_t q = 64;
  for (uint32_t k = 0; k < DC_DATA_SYMS; k++) {
    uint64_t sy = dc_get64(w, p0 + 64u * k);
    unsigned best_v = 0; int best_d = 65;
    for (unsigned v = 0; v < 256u; v++) { int d = __builtin_popcountll(sy ^ dc_sym[v]); if (d < best_d) { best_d = d; best_v = v; } }
    dc_dx[k] = (uint8_t)best_v; if (64 - best_d < q) q = (uint8_t)(64 - best_d);
  }
  uint16_t c0 = dc_crc16(dc_dx, 17u, DC_CRC_SEED), c1 = dc_crc16(dc_dx, 17u, (uint16_t)~DC_CRC_SEED);
  uint16_t got = (uint16_t)(dc_dx[17] | (dc_dx[18] << 8));
  dc_dx_seed = (got == c0) ? 0u : (got == c1) ? 1u : 2u;
  if (dc_dx_seed < 2u) dc_dx_crc_ok++;
  dc_dx_q = q; dc_dx_sop = (uint16_t)sop; dc_dx_rssi = rssi; dc_dx_full = true;
  dc_dt_add(t_us, rssi, dc_dx_seed == 2u ? 'x' : (dc_dx[3] & 0x80u) ? 'A' : 'B', dc_dx_seed == 0u ? '1' : dc_dx_seed == 1u ? '2' : 0);
}

/* packet phase: a frame is held in RAIL (app_process holds it on RX_PACKET_RECEIVED) */
void dsm_cap_rx_process(void)
{
  sl_rail_rx_packet_info_t info;
  sl_rail_rx_packet_handle_t h = sl_rail_get_rx_packet_info(dc_rail, SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &info);
  if (h == SL_RAIL_RX_PACKET_HANDLE_INVALID) return;
  if (!dc_soft_mode) {
    static uint8_t frame[DC_DATA_BYTES];
    uint16_t n = info.packet_bytes; if (n > DC_DATA_BYTES) n = DC_DATA_BYTES;
    sl_rail_rx_packet_details_t det; memset(&det, 0, sizeof(det));
    det.time_received.time_position = SL_RAIL_PACKET_TIME_DEFAULT;
    (void)sl_rail_get_rx_packet_details(dc_rail, h, &det);
    det.time_received.total_packet_bytes = info.packet_bytes;
    (void)sl_rail_get_rx_time_sync_word_end(dc_rail, &det);  /* packet time = end of the sync word (v3.28) */
    info.packet_bytes = n;
    sl_rail_copy_rx_packet(dc_rail, frame, &info);
    if (dc_data_mode) {                                      /* data test: decode, and keep the head of the frame for printing */
      if (!dc_census) dc_decode_data(frame, n, dc_q2ch(dc_q), (uint32_t)det.time_received.packet_time, det.rssi_dbm);
      if (!dc_pkt_full) { uint16_t k = (n < DC_DATA_PRINT) ? n : (uint16_t)DC_DATA_PRINT; memcpy(dc_pkt, frame, k); dc_pkt_len = k; dc_pkt_rssi = det.rssi_dbm; dc_pkt_sync = det.sync_word_id; dc_pkt_full = true; }
    } else if (n == DC_PKT_BYTES) {
      dc_despread(frame, (uint8_t)(det.sync_word_id ^ ((dc_sync_sel & 1u) ? 0u : 1u)));   /* sync0 = inverse in variants 0 and 2 */
      if (!dc_pkt_full) { memcpy(dc_pkt, frame, n); dc_pkt_len = n; dc_pkt_rssi = det.rssi_dbm; dc_pkt_sync = det.sync_word_id; dc_pkt_full = true; }
    }
  }
  sl_rail_release_rx_packet(dc_rail, h);
}

/* soft phase: pull the frozen FIFO after an overflow, keep it if it beats what we have.
 * Two slots: the best and the second best taken at least DC_SNAP_APART_US later/earlier. */
/* score a snapshot: percent saturated, but 0 for a "wall" (the discriminator pinned at one rail
 * because the signal is on a neighbouring channel: hardly any zero crossings) */
static uint16_t dc_snap_score(const uint8_t *d, uint16_t n, uint16_t *p_sat, uint16_t *p_tr)
{
  uint16_t sat = 0, tr = 0; int32_t sum = 0;
  for (uint16_t i = 0; i < n; i++) { int8_t v = (int8_t)d[i]; sum += v; if (v >= DC_SNAP_SAT || v <= -DC_SNAP_SAT) sat++; }
  int8_t mid = (int8_t)(sum / (int32_t)n);
  bool above = ((int8_t)d[0] > mid);
  for (uint16_t i = 1; i < n; i++) { bool a = ((int8_t)d[i] > mid); if (a != above) { tr++; above = a; } }
  *p_sat = sat; *p_tr = tr;
  uint16_t pct = (uint16_t)((uint32_t)sat * 100u / n);
  if (tr < n / 20u) return 0;                       /* < 5 % crossings: pinned                       */
  return pct;
}

/* where the demod output sits: midpoint of the 10th and 90th percentile.  0 = the LO is on the
 * carrier (a two-level signal is symmetric about it whatever its chip balance), -128/+127 = pinned */
static int16_t dc_snap_mid(const uint8_t *d, uint16_t n)
{
  static uint16_t h[256];
  memset(h, 0, sizeof(h));
  for (uint16_t i = 0; i < n; i++) h[(uint8_t)(d[i] ^ 0x80u)]++;   /* index 0 = -128 .. 255 = +127 */
  uint32_t acc = 0; int16_t p10 = -128, p90 = 127; bool got10 = false;
  for (uint16_t k = 0; k < 256u; k++) {
    acc += h[k];
    if (!got10 && acc * 10u >= n)      { p10 = (int16_t)k - 128; got10 = true; }
    if (acc * 10u >= (uint32_t)n * 9u) { p90 = (int16_t)k - 128; break; }
  }
  return (int16_t)((p10 + p90) / 2);
}

static void dc_take_snapshot(uint32_t now)
{
  uint16_t got = 0, sat = 0, tr = 0;
  uint16_t avail = sl_rail_get_rx_fifo_bytes_available(dc_rail);
  if (avail > DC_FIFO_BYTES) avail = DC_FIFO_BYTES;
  static uint8_t tmp[DC_FIFO_BYTES];
  while (got < avail) {
    uint16_t n = sl_rail_read_rx_fifo(dc_rail, &tmp[got], (uint16_t)(avail - got));
    if (n == 0u) break;
    got = (uint16_t)(got + n);
  }
  dc_snaps++;
  if (got < 1024u) return;
  uint16_t score = dc_snap_score(tmp, got, &sat, &tr);
  int16_t  mid   = dc_snap_mid(tmp, got);
  uint16_t pct   = (uint16_t)((uint32_t)sat * 100u / got);
  if (score == 0u) dc_snap_walls++;
  /* v3.16: rank by the RSSI read while the snapshot filled, not by saturation - with no signal the
   * discriminator swings rail to rail (45 % saturated), a strong on-frequency signal gives a small
   * centred swing.  The two strongest, at least DC_SNAP_APART_US apart, are kept. */
  int16_t tag = (dc_snap_tag == DC_DIAG_NONE) ? (int16_t)-128 : dc_snap_tag;
  if (tag > dc_snap_maxrssi) dc_snap_maxrssi = tag;
  int slot = -1;
  if (!dc_snap_full[0] || tag > dc_snap_rssi[0]) {
    if (dc_snap_full[0] && (now - dc_snap_time[0]) >= DC_SNAP_APART_US) {          /* old best becomes second */
      memcpy(dc_snap[1], dc_snap[0], dc_snap_len[0]);
      dc_snap_len[1] = dc_snap_len[0]; dc_snap_sat[1] = dc_snap_sat[0]; dc_snap_rssi[1] = dc_snap_rssi[0]; dc_snap_tr[1] = dc_snap_tr[0];
      dc_snap_mid_[1] = dc_snap_mid_[0]; dc_snap_time[1] = dc_snap_time[0]; dc_snap_full[1] = true;
    }
    slot = 0;
  } else if ((!dc_snap_full[1] || tag > dc_snap_rssi[1]) && (now - dc_snap_time[0]) >= DC_SNAP_APART_US) slot = 1;
  if (slot < 0) return;
  memcpy(dc_snap[slot], tmp, got);
  dc_snap_len[slot] = got; dc_snap_sat[slot] = pct; dc_snap_tr[slot] = tr; dc_snap_mid_[slot] = mid; dc_snap_rssi[slot] = tag;
  dc_snap_time[slot] = now; dc_snap_full[slot] = true;
}

/* ---- burst phase (v3.18): stream the demod samples through the ring, trigger on an RSSI edge ---- */
static bool dc_pk_snap_pending;                  /* the peak search's snapshot waits until the burst phase is over */

#if defined(SL_CATALOG_APP_LOG_PRESENT)

static void dc_burst_print(void)
{
  /* the whole window, every DC_DEC-th sample (3 samples per chip): 32768 samples, 5.8 s of UART.
   * Chip timing is recovered on the PC (dsm_analyse.py fits the symbol grid); v3.18's hard
   * chips sliced here with the measured sample rate slipped one chip every 150 (the two-fill
   * rate measurement is only good to 0.5 %). */
  uint32_t start = (dc_b_trig_pos + DC_RING_BYTES - DC_RING_PRE) % DC_RING_BYTES;   /* window: 1 ms before the edge .. */
  static uint8_t seg[DC_RING_BYTES / DC_DEC];
  for (uint32_t i = 0; i < DC_RING_BYTES / DC_DEC; i++) seg[i] = dc_ring[(start + i * DC_DEC) % DC_RING_BYTES];
  app_log("DCD ch=%s f=%lu rssi=%d len=%lu dec=%u pre=%lu fs=%lu ovf=%lu\r\n", dc_chs(dc_q), (unsigned long)dc_khz(dc_q), (int)dc_b_rssi,
          (unsigned long)(DC_RING_BYTES / DC_DEC), (unsigned)DC_DEC, (unsigned long)(DC_RING_PRE / DC_DEC), (unsigned long)dc_rate_kss, (unsigned long)dc_b_ovf);
  static const char hx[] = "0123456789ABCDEF";
  static char line[130];
  for (uint32_t i = 0; i < DC_RING_BYTES / DC_DEC; i += 64u) {
    for (uint32_t k = 0; k < 64u; k++) { uint8_t v = seg[i + k]; line[2 * k] = hx[v >> 4]; line[2 * k + 1] = hx[v & 0x0Fu]; }
    line[128] = '\0';
    app_log("%s\r\n", line);
  }
  app_log("DCE\r\n");
  dsm_cap_diag[DC_SENT]++;
}
#endif

static void dc_burst_end(void)
{
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (dc_pk_snap_pending) {                                        /* the peak search's strongest snapshot, held back until now */
    app_log("%s ch=%s f=%lu rssi=%d len=%u sat=%u tr=%u mid=%d fs=%lu peak=1\r\n", (dc_pk_snap_tr < dc_pk_snap_len / 20u) ? "DCW" : "DCS",
            dc_chs(dc_pk_snap_q), (unsigned long)dc_khz(dc_pk_snap_q), (int)dc_pk_snap_rssi, (unsigned)dc_pk_snap_len, (unsigned)dc_pk_snap_sat,
            (unsigned)dc_pk_snap_tr, (int)dc_pk_snap_mid, (unsigned long)dc_rate_kss);
    dc_send_hex(dc_pk_snap, dc_pk_snap_len);
  }
#endif
  dc_pk_snap_pending = false;
  dc_enter_phase(1);
}

static void dc_burst_service(uint32_t now)
{
  uint16_t avail = sl_rail_get_rx_fifo_bytes_available(dc_rail);   /* drain the RAIL FIFO into the ring */
  while (avail) {
    uint32_t room = DC_RING_BYTES - dc_b_wr;
    uint16_t n = (avail < room) ? avail : (uint16_t)room;
    n = sl_rail_read_rx_fifo(dc_rail, &dc_ring[dc_b_wr], n);
    if (n == 0u) break;
    dc_b_wr = (dc_b_wr + n) % DC_RING_BYTES;
    if (dc_b_filled < DC_RING_BYTES) dc_b_filled += n;
    if (dc_b_trig) dc_b_post += n;
    avail = (uint16_t)(avail - n);
  }
  if (dc_overflow) {                                                 /* the loop stalled (a print?): start over */
    dc_b_ovf++;
    dc_rx(dc_q); dc_ring_reset();
    return;
  }
  if (dc_b_trig) {
    if (dc_b_post >= DC_RING_BYTES - DC_RING_PRE) {                  /* the window is complete */
      sl_rail_idle(dc_rail, SL_RAIL_IDLE, true);
      dc_b_done = true; dsm_cap_diag[DC_CAPTURED]++;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("DC burst: edge %d dBm after %lu ms, printing\r\n", (int)dc_b_rssi, (unsigned long)((now - dc_ph_since) / 1000u));
      dc_burst_print();
#endif
      dc_burst_end();
    }
    return;
  }
  if ((now - dc_rx_since) >= DC_SETTLE_US) {
    int16_t r = dc_rssi();
    if (r != DC_DIAG_NONE) {
      if (r > dsm_cap_peak[dc_q2ch(dc_q)]) dsm_cap_peak[dc_q2ch(dc_q)] = (int8_t)r;
      if (r < dc_b_level) dc_b_armed = true;                         /* seen the gap: the next rise is a burst start */
      else if (dc_b_armed && dc_b_filled >= DC_RING_PRE) { dc_b_trig = true; dc_b_trig_pos = dc_b_wr; dc_b_post = 0; dc_b_rssi = r; dsm_cap_diag[DC_BURSTS]++; dc_cd_set(true); }
    }
  }
  if ((now - dc_ph_since) >= DC_BURST_US) {
    sl_rail_idle(dc_rail, SL_RAIL_IDLE, true);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("DC burst phase: no edge above %d dBm in %lu ms (%lu overflows)\r\n", (int)dc_b_level, (unsigned long)(DC_BURST_US / 1000u), (unsigned long)dc_b_ovf);
#endif
    dc_burst_end();
  }
}

static void dc_lock_service(uint32_t now, bool bind_held)
{
  /* ---- soft phase: sample-rate window, then free-running snapshots (restart, fill, freeze, read) ---- */
  if (dc_soft_mode) {
    if (!dc_rate_done) {                                             /* let the FIFO fill and freeze, twice */
      if (dc_overflow) {
        uint32_t fill = dc_ovf_us - dc_rate_t0;
        if (dc_rate_step == 0u) { dc_rate_t2 = fill; dc_rate_step = 1u; dc_rx_fifo(dc_q, 4096u); dc_rate_t0 = dc_rx_since; return; }
        dc_rate_t4 = fill;
        uint32_t d = (dc_rate_t4 > dc_rate_t2) ? (dc_rate_t4 - dc_rate_t2) : 1u;
        dc_rate_kss = (2048u * 1000u) / d;                           /* samples per us x 1000 */
        dc_rate_done = true;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("DC rate: 2048 fill %lu us, 4096 fill %lu us -> %lu kS/s\r\n", (unsigned long)dc_rate_t2, (unsigned long)dc_rate_t4, (unsigned long)dc_rate_kss);
#endif
        dc_rx(dc_q); dc_snap_t0 = now; dc_snap_tag = DC_DIAG_NONE; dc_snap_tagged = false; dc_ring_reset();
      } else if ((now - dc_rate_t0) > 50000u) {                        /* never filled?! */
        dc_rate_done = true; dc_rate_kss = 0;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("DC rate: FIFO did not fill\r\n");
#endif
        dc_rx(dc_q); dc_snap_t0 = now; dc_snap_tag = DC_DIAG_NONE; dc_snap_tagged = false; dc_ring_reset();
      }
      return;
    }
    if (dc_ph == 2u) { dc_burst_service(now); return; }
    /* RX is running and the FIFO is filling; tag the snapshot with one RSSI read, then wait for the freeze */
    if (!dc_snap_tagged && (now - dc_rx_since) >= DC_SETTLE_US) {
      int16_t r = dc_rssi();
      if (r != DC_DIAG_NONE) {
        dc_snap_tag = r; dc_snap_tagged = true;
        if (r > dsm_cap_peak[dc_q2ch(dc_q)]) dsm_cap_peak[dc_q2ch(dc_q)] = (int8_t)r;
        if (r >= DC_THRESH_DBM) { dc_cd_last = now; if (!dc_cd) { dsm_cap_diag[DC_BURSTS]++; dc_cd_set(true); } }
        else if (dc_cd && (now - dc_cd_last) > 20000u) dc_cd_set(false);
      }
    }
    if (dc_overflow) {                                                 /* FIFO full and frozen: read, restart */
      dc_take_snapshot(now);
      dc_rx(dc_q); dc_snap_t0 = now; dc_snap_tag = DC_DIAG_NONE; dc_snap_tagged = false;
    } else if ((now - dc_snap_t0) > 20000u) {                          /* never filled: restart */
      dc_rx(dc_q); dc_snap_t0 = now; dc_snap_tag = DC_DIAG_NONE; dc_snap_tagged = false;
    }
    if ((now - dc_ph_since) >= DC_SOFT_US) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("DC soft phase ch=%s: %lu snapshots, %lu pinned, strongest %d dBm\r\n",
              dc_chs(dc_q), (unsigned long)dc_snaps, (unsigned long)dc_snap_walls, (int)dc_snap_maxrssi);
      for (int k = 0; k < 2; k++) {                                  /* the strongest always, the second only if it holds a signal */
        if (!dc_snap_full[k]) continue;
        if (k == 1 && dc_snap_rssi[k] < DC_THRESH_DBM) continue;
        app_log("%s ch=%s f=%lu rssi=%d len=%u sat=%u tr=%u mid=%d fs=%lu\r\n", (dc_snap_tr[k] < dc_snap_len[k] / 20u) ? "DCW" : "DCS",
                dc_chs(dc_q), (unsigned long)dc_khz(dc_q), (int)dc_snap_rssi[k], (unsigned)dc_snap_len[k], (unsigned)dc_snap_sat[k],
                (unsigned)dc_snap_tr[k], (int)dc_snap_mid_[k], (unsigned long)dc_rate_kss);
        dc_send_hex(dc_snap[k], dc_snap_len[k]);
      }
#endif
      /* then the same signal seen from one grid step below and above the lock: the shift of the
       * demod output between the three views is the discriminator's scale, and with it the deviation */
      if (DC_NEIGHBOUR_VIEWS && !dc_fixed && dc_neighbour_tries < 2u) {
        uint16_t nb = (dc_neighbour_tries == 0u) ? (uint16_t)(dc_lock_q - 1u) : (uint16_t)(dc_lock_q + 1u);
        dc_neighbour_tries++;
        if (nb < DC_QCHANNELS && dc_lock_q >= 1u) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
          app_log("DC neighbour view: ch=%s (%+d kHz)\r\n", dc_chs(nb), (int)((int32_t)dc_khz(nb) - (int32_t)dc_khz(dc_lock_q)));
#endif
          dc_q = nb; dc_enter_phase(1);
          return;
        }
      }
      dc_q = dc_lock_q;
      if (dc_fixed) { dc_enter_phase(0); return; }                   /* fixed channel: packet, burst, soft, packet, ... */
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("DC cycle done on ch=%s: hunt again\r\n", dc_chs(dc_q));
#endif
      dc_enter_hunt();
    }
    return;
  }

  /* ---- packet phase: CD + event counters, frames printed ---- */
  if ((now - dc_rx_since) >= DC_SETTLE_US) {
    int16_t r = dc_rssi();
    if (r != DC_DIAG_NONE) {
      if (r > dsm_cap_peak[dc_q2ch(dc_q)]) dsm_cap_peak[dc_q2ch(dc_q)] = (int8_t)r;
      if (r >= DC_THRESH_DBM) {
        dc_cd_last = now;
        if (!dc_cd) { dc_cd_start = now; dsm_cap_diag[DC_BURSTS]++; dc_cd_set(true); }
      }
    }
  }
  if (dc_cd && (now - dc_cd_last) > DC_CD_HOLD_US) {
    uint32_t len = now - dc_cd_start;
    dc_cd_high_us += dc_cd_since_window(now);
    dc_cd_sum_us += len; dc_cd_n++;
    if ((int32_t)len > dsm_cap_diag[DC_LONGEST]) dsm_cap_diag[DC_LONGEST] = (int32_t)len;
    dc_cd_set(false);
  }
  if (dc_overflow) { dc_rx(dc_q); }
  if (!dc_dual && !dc_sync_locked && !dc_data_mode && (now - dc_sync_sel_us) >= DC_SYNC_ALT_US) {   /* no dual sync: try the variants in turn */
    uint32_t got = dc_ev_sync - dc_sync_slot_sync, gotp = dc_ev_pkt - dc_sync_slot_pkt;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    if (dc_ev_tim) app_log("DC sync slot %s: sync=%lu pkt=%lu (tim=%lu so far)\r\n", dc_sync_name[dc_sync_sel], (unsigned long)got, (unsigned long)gotp, (unsigned long)dc_ev_tim);
#endif
    if (got) dc_sync_locked = true;                                  /* this one works: keep it */
    else {
      sl_rail_idle(dc_rail, SL_RAIL_IDLE, true);
      dc_sync_set((uint8_t)((dc_sync_sel + 1u) % DC_SYNC_VARIANTS));
      dc_rx(dc_q);
    }
  }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (dc_dec_full) {                                                 /* every despread frame, one line */
    static uint32_t last_dec_us; static uint32_t dropped;
    if ((now - last_dec_us) >= 100000u) {                            /* at most 10 lines per second */
      last_dec_us = now;
      uint32_t sum = 384u - 0x10u; for (int i = 0; i < 8; i++) sum += dc_dec[i];
      bool ok1 = (dc_dec[8] == ((sum >> 8) & 0xFFu)) && (dc_dec[9] == (sum & 0xFFu));
      for (int i = 8; i < 14; i++) sum += dc_dec[i];
      bool ok2 = (dc_dec[14] == ((sum >> 8) & 0xFFu)) && (dc_dec[15] == (sum & 0xFFu));
      app_log("DCK ch=%s rssi=%d sync=%u sel=%s order=%s inv=%u q=%u/32 bytes=", dc_chs(dc_q), (int)dc_pkt_rssi, (unsigned)dc_pkt_sync, dc_sync_name[dc_sync_sel], dc_dec_order ? "MSB" : "LSB", (unsigned)dc_dec_inv, (unsigned)dc_dec_q);
      for (int i = 0; i < 16; i++) app_log("%02X%s", dc_dec[i], (i < 15) ? " " : "");
      app_log(" %s%s%s\r\n", ok1 ? "sum1=OK" : "sum1=bad", ok2 ? " sum2=OK" : " sum2=bad", dropped ? " (+more)" : "");
      dropped = 0;
    } else dropped++;
    dc_dec_full = false;
  }
  if (dc_pkt_full && !dc_census && (now - dc_pkt_send_us) >= (dc_data_mode ? DC_DATA_PRINT_US : DC_PKT_SEND_US)) {
    app_log("DCP ch=%s f=%lu rssi=%d sync=%u len=%u%s\r\n", dc_chs(dc_q), (unsigned long)dc_khz(dc_q), (int)dc_pkt_rssi, (unsigned)dc_pkt_sync, (unsigned)dc_pkt_len,
            dc_data_mode ? " data=1" : "");
    dc_send_hex(dc_pkt, dc_pkt_len);
    dc_pkt_full = false; dc_pkt_send_us = dc_now();
  } else if (dc_pkt_full) dc_pkt_full = false;                       /* too soon: drop it, the next one is as good */
#endif
  if (dc_data_mode && dc_census) {                                   /* census: next channel every DC_CENSUS_US */
    if ((now - dc_census_since) >= DC_CENSUS_US) {
      uint8_t c = dc_census_ch;
      if (dc_ev_pre) { dc_census_pre[c] = (uint16_t)(dc_census_pre[c] + dc_ev_pre); dc_census_pkt[c] = (uint16_t)(dc_census_pkt[c] + dc_ev_pkt);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("DC census ch=%u: tim=%lu pre=%lu sync=%lu pkt=%lu\r\n", (unsigned)c, (unsigned long)dc_ev_tim, (unsigned long)dc_ev_pre, (unsigned long)dc_ev_sync, (unsigned long)dc_ev_pkt);
#endif
      }
      dc_ev_tim = dc_ev_timlost = dc_ev_pre = dc_ev_prelost = dc_ev_sync = dc_ev_pkt = dc_ev_abort = dc_ev_ferr = 0;
      dc_pkt_full = false;
      if (c < DC_CENSUS_HI) { dc_census_ch = (uint8_t)(c + 1u); }
      else if (++dc_census_pass < DC_CENSUS_PASSES) { dc_census_ch = DC_CENSUS_LO; }
      else {                                                         /* done: report, then sit on the busiest channel */
        uint8_t best = DC_DATA_CH; uint16_t bp = 0;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("DC census result (channel:preamble detects/frames):");
        for (uint8_t i = DC_CENSUS_LO; i <= DC_CENSUS_HI; i++) if (dc_census_pre[i]) app_log(" %u:%u/%u", (unsigned)i, (unsigned)dc_census_pre[i], (unsigned)dc_census_pkt[i]);
        app_log("\r\n");
#endif
        for (uint8_t i = DC_CENSUS_LO; i <= DC_CENSUS_HI; i++) if (dc_census_pre[i] > bp) { bp = dc_census_pre[i]; best = i; }
        uint16_t list[24]; uint8_t nl = 0;
        for (uint8_t i = DC_CENSUS_LO; i <= DC_CENSUS_HI && nl < 24u; i++) if (dc_census_pre[i] >= 2u) list[nl++] = i;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("DC census: %u channels with 2+ detects:", (unsigned)nl);
        for (uint8_t i = 0; i < nl; i++) app_log(" %u", (unsigned)list[i]);
        app_log("  (computed from the id: 3 5 7 9 15 17 23 25 31 33 35 37 39 47 49 53 55 57 59 61 69 71 73)\r\n");
        app_log("DC data test: staying on ch=%u (%u detects)\r\n", (unsigned)best, (unsigned)bp);
#endif
        dc_census = false;
        dc_census_ch = best;
      }
      sl_rail_idle(dc_rail, SL_RAIL_IDLE, true);
      dc_rx((uint16_t)dc_census_ch * DC_QPC);
      dc_census_since = now; dc_ph_since = now;
    }
    return;
  }
  if (dc_data_mode) {                                                /* data test: stay in packet mode, a summary every 10 s */
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    if (dc_dx_full) {
      static uint32_t last_dx_us;
      if ((now - last_dx_us) >= 200000u) {                           /* 5 lines per second at most */
        last_dx_us = now;
        app_log("DCX ch=%s rssi=%d sop=%u q=%u/64 crc=%s bytes=", dc_chs(dc_q), (int)dc_dx_rssi, (unsigned)dc_dx_sop, (unsigned)dc_dx_q,
                dc_dx_seed == 0u ? "OK(1st)" : dc_dx_seed == 1u ? "OK(2nd)" : "BAD");
        for (int i = 0; i < (int)DC_DATA_SYMS; i++) app_log("%02X%s", dc_dx[i], i < (int)DC_DATA_SYMS - 1 ? " " : "");
        app_log(" |");
        for (int i = 0; i < 7; i++) { uint16_t wv = (uint16_t)((dc_dx[3 + 2 * i] << 8) | dc_dx[4 + 2 * i]); app_log(" ch%u=%u%s", (unsigned)((wv >> 11) & 15u), (unsigned)(wv & 0x7FFu), (wv & 0x8000u) ? "*" : ""); }
        app_log("\r\n");
      }
      dc_dx_full = false;
    }
    {
      static uint32_t last_dt_us;
      if ((now - last_dt_us) >= 1000000u) {                          /* DCT: the second's frames, delta t in us from the previous one */
        last_dt_us = now;
        if (dc_dt_n) {
          app_log("DCT ch=%s n=%u t0=%lu:", dc_chs(dc_q), (unsigned)dc_dt_n, (unsigned long)dc_dt[0].t);
          for (uint8_t i = 0; i < dc_dt_n; i++) {
            uint32_t d = dc_dt_have ? (dc_dt[i].t - dc_dt_last) : 0u;
            if (dc_dt[i].pos) app_log(" %lu/%c%c/%d", (unsigned long)d, dc_dt[i].kind, dc_dt[i].pos, (int)dc_dt[i].rssi);
            else              app_log(" %lu/%c/%d", (unsigned long)d, dc_dt[i].kind, (int)dc_dt[i].rssi);
            dc_dt_last = dc_dt[i].t; dc_dt_have = true;
          }
          if (dc_dt_drop) app_log(" +%u more", (unsigned)dc_dt_drop);
          app_log("\r\n");
        }
        dc_dt_n = 0; dc_dt_drop = 0;
      }
    }
#endif
    if ((now - dc_ph_since) >= 10000000u) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("DC data test ch=%s: tim=%lu/%lu pre=%lu/%lu sync=%lu pkt=%lu abort=%lu ferr=%lu in 10 s; decoded %lu, crc ok %lu, no sop %lu\r\n", dc_chs(dc_q),
              (unsigned long)dc_ev_tim, (unsigned long)dc_ev_timlost, (unsigned long)dc_ev_pre, (unsigned long)dc_ev_prelost,
              (unsigned long)dc_ev_sync, (unsigned long)dc_ev_pkt, (unsigned long)dc_ev_abort, (unsigned long)dc_ev_ferr,
              (unsigned long)dc_dx_n, (unsigned long)dc_dx_crc_ok, (unsigned long)dc_dx_nosop);
      dc_dx_n = 0; dc_dx_crc_ok = 0; dc_dx_nosop = 0;
#endif
      dc_ev_tim = dc_ev_timlost = dc_ev_pre = dc_ev_prelost = dc_ev_sync = dc_ev_pkt = dc_ev_abort = dc_ev_ferr = 0;
      dc_ph_since = now;
    }
    return;
  }
  if ((now - dc_ph_since) >= DC_PKT_US) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("DC packet phase: tim=%lu/%lu pre=%lu/%lu sync=%lu pkt=%lu abort=%lu ferr=%lu despread=%lu ok=%lu\r\n",
            (unsigned long)dc_ev_tim, (unsigned long)dc_ev_timlost, (unsigned long)dc_ev_pre, (unsigned long)dc_ev_prelost,
            (unsigned long)dc_ev_sync, (unsigned long)dc_ev_pkt, (unsigned long)dc_ev_abort, (unsigned long)dc_ev_ferr,
            (unsigned long)dc_dec_n, (unsigned long)dc_dec_ok);
    if (!dc_dual) app_log("DC packet phase: sync0 was %s%s\r\n", dc_sync_name[dc_sync_sel], dc_sync_locked ? " (it produced sync detects)" : " at the end, none produced sync detects");
#endif
    dc_enter_phase(2);                                               /* then the burst capture */
    return;
  }
  if (!dc_fixed) {
    if ((now - dc_cd_last) > DC_SILENCE_US) {
      dsm_cap_diag[DC_RELOCKS]++;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("DC silence on ch=%s\r\n", dc_chs(dc_q));
#endif
      dc_enter_hunt();
      return;
    }
    if (bind_held) {
      if (!dc_bind_seen) { dc_bind_seen = true; dc_bind_since = now; }
      else if ((now - dc_bind_since) > DC_BIND_US) {
        dc_bind_seen = false;
        dsm_cap_diag[DC_RELOCKS]++;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("DC BIND: re-hunt\r\n");
#endif
        dc_enter_hunt();
        return;
      }
    } else dc_bind_seen = false;
  }
}

void dsm_cap_service(bool bind_held)
{
  uint32_t now = dc_now();

  switch (dc_state) {
  case 1: {                                                          /* hunt */
    if ((now - dc_rx_since) < DC_SETTLE_US) break;
    int16_t r = dc_rssi();
    if (r != DC_DIAG_NONE) {
      if (r > dc_max) dc_max = r;
      dc_h_reads++; if (r >= DC_THRESH_DBM) { dc_h_hits++; dc_run_hit(now); }
      if (r >= DC_STRONG_DBM) dc_strong_seen = true;                /* decided at the end of the sweep */
    }
    if ((now - dc_rx_since) >= DC_DWELL_US) {
      uint8_t c = dc_hunt_ch;                                        /* hunt tables are per MHz channel */
      dsm_cap_scan[c] = (int8_t)dc_max;
      dc_duty[c] = (uint8_t)(dc_h_reads ? ((uint32_t)dc_h_hits * 100u / dc_h_reads) : 0u);
      dc_run[c] = (uint16_t)(dc_h_run_max > 65535u ? 65535u : dc_h_run_max);
      if (dc_max != DC_DIAG_NONE && dc_max > dsm_cap_peak[c]) dsm_cap_peak[c] = (int8_t)dc_max;
      dc_h_reads = 0; dc_h_hits = 0; dc_run_reset();
      if (++dc_hunt_ch >= DC_CHANNELS) dc_sweep_done(); else dc_rx((uint16_t)dc_hunt_ch * DC_QPC);
    }
    break;
  }
  case 2: {                                                          /* confirm */
    if ((now - dc_rx_since) >= DC_SETTLE_US) {
      int16_t r = dc_rssi();
      if (r != DC_DIAG_NONE) {
        if (r > dc_max) dc_max = r;
        dc_h_reads++; if (r >= DC_THRESH_DBM) { dc_h_hits++; dc_run_hit(now); }
      }
    }
    if ((now - dc_phase_since) >= DC_CONFIRM_US) {
      uint32_t duty = dc_h_reads ? ((uint32_t)dc_h_hits * 100u / dc_h_reads) : 0u;
      bool ok = (duty >= DC_CONFIRM_DUTY) && (dc_rssi_hunt || dc_h_run_max >= DC_LONG_RUN_US);
      if (ok) { dc_enter_lock(dc_q, dc_max); break; }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("DC confirm failed ch=%s duty=%lu%% max=%d longest=%lu us\r\n", dc_chs(dc_q), (unsigned long)duty, (int)dc_max, (unsigned long)dc_h_run_max);
#endif
      if (++dc_cand_i < dc_cand_n) {                                 /* try the next candidate */
        dc_phase_since = now;
        dc_h_reads = 0; dc_h_hits = 0; dc_run_reset();
        dc_rx((uint16_t)dc_cand[dc_cand_i] * DC_QPC);
        break;
      }
      dc_enter_hunt();
    }
    break;
  }
  case 3: dc_lock_service(now, bind_held); break;
  case 4: {                                                          /* peak search around a strong reading (coarse 1 MHz, then fine steps) */
    if (!dc_pk_ready && (now - dc_rx_since) >= DC_SETTLE_US) {        /* one RSSI read while each snapshot fills: its tag */
      int16_t r = dc_rssi();
      if (r != DC_DIAG_NONE) { dc_pk_ready = true; dc_pk_tag = r; if (r > dsm_cap_peak[dc_q2ch(dc_q)]) dsm_cap_peak[dc_q2ch(dc_q)] = (int8_t)r; }
    }
    if (dc_overflow) {                                               /* a snapshot froze: score it */
      uint16_t got = 0, sat = 0, tr = 0;
      uint16_t avail = sl_rail_get_rx_fifo_bytes_available(dc_rail);
      if (avail > DC_FIFO_BYTES) avail = DC_FIFO_BYTES;
      static uint8_t tmp[DC_FIFO_BYTES];
      while (got < avail) { uint16_t n = sl_rail_read_rx_fifo(dc_rail, &tmp[got], (uint16_t)(avail - got)); if (!n) break; got = (uint16_t)(got + n); }
      if (got >= 1024u) {
        uint16_t sc = dc_snap_score(tmp, got, &sat, &tr);
        if (sc > dc_pk_score) dc_pk_score = sc;
        if (dc_pk_ready && dc_pk_tag > dc_pk_rssi) {                 /* the snapshot taken with the strongest RSSI represents this step */
          dc_pk_rssi = dc_pk_tag; dc_pk_mid = dc_snap_mid(tmp, got); dc_pk_sat = (uint16_t)((uint32_t)sat * 100u / got);
          if (dc_pk_tag > dc_pk_snap_rssi) {                         /* and the strongest of the whole search is kept for printing */
            memcpy(dc_pk_snap, tmp, got); dc_pk_snap_len = got; dc_pk_snap_q = dc_q; dc_pk_snap_rssi = dc_pk_tag;
            dc_pk_snap_mid = dc_pk_mid; dc_pk_snap_sat = dc_pk_sat; dc_pk_snap_tr = tr; dc_pk_snap_full = true;
          }
        }
      }
      dc_rx(dc_q); dc_pk_ready = false;
    } else if ((now - dc_rx_since) > 20000u) { dc_rx(dc_q); dc_pk_ready = false; }
    if ((now - dc_pk_snap_t0) >= DC_PEAK_DWELL_US) {
      uint16_t i = dc_pk_i;
      dc_pk_r[i] = dc_pk_rssi; dc_pk_m[i] = dc_pk_mid;
      if (dc_pk_stage == 1u) { dc_pk_cq[i] = dc_q; dc_pk_cr[i] = dc_pk_rssi; dc_pk_cm[i] = dc_pk_mid; }
      else for (uint16_t k = 0; k < DC_PEAK_COARSE; k++)             /* a bursty signal may have been missed now but caught in the coarse dwell */
        if (dc_pk_cq[k] == dc_q && dc_pk_cr[k] > dc_pk_r[i]) { dc_pk_r[i] = dc_pk_cr[k]; dc_pk_m[i] = dc_pk_cm[k]; }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("DC peak ch=%s: %d dBm, mid %+d, sat %u%%, modulation %u%%%s\r\n", dc_chs(dc_q), (int)dc_pk_rssi, (int)dc_pk_mid, (unsigned)dc_pk_sat, (unsigned)dc_pk_score,
              (dc_pk_r[i] != dc_pk_rssi) ? " (coarse reading kept)" : "");
#endif
      if (i + 1u < dc_pk_n) { dc_pk_i++; dc_pk_step_reset(); dc_rx((uint16_t)(dc_pk_first + dc_pk_i * dc_pk_step)); dc_pk_snap_t0 = now; break; }
      int16_t best_r = DC_DIAG_NONE; uint16_t best_i = 0;
      for (uint16_t k = 0; k < dc_pk_n; k++) if (dc_pk_r[k] > best_r) { best_r = dc_pk_r[k]; best_i = k; }
      uint16_t bq = (uint16_t)(dc_pk_first + best_i * dc_pk_step);
      if (best_r < DC_STRONG_DBM) {                                  /* gone already: a hopping TX; it will be back on c+1 (or c) */
        uint16_t guess = (uint16_t)((dc_pk_c + 1u) * DC_QPC);
        if (guess >= DC_QCHANNELS) guess = (uint16_t)(dc_pk_c * DC_QPC);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("DC peak gone (hopping?): lock ch=%s anyway\r\n", dc_chs(guess));
#endif
        dc_enter_lock(guess, best_r);
        break;
      }
      if (dc_pk_stage == 1u && dc_pk_m[best_i] >= -25 && dc_pk_m[best_i] <= 25) {   /* centred already: no fine stage, every 100 ms counts */
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("DC peak coarse: ch=%s %d dBm, mid %+d: centred, lock\r\n", dc_chs(bq), (int)best_r, (int)dc_pk_m[best_i]);
#endif
        dc_pk_snap_pending = dc_pk_snap_full && dc_pk_snap_rssi >= DC_THRESH_DBM;
        dc_enter_lock(bq, best_r);
        break;
      }
      if (dc_pk_stage == 1u && (dc_pk_m[best_i] >= 100 || dc_pk_m[best_i] <= -100)) {
        /* v3.24: pinned = the carrier is a whole MHz away in the direction of the sign (the Lemon
         * sits on the MHz grid; run 23 had ch 61 pinned +127 and ch 63 pinned -128 while the 80 ms
         * dwell on 62 missed every burst, and the fine stage then locked 500 kHz off) */
        uint16_t lq = (dc_pk_m[best_i] > 0) ? (uint16_t)(bq + DC_QPC) : (uint16_t)(bq - DC_QPC);
        if (lq >= DC_QCHANNELS) lq = bq;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("DC peak coarse: ch=%s %d dBm, mid %+d: pinned, the carrier is on ch=%s: lock\r\n", dc_chs(bq), (int)best_r, (int)dc_pk_m[best_i], dc_chs(lq));
#endif
        dc_pk_snap_pending = dc_pk_snap_full && dc_pk_snap_rssi >= DC_THRESH_DBM;
        dc_enter_lock(lq, best_r);
        break;
      }
      if (dc_pk_stage == 1u) {                                       /* coarse done: fine steps +-500 kHz around the strongest */
        uint16_t half = (uint16_t)(DC_PEAK_FINE / 2u);
        uint16_t first = (bq >= half) ? (uint16_t)(bq - half) : 0u;
        if (first + DC_PEAK_FINE > DC_QCHANNELS) first = (uint16_t)(DC_QCHANNELS - DC_PEAK_FINE);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("DC peak coarse: ch=%s %d dBm\r\n", dc_chs(bq), (int)best_r);
#endif
        dc_pk_begin(2u, first, 1u, DC_PEAK_FINE);
        break;
      }
      /* fine done.  The carrier: among the steps within DC_PEAK_WITHIN dB of the strongest, the one
       * whose demod output is centred (|mid| smallest); RSSI alone cannot tell 125 kHz steps apart */
      uint16_t win = best_i; int16_t win_mid = 127;
      for (uint16_t k = 0; k < dc_pk_n; k++) {
        if (dc_pk_r[k] == DC_DIAG_NONE || dc_pk_r[k] < best_r - DC_PEAK_WITHIN) continue;
        int16_t am = (dc_pk_m[k] < 0) ? (int16_t)-dc_pk_m[k] : dc_pk_m[k];
        if (am < win_mid || (am == win_mid && dc_pk_r[k] > dc_pk_r[win])) { win = k; win_mid = am; }
      }
      uint16_t wq = (uint16_t)(dc_pk_first + win * dc_pk_step);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("DC peak: ch=%s (%lu kHz) %d dBm, mid %+d; strongest ch=%s %d dBm\r\n", dc_chs(wq), (unsigned long)dc_khz(wq), (int)dc_pk_r[win], (int)dc_pk_m[win],
              dc_chs(bq), (int)best_r);
#endif
      dc_pk_snap_pending = dc_pk_snap_full && dc_pk_snap_rssi >= DC_THRESH_DBM;   /* printed after the burst phase */
      dc_enter_lock(wq, dc_pk_r[win]);
    }
    break;
  }
  default: break;
  }

  if (dc_state == 3 && (now - dc_uart_us) >= DC_UART_US && (dc_rate_done || !dc_soft_mode)) {
    dc_uart_us = now;
    uint32_t win = now - dc_cd_win_since;
    uint32_t high = dc_cd_high_us + (dc_cd ? dc_cd_since_window(now) : 0u);
    uint32_t duty = (win > 0u) ? (high * 100u / win) : 0u;
    if (duty > 100u) duty = 100u;
    dsm_cap_diag[DC_DUTY] = (int32_t)duty;
    dc_cd_win_since = now; dc_cd_high_us = 0;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    if (dc_ph == 2u || dc_census) { }                                /* burst phase / census: no status line */
    else if (dc_soft_mode)
      app_log("DC S ch=%s r=%ld b=%ld snaps=%lu strongest=%d\r\n", dc_chs(dc_q), (long)dsm_cap_diag[DC_RSSI],
              (long)dsm_cap_diag[DC_BURSTS], (unsigned long)dc_snaps, (int)dc_snap_maxrssi);
    else
      app_log("DC P ch=%s r=%ld d=%lu%% b=%ld n=%lu avg=%lu max=%ld tim=%lu pre=%lu sync=%lu pkt=%lu\r\n", dc_chs(dc_q), (long)dsm_cap_diag[DC_RSSI],
              (unsigned long)duty, (long)dsm_cap_diag[DC_BURSTS], (unsigned long)dc_cd_n, (unsigned long)(dc_cd_n ? dc_cd_sum_us / dc_cd_n : 0u),
              (long)dsm_cap_diag[DC_LONGEST], (unsigned long)dc_ev_tim, (unsigned long)dc_ev_pre, (unsigned long)dc_ev_sync, (unsigned long)dc_ev_pkt);
    dc_cd_sum_us = 0; dc_cd_n = 0;
#endif
  }
}
