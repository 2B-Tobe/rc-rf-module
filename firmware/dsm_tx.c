/* dsm_tx.c — see dsm_tx.h.  #included into app_process.c.   v1.14: EU lock, DTX_EU_LOCK 1 (default) = 22 ms only:
 *   BIND + RANGE binds at 22 ms too, a stored 11 ms bind runs at 22 ms (the receiver needs one rebind), the log
 *   says so; DTX_EU_LOCK 0 = as v1.13.
 *   v1.13 (frame rate chosen at bind: BIND + RANGE at power-up
 *   = 11 ms / 8 channels, BIND alone = 22 ms / 7 channels; stored in flash slot 4 and used at every later power-up;
 *   flight power follows the rate: 18 dBm at 22 ms, 15 dBm at 11 ms (EU 10 % rule); the bind burst always at 0 dBm.
 *   The DTX_TEST_6CH_REDUNDANT experiment (reverted in v1.2) is removed.
 *   v1.12: RANGE follows the input all the time, as in nRF24
 *   and FrSky: switching it while flying drops to -10 dBm and back; it used to be read only at power-up.
 *   v1.11: flight power 18 dBm, which keeps DSMX at 22 ms
 *   under the EU 10 % medium-utilisation limit (EN 300 328) with a 0 dBi antenna; the module could do ~+20 dBm.
 *   The bind burst stays at 0 dBm as tested with every receiver; RANGE -10 dBm.
 *   v1.10: fix: the TX could freeze during the bind burst --
 *   a frame heard between our bind packets was copied with a clamped length that RAIL's copy does not honour, and
 *   overwrote RAM; see dsm_tx_rx_process. Seen as: Lemon RX bound (LED off) but no data, TX LEDs stopped, TX log
 *   ends after "DT bind:". v1.9 (answer wait 0.5 s) is withdrawn: the wait was not the cause; back to 4.2 s.
 *   v1.8: DTX_LEMON_DATA: data packets with the same short
 *   60-chip preamble as the v1.7 bind packets -- the Lemon TX's data SOP sits 31..35 chips after the receiver's sync
 *   point, ours ~103, and the Lemon RX took our bind only after the bind preamble was shortened the same way;
 *   v1.7: DTX_LEMON_BIND: bind burst like the Lemon TX, 10.545 ms / 60-chip preamble;
 *   v1.6: DTX_MODE_22MS: DSMX 22 ms 7ch type A2 as a DX6i, default ON;
 *   v1.5: bind burst 1.0 s instead of 1.8 s, for the Lemon RX;
 *   v1.4: answer wait syncs on the answer's first symbol, not on the preamble,
 *   + RSSI energy log on the bind channel; v1.3: bind answer logged in full, accepted as Multi does -- id must match,
 *   rejected only when both checksum bytes differ; v1.2: carrier off ~24 chips after the last symbol, as the Lemon; transmits without PPM too)
 *
 * Packets (chip i = bit i of the byte stream RAIL transmits after its preamble and the 0x99999999
 * "sync word", which is more preamble):
 *   bind  16 bytes -> 128 SDR symbols: pn[0][8] for a 1, inverted for a 0, LSB first; then ~24 chips of
 *         unmodulated carrier and off (measured on the Lemon: 2 chips + ~25 chips of tone, then off)
 *   data  SOP pn[row][sop_col] | 4 chips 0101 | ~SOP | 19 8DR symbols for 10 id2 id3 <7 words> <crc lo hi>
 *         symbol for byte v = pn[row][data_col + (v>>6 & 1)] rotated right by (v-1)&63, inverted if bit 7 clear
 *   row = (dsmx-2)%5, sop_col = (id0+id1+id2+2)&7, data_col = 7-sop_col; CRC-16 (0xA001 reflected,
 *   init = bit-reversed seed, xor FFFF) over the 17 bytes, seed ~(id0<<8|id1) on the first packet of a
 *   pair, id0<<8|id1 on the second.
 * Timing (Multi DSM_callback): frame m at t = 11 ms x m: the frame's packet on list[2m % 23], the same
 *   packet again 4.01 ms later on list[(2m+1) % 23]; lower channels on even frames, upper on odd.
 *   Launches are RAIL scheduled transmits at absolute times. */
#include "dsm_tx.h"
#include "rx_store.h"
#include "rail_config.h"
#include "sl_gpio.h"
#include "em_device.h"
#include <string.h>
#if defined(SL_CATALOG_APP_LOG_PRESENT)
#include "app_log.h"
#endif

extern const RAIL_ChannelConfig_t dsm_raw_channelConfig;

#define DTX_HOPS            23u
#define DTX_FRAME_US     11000u    /* tried 11023 (measured Lemon TX cadence): no change on the AR610. Reverted. */
#define DTX_PAIR_US       4010u
#define DTX_SYNC_PRE 0x99999999ul
#define DTX_BIND_RF         14u      /* Multi binds on CYRF channel 13 = 2414 MHz                     */
#ifndef DTX_BIND_COUNT
#define DTX_BIND_COUNT     100u      /* v1.5: 1.0 s of bind packets, 10 ms apart (was 180 = 1.8 s, Multi's value).
                                        The Lemon RX accepted the 1.8 s burst (slow blink) but never answered and
                                        dropped back to bind; Multi notes the Lemon RX waits for the channel to go
                                        quiet after the first good bind packet and gives up if the burst runs too
                                        long, and the Lemon's own TX binds for ~1 s. AR610 / R620X to be re-checked. */
#endif
/* v1.7: bind burst shaped like the Lemon TX's, as measured by dsm_rx v1.7's bind profiler (same receiver for both):
 *   Lemon TX: bind packets 10.545 ms apart, first symbol ~33 chips (32..36) after the receiver's sync point;
 *   ours was: 10.000 ms apart, ~103 chips after the sync point (PHY preamble 128 + 32 sync chips).
 * DTX_LEMON_BIND 1: bind packets 10.545 ms apart with a 60-chip preamble (RAIL alternate preamble length, bind
 * packets only; 60 = 15 x the 4-chip 1001 pattern, so it still runs into the sync word). Data packets unchanged. */
#ifndef DTX_LEMON_BIND
#define DTX_LEMON_BIND       1
#endif
/* v1.8: DTX_LEMON_DATA 1: data packets use the same short preamble. Result of v1.7 on the Lemon RX: fast blink ->
 * slow -> LED off (bind taken), but no link afterwards, LED stays off. The DSM capture of the Lemon TX's own data
 * shows its SOP 31..35 chips after the sync point (bind: 32..36); our data still had the full 128-chip preamble,
 * SOP ~103 chips after the sync point -- the same difference the Lemon RX rejected in the bind packets. */
#ifndef DTX_LEMON_DATA
#define DTX_LEMON_DATA       1
#endif
#define DTX_SHORT_PREAMBLE   60u     /* chips; the PHY's own preamble is 128 (60 = 15 x the 4-chip 1001 pattern) */
#if DTX_LEMON_BIND
#define DTX_BIND_EVERY_US 10545u
#define DTX_BIND_PREAMBLE_SHOWN DTX_SHORT_PREAMBLE
#else
#define DTX_BIND_EVERY_US 10000u
#define DTX_BIND_PREAMBLE_SHOWN 128u
#endif
#if DTX_LEMON_DATA
#define DTX_DATA_PREAMBLE_SHOWN DTX_SHORT_PREAMBLE
#else
#define DTX_DATA_PREAMBLE_SHOWN 128u
#endif
#define DTX_ANSWER_WAIT_US 4200000u  /* then listen for the receiver's answer this long               */
#define DTX_BIND_LEN      1027u      /* 128 symbols + 24 chips of carrier, then off (the Lemon: ~25 chips) */
#define DTX_ANSWER_LEN     700u      /* RX frame for the 80-symbol answer                             */
#define DTX_DATA_SYMS       19u
#define DTX_DATA_LEN       172u      /* 132 + 19 x 64 = 1348 chips -> 169 bytes, + ~28 chips of carrier */
/* v1.13: the frame rate is chosen at bind time and remembered by the transmitter (flash slot 4):
 *   BIND held at power-up            -> bind at 22 ms: type A2 (DSMX 2048 22 ms, as a DX6i), 7 channels, every
 *                                       packet carries all 7, one pair (2nd packet 4.01 ms after the 1st) per 22 ms
 *   BIND + RANGE held at power-up    -> bind at 11 ms: type B2 (DSMX 2048 11 ms), 8 channels split over a
 *                                       lower and an upper frame, one pair per 11 ms
 *   no BIND                          -> the rate of the last bind; DTX_MODE_22MS when none is stored yet.
 * The receiver learns the rate from the bind type, so the transmitter must keep sending the rate it bound with.
 * PPM channel 8 only goes out at 11 ms: with 8+ channels DSMX needs the 11 ms split frames. */
#ifndef DTX_MODE_22MS
#define DTX_MODE_22MS        1       /* default rate before any bind is stored: 1 = 22 ms, 0 = 11 ms  */
#endif
#ifndef DTX_EU_LOCK
#define DTX_EU_LOCK          1       /* v1.14: 1 = EU build, 22 ms only; 0 = 11 ms allowed (outside the EU) */
#endif
#define DTX_PERIOD_22_US 22000u
#define DTX_TX_FIFO_BYTES 2048u
#define DTX_RX_FIFO_BYTES 2048u
#define DTX_MIN_SLACK_US    150u     /* a launch nearer than this goes out now, counted as late       */
#define DTX_TX_POWER_DDBM   180      /* deci-dBm, flight at 22 ms: 18 dBm. On air ~13 %:
                                        63 mW x 13 % = 8.4 % < the EU 10 % limit. 200 = module maximum.   */
#define DTX_TX_POWER_11MS_DDBM 150   /* flight at 11 ms: 15 dBm. On air ~27 %: 32 mW x 27 % = 8.5 % < 10 % */
#define DTX_BIND_POWER_DDBM   0      /* bind burst: 0 dBm, as tested with every receiver              */
#define DTX_RANGE_POWER_DDBM -100
#define DTX_PPM_LOSS_US   100000u
#ifndef DTX_UART_STATUS
#define DTX_UART_STATUS       1    /* diagnostic: prints cal (calibrations run) so we can see if it lines up with the glitch */
#endif
extern volatile uint32_t cal_count;   /* app_process.c: calibrations actually run since boot */

/* DSM channel id 0..7 (T A E R Gear Aux1 Aux2 Aux3) <- input channel index 0..7 (PPM or encoder frames).
 * Tobe's encoder sends TAER (channel 1 = throttle, 2 = aileron, 3 = elevator, 4 = rudder), Spektrum's own order,
 * so the map is straight through: THRO AILE ELEV RUDD GEAR AUX1 AUX2 on the receiver = encoder channels 1..7.
 * A 4-in-1 style AETR input would need { 2, 0, 1, 3, 4, 5, 6, 7 }. */
static const uint8_t DTX_IN_MAP[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
/* 22 ms: Multi's 7-channel DX6i map, every packet carries all 7 (both packets of the pair: redundancy).
 * 11 ms: Multi's 8-channel map (DX7 dump), lower frame ids and upper frame ids; 0xFF = empty slot.
 *   8 channels only fit in one 11 ms frame by splitting: aileron/elevator/aux1 repeat in both,
 *   rudder/aux2/aux3 only in the lower frame, gear/throttle only in the upper. */
static const uint8_t DTX_MAP_22[7]    = { 1, 5, 2, 4, 3, 6, 0 };
static const uint8_t DTX_MAP_LOWER[7] = { 1, 5, 2, 3, 6, 7, 0xFF };
static const uint8_t DTX_MAP_UPPER[7] = { 1, 5, 2, 4, 0, 0xFF, 0xFF };
static const uint8_t dtx_answer_code[8] = { 0x98, 0x88, 0x1B, 0xE4, 0x30, 0x79, 0x03, 0x84 };

static const uint8_t dtx_pn[5][9][8] = {
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

volatile int32_t dsm_tx_diag[DTX_DIAG_N];
static sl_rail_handle_t t_rail;
static const volatile uint16_t *t_in;
static SL_RAIL_DECLARE_FIFO_BUFFER(t_txfifo, DTX_TX_FIFO_BYTES);
static SL_RAIL_DECLARE_FIFO_BUFFER(t_rxfifo, DTX_RX_FIFO_BYTES);
static uint8_t  t_id[4], t_list[DTX_HOPS], t_sop_col, t_data_col;
static uint16_t t_seed[2];
static uint8_t  t_state;                 /* 0 bind, 1 answer wait, 2 data */
static uint8_t  t_pkt[DTX_BIND_LEN];     /* the chip stream being sent */
static uint16_t t_len;                   /* fixed length in effect */
static uint32_t t_hop;                   /* hop counter: list[t_hop % 23], frame = t_hop / 2 */
static uint32_t t_next;                  /* launch time of the next packet */
static uint32_t t_bind_sent, t_answer_since, t_uart_us, t_tx_count, t_tx_last;
static uint32_t t_ans_frames, t_ans_logged, t_early_frames;   /* v1.3 diag: frames heard while waiting / during the burst */
#define DTX_ANSWER_LOG_MAX 12u   /* answer-wait frames printed in full (the rest are only counted) */
/* v1.4: while waiting for the answer the receiver syncs on the answer's own first symbol instead of
 * on 32 more chips of preamble: Multi's CYRF sends only a short preamble (0x333304), so a CYRF
 * receiver's answer may never give our preamble sync the ~64 chips it needs. Restored before data. */
static bool     t_ans_codesync;
static uint32_t t_ans_sync;
/* v1.4 energy log on the bind channel during the answer wait (independent of any packet sync) */
#define DTX_EN_DBM       (-75)   /* a burst = RSSI at or above this                                */
#define DTX_EN_MAX         24u
static uint32_t t_en_t[DTX_EN_MAX], t_en_len[DTX_EN_MAX];
static int8_t   t_en_pk[DTX_EN_MAX];
static uint32_t t_en_n, t_en_start, t_en_last;
static int16_t  t_en_peak, t_en_min, t_en_max;
static bool     t_en_on;
static bool     t_ppm, t_busy, t_range;
static bool     t_pwr_dirty;             /* v1.12: RANGE changed, apply the power before the next data packet */
static bool     t_mode22;                /* v1.13: true = 22 ms (A2, 7 ch), false = 11 ms (B2, 8 ch) */
#ifndef LOG_TXT
#define LOG_TXT(s) s                          /* a release build (app_process.c) defines it empty: no log texts */
#endif
static const char *t_mode_from = LOG_TXT("default");
static inline uint8_t  t_num_ch(void)  { return t_mode22 ? 7u : 8u; }
static inline uint8_t  t_type(void)    { return t_mode22 ? 0xA2u : 0xB2u; }
static inline uint32_t t_period(void)  { return t_mode22 ? DTX_PERIOD_22_US : DTX_FRAME_US; }
static inline int16_t  t_flight_ddbm(void) { return t_mode22 ? (int16_t)DTX_TX_POWER_DDBM : (int16_t)DTX_TX_POWER_11MS_DDBM; }
static volatile bool t_done, t_txerr;
static uint64_t t_w[DTX_ANSWER_LEN / 8u + 2u];

static sl_gpio_t t_led_a = { .port = SL_GPIO_PORT_C, .pin = 4 };
static sl_gpio_t t_led_b = { .port = SL_GPIO_PORT_C, .pin = 5 };

static inline uint32_t t_now(void) { return (uint32_t)sl_rail_get_time(t_rail); }
static inline unsigned t_pop32(uint32_t x)
{
  x = x - ((x >> 1) & 0x55555555u); x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
  x = (x + (x >> 4)) & 0x0F0F0F0Fu; return (x * 0x01010101u) >> 24;
}
static inline unsigned t_pop64(uint64_t x) { return t_pop32((uint32_t)x) + t_pop32((uint32_t)(x >> 32)); }
static inline uint64_t t_rotr64(uint64_t w, unsigned n) { n &= 63u; return n ? ((w >> n) | (w << (64u - n))) : w; }
static uint64_t t_code64(const uint8_t *c) { uint64_t w = 0; for (int i = 0; i < 8; i++) w |= (uint64_t)c[i] << (8 * i); return w; }
static inline uint64_t t_get64(uint32_t p) { uint32_t i = p >> 6, b = p & 63u; return b ? ((t_w[i] >> b) | (t_w[i + 1] << (64u - b))) : t_w[i]; }

/* ---- chip writer ---- */
static uint32_t t_nb;                    /* chips written into t_pkt */
static inline void t_put(unsigned chip) { if (chip) t_pkt[t_nb >> 3] |= (uint8_t)(1u << (t_nb & 7u)); t_nb++; }
static void t_put64(uint64_t w) { for (int i = 0; i < 64; i++) t_put((unsigned)((w >> i) & 1u)); }
static void t_begin(void) { memset(t_pkt, 0, sizeof(t_pkt)); t_nb = 0; }

/* ---- id and what follows ---- */
static void t_calc_channels(void)        /* Multi-module DSM_calc_dsmx_channel */
{
  uint32_t id = ~(((uint32_t)t_id[0] << 24) | ((uint32_t)t_id[1] << 16) | ((uint32_t)t_id[2] << 8) | t_id[3]);
  uint32_t t = id; uint8_t idx = 0;
  while (idx < DTX_HOPS) {
    t = t * 0x0019660Du + 0x3C6EF35Fu;
    uint8_t next = (uint8_t)(((t >> 8) % 0x49u) + 3u);
    if ((next ^ t_id[3]) & 1u) continue;
    uint8_t c1 = 0, c2 = 0, c3 = 0, i;
    for (i = 0; i < idx; i++) {
      if (t_list[i] == next) break;
      if (t_list[i] <= 27u) c1++; else if (t_list[i] <= 51u) c2++; else c3++;
    }
    if (i != idx) continue;
    if ((next < 28u && c1 < 8u) || (next >= 28u && next < 52u && c2 < 7u) || (next >= 52u && c3 < 8u)) t_list[idx++] = next;
  }
}
static void t_gen_id(void)               /* fixed per module: from the chip's EUI64 */
{
  uint64_t u = ((uint64_t)DEVINFO->EUI64H << 32) | DEVINFO->EUI64L;
  t_id[0] = (uint8_t)(u >> 0); t_id[1] = (uint8_t)(u >> 8); t_id[2] = (uint8_t)(u >> 16);
  t_id[3] = (uint8_t)((u >> 24) ^ (u >> 32) ^ (u >> 40));
  t_id[0] ^= rc_model;                        /* per-model binding: changes the CRC seed and the channel list; 0 = unchanged */
  t_id[1] ^= (uint8_t)(rc_model * 37u);
  t_sop_col = (uint8_t)((t_id[0] + t_id[1] + t_id[2] + 2u) & 7u);
  if (t_sop_col == 0u) { t_id[2] ^= 1u; t_sop_col = (uint8_t)((t_id[0] + t_id[1] + t_id[2] + 2u) & 7u); }   /* Multi: never col 0 (OrangeRX bug) */
  t_data_col = (uint8_t)(7u - t_sop_col);
  t_seed[1] = (uint16_t)(((uint16_t)t_id[0] << 8) | t_id[1]);
  t_seed[0] = (uint16_t)~t_seed[1];
  t_calc_channels();
  dsm_tx_diag[DT_ID] = (int32_t)(((uint32_t)t_id[0] << 24) | ((uint32_t)t_id[1] << 16) | ((uint32_t)t_id[2] << 8) | t_id[3]);
}
static inline uint8_t t_row_of(uint32_t hop) { return (uint8_t)((t_list[hop % DTX_HOPS] + 5u - 2u) % 5u); }
static inline uint8_t t_rf_of(uint32_t hop)  { return (uint8_t)(t_list[hop % DTX_HOPS] + 1u); }

/* ---- packets ---- */
static uint16_t t_crc16(const uint8_t *d, uint32_t n, uint16_t seed)
{
  uint16_t crc = 0; for (int i = 0; i < 16; i++) if (seed & (1u << i)) crc |= (uint16_t)(0x8000u >> i);
  for (uint32_t k = 0; k < n; k++) { crc ^= d[k]; for (int i = 0; i < 8; i++) crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1); }
  return (uint16_t)(crc ^ 0xFFFFu);
}
static void t_build_bind(void)
{
  uint8_t pk[16]; uint16_t sum = 384u - 0x10u;
  for (int i = 0; i < 4; i++) pk[i] = pk[4 + i] = (uint8_t)~t_id[i];
  for (int i = 0; i < 8; i++) sum += pk[i];
  pk[8] = (uint8_t)(sum >> 8); pk[9] = (uint8_t)sum;
  pk[10] = 0x01; pk[11] = t_num_ch(); pk[12] = t_type(); pk[13] = 0x00;
  for (int i = 8; i < 14; i++) sum += pk[i];
  pk[14] = (uint8_t)(sum >> 8); pk[15] = (uint8_t)sum;
  uint64_t code = t_code64(dtx_pn[0][8]);
  t_begin();
  for (uint32_t k = 0; k < 128u; k++) t_put64(((pk[k >> 3] >> (k & 7u)) & 1u) ? code : ~code);
}
static inline uint16_t t_val(uint16_t us) { int32_t v = 1024 + ((int32_t)us - 1500) * 2046 / 1200; if (v < 0) v = 0; if (v > 2047) v = 2047; return (uint16_t)v; }
static uint16_t t_input(uint8_t ppm_idx)   /* the PPM channel, or the no-PPM position (centre, throttle 1000 us) */
{
  if (t_ppm) return t_in[ppm_idx];
  return (ppm_idx == DTX_IN_MAP[0]) ? 1000u : 1500u;
}
static void t_build_data(uint32_t hop)   /* the packet for hop: frame parity picks lower/upper, hop parity the seed */
{
  uint8_t by[DTX_DATA_SYMS];
  bool upper = !t_mode22 && ((hop >> 1) & 1u) != 0u;            /* 22 ms: every packet carries all 7 channels */
  const uint8_t *map = t_mode22 ? DTX_MAP_22 : (upper ? DTX_MAP_UPPER : DTX_MAP_LOWER);
  by[0] = 0x10; by[1] = t_id[2]; by[2] = t_id[3];
  for (int i = 0; i < 7; i++) {
    uint16_t w = 0xFFFFu;
    if (map[i] != 0xFFu) w = (uint16_t)(((upper && i == 0) ? 0x8000u : 0u) | ((uint16_t)map[i] << 11) | t_val(t_input(DTX_IN_MAP[map[i]])));
    by[3 + 2 * i] = (uint8_t)(w >> 8); by[4 + 2 * i] = (uint8_t)w;
  }
  uint16_t crc = t_crc16(by, 17u, t_seed[hop & 1u]);
  by[17] = (uint8_t)crc; by[18] = (uint8_t)(crc >> 8);
  uint8_t row = t_row_of(hop);
  uint64_t sop = t_code64(dtx_pn[row][t_sop_col]);
  uint64_t c0 = t_code64(dtx_pn[row][t_data_col]), c1 = t_code64(dtx_pn[row][t_data_col + 1u]);
  t_begin();
  t_put64(sop); t_put(0); t_put(1); t_put(0); t_put(1); t_put64(~sop);
  for (uint32_t k = 0; k < DTX_DATA_SYMS; k++) {
    unsigned v = by[k];
    uint64_t w = t_rotr64((v & 0x40u) ? c1 : c0, (v - 1u) & 63u);
    t_put64((v & 0x80u) ? w : ~w);
  }
}

/* ---- radio ---- */
static void t_set_len(uint16_t len) { if (len != t_len) { if (sl_rail_set_fixed_length(t_rail, len) != len) dsm_tx_diag[DT_CFG_ERR] |= 2; t_len = len; } }
static void t_launch(uint8_t rf, uint16_t len, uint32_t when)     /* the packet in t_pkt at absolute time when (0 = now) */
{
  /* v1.7: bind packets (state 0) use the shorter Lemon-like preamble; v1.8: data packets (state 2) too */
  bool short_pre = (DTX_LEMON_BIND && t_state == 0u) || (DTX_LEMON_DATA && t_state == 2u);
  sl_rail_tx_options_t opt = short_pre ? SL_RAIL_TX_OPTION_ALT_PREAMBLE_LEN : SL_RAIL_TX_OPTIONS_DEFAULT;
  t_set_len(len);
  (void)sl_rail_write_tx_fifo(t_rail, t_pkt, len, true);
  sl_rail_status_t s;
  uint32_t now = t_now();
  if (when == 0u || (int32_t)(when - now) < (int32_t)DTX_MIN_SLACK_US) {
    if (when != 0u) dsm_tx_diag[DT_LATE]++;
    s = sl_rail_start_tx(t_rail, rf, opt, NULL);
  } else {
    sl_rail_scheduled_tx_config_t cfg = { .when = when, .mode = SL_RAIL_TIME_ABSOLUTE, .tx_during_rx = SL_RAIL_SCHEDULED_TX_DURING_RX_POSTPONE_TX };
    s = sl_rail_start_scheduled_tx(t_rail, rf, opt, &cfg, NULL);
    if (s != SL_RAIL_STATUS_NO_ERROR) { dsm_tx_diag[DT_LATE]++; s = sl_rail_start_tx(t_rail, rf, opt, NULL); }
  }
  if (s != SL_RAIL_STATUS_NO_ERROR) { dsm_tx_diag[DT_TXERR]++; dsm_tx_diag[DT_CFG_ERR] |= 64; t_done = true; }
  else t_busy = true;
}
static void t_set_sync(uint32_t w)
{
  sl_rail_sync_word_config_t sc = { .sync_word_bits = 32u, .sync_word_0 = w, .sync_word_1 = w };
  if (sl_rail_config_sync_words(t_rail, &sc) != SL_RAIL_STATUS_NO_ERROR) dsm_tx_diag[DT_CFG_ERR] |= 32;
}
static void t_listen_answer(void)
{
  sl_rail_idle(t_rail, SL_RAIL_IDLE_ABORT, true);
  /* the answer's byte 0 is ~id[0] (Multi checks it), sent LSB first: its first symbol is the answer
   * code for a 1 bit, the inverse for a 0 -> its first 32 chips are known exactly */
  uint32_t lo = (uint32_t)t_code64(dtx_answer_code);
  t_ans_sync = (((uint8_t)~t_id[0]) & 1u) ? lo : ~lo;
  t_set_sync(t_ans_sync); t_ans_codesync = true;
  t_en_n = 0; t_en_on = false; t_en_min = 32767; t_en_max = -32768;
  t_set_len(DTX_ANSWER_LEN);
  if (sl_rail_start_rx(t_rail, DTX_BIND_RF, NULL) != SL_RAIL_STATUS_NO_ERROR) dsm_tx_diag[DT_CFG_ERR] |= 8;
}
static void t_energy_sample(uint32_t now)       /* state 1: RSSI bursts on the bind channel */
{
  int16_t q = sl_rail_get_rssi(t_rail, false);
  if (q == SL_RAIL_RSSI_INVALID) return;
  int16_t dbm = (int16_t)(q / 4);
  if (dbm < t_en_min) t_en_min = dbm;
  if (dbm > t_en_max) t_en_max = dbm;
  if (dbm >= DTX_EN_DBM) {
    if (!t_en_on) { t_en_on = true; t_en_start = now; t_en_peak = dbm; }
    else if (dbm > t_en_peak) t_en_peak = dbm;
    t_en_last = now;
  } else if (t_en_on) {
    t_en_on = false;
    if (t_en_n < DTX_EN_MAX) { t_en_t[t_en_n] = t_en_start - t_answer_since; t_en_len[t_en_n] = t_en_last - t_en_start; t_en_pk[t_en_n] = (int8_t)t_en_peak; }
    t_en_n++;
  }
}
static void t_energy_report(void)
{
  if (t_en_on) {                                              /* a burst still going when the wait ended */
    t_en_on = false;
    if (t_en_n < DTX_EN_MAX) { t_en_t[t_en_n] = t_en_start - t_answer_since; t_en_len[t_en_n] = t_en_last - t_en_start; t_en_pk[t_en_n] = (int8_t)t_en_peak; }
    t_en_n++;
  }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("DT energy on %u MHz after the burst: %lu bursts >= %d dBm, RSSI range %d..%d dBm\r\n", 2400u + DTX_BIND_RF,
          (unsigned long)t_en_n, DTX_EN_DBM, (int)t_en_min, (int)t_en_max);
  for (uint32_t i = 0; i < t_en_n && i < DTX_EN_MAX; i++)
    app_log("DT   burst +%lu.%01lu ms  %lu us  peak %d dBm\r\n", (unsigned long)(t_en_t[i] / 1000u), (unsigned long)((t_en_t[i] / 100u) % 10u),
            (unsigned long)t_en_len[i], (int)t_en_pk[i]);
#endif
}
static bool t_decode_answer(uint32_t bits)
{
  uint64_t code = t_code64(dtx_answer_code); int32_t p0 = -1; unsigned best = 64u;
  for (uint32_t p = 0; p + 64u <= bits && p < 500u; p++) {
    unsigned d = t_pop64(t_get64(p) ^ code);
    unsigned e = (d < 32u) ? d : 64u - d;                         /* the code or its inverse (a 0 bit first) */
    if (e < best) best = e;
    if (e <= 8u) { p0 = (int32_t)p; break; }
  }
  bool show = (t_ans_logged < DTX_ANSWER_LOG_MAX);
  unsigned long ms = (unsigned long)((t_now() - t_answer_since) / 1000u);
  if (show) t_ans_logged++;
  if (p0 < 0 || (uint32_t)p0 + 80u * 64u > bits) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    if (show) app_log("DT ans +%lums: %u chips, no response code (best %u errors of 64)\r\n", ms, (unsigned)bits, best);
#endif
    return false;
  }
  uint8_t pk[10]; memset(pk, 0, sizeof(pk));
  for (uint32_t k = 0; k < 80u; k++) if (t_pop64(t_get64((uint32_t)p0 + 64u * k) ^ code) < 32u) pk[k >> 3] |= (uint8_t)(1u << (k & 7u));
  bool id_ok = true;
  for (int i = 0; i < 4; i++) if (pk[i] != (uint8_t)~t_id[i]) id_ok = false;
  uint16_t sum = 384u - 0x10u; for (int i = 0; i < 8; i++) sum += pk[i];
  bool hi_ok = (pk[8] == (uint8_t)(sum >> 8)), lo_ok = (pk[9] == (uint8_t)sum);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (show) app_log("DT ans +%lums @%ld: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X  id %s  sum %s (want %02X %02X)\r\n",
                    ms, (long)p0, pk[0], pk[1], pk[2], pk[3], pk[4], pk[5], pk[6], pk[7], pk[8], pk[9],
                    id_ok ? "ok" : "BAD", (hi_ok && lo_ok) ? "ok" : (hi_ok || lo_ok) ? "half" : "BAD",
                    (unsigned)(uint8_t)(sum >> 8), (unsigned)(uint8_t)sum);
#endif
  if (!id_ok) return false;
  if (!hi_ok && !lo_ok) return false;     /* as Multi DSM_Check_RX_packet: rejected only when BOTH sum bytes differ */
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("DT bound by receiver +%lums: version %02X, %u ch, type %02X -> data now\r\n", ms, pk[4], (unsigned)pk[5], pk[6]);
#endif
  return true;
}

/* ---- states ---- */
static void t_enter_data(void)
{
  sl_rail_idle(t_rail, SL_RAIL_IDLE_ABORT, true);
  if (t_ans_codesync) { t_set_sync(DTX_SYNC_PRE); t_ans_codesync = false; }   /* data packets need the preamble sync back */
  sl_rail_set_tx_power_dbm(t_rail, t_range ? DTX_RANGE_POWER_DDBM : t_flight_ddbm());   /* flight power (per rate) after the bind */
#if DTX_LEMON_DATA
  /* v1.8: set again here, after the sync words were reconfigured, so the data packets surely get it */
  if (sl_rail_set_tx_alt_preamble_length(t_rail, DTX_SHORT_PREAMBLE) != SL_RAIL_STATUS_NO_ERROR) dsm_tx_diag[DT_CFG_ERR] |= 16;
#endif
  if (t_state == 1u) t_energy_report();
  t_state = 2; dsm_tx_diag[DT_STATE] = 2;
  t_hop = 0; t_next = t_now() + 2000u; t_busy = false; t_done = false;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("DT data: 2 packets per %u ms over the 23 channels, preamble %u chips, cfg_err=%ld%s\r\n", (unsigned)(t_period() / 1000u),
          (unsigned)DTX_DATA_PREAMBLE_SHOWN, (long)dsm_tx_diag[DT_CFG_ERR], t_ppm ? "" : " (no PPM: centre sticks, throttle low)");
#endif
}

void dsm_tx_on_events(sl_rail_events_t events)
{
  if (events & SL_RAIL_EVENTS_TX_COMPLETION) { t_done = true; if (!(events & SL_RAIL_EVENT_TX_PACKET_SENT)) t_txerr = true; }
}

void dsm_tx_start(sl_rail_handle_t rh, const volatile uint16_t *ch_us, bool bind_held, bool range_low)
{
  t_rail = rh; t_in = ch_us; t_range = range_low;
  for (uint8_t i = 0; i < DTX_DIAG_N; i++) dsm_tx_diag[i] = 0;
  t_len = 0; t_ppm = false; t_busy = false; t_done = false; t_txerr = false; t_tx_count = 0; t_tx_last = 0;
  t_ans_frames = 0; t_ans_logged = 0; t_early_frames = 0;
  t_uart_us = t_now();
  sl_gpio_set_pin_mode(&t_led_a, SL_GPIO_MODE_PUSH_PULL, 0);
  sl_gpio_set_pin_mode(&t_led_b, SL_GPIO_MODE_PUSH_PULL, 0);
  t_gen_id();

  /* v1.13: the frame rate. BIND at power-up chooses it (RANGE held too = 11 ms, else 22 ms) and stores it, so
   * later power-ups keep sending what the receiver was bound with; without a stored bind: DTX_MODE_22MS. */
  rx_record_t rec;
  bool have = rx_store_load(RX_SLOT_DSMTX, &rec) && rec.bound && (rec.hop[0] == 22u || rec.hop[0] == 11u);
  if (bind_held) {
    t_mode22 = !range_low || DTX_EU_LOCK;
    t_mode_from = (range_low && DTX_EU_LOCK) ? LOG_TXT("chosen at this bind; 11 ms is locked out in this EU build") : LOG_TXT("chosen at this bind");
    if (!have || (rec.hop[0] == 22u) != t_mode22) {             /* flash is written only when the rate changes */
      memset(&rec, 0, sizeof(rec)); rec.bound = 1; rec.hop[0] = t_mode22 ? 22u : 11u;
      if (!rx_store_save(RX_SLOT_DSMTX, &rec)) dsm_tx_diag[DT_CFG_ERR] |= 128;
    }
  } else if (have) {
    t_mode22 = (rec.hop[0] == 22u) || DTX_EU_LOCK;
    t_mode_from = (rec.hop[0] == 11u && DTX_EU_LOCK) ? LOG_TXT("11 ms stored at the last bind, locked out in this EU build: rebind the receiver")
                                                     : LOG_TXT("stored at the last bind");
  }
  else             { t_mode22 = (DTX_MODE_22MS != 0) || DTX_EU_LOCK; t_mode_from = LOG_TXT("default, no bind stored"); }

  sl_rail_idle(rh, SL_RAIL_IDLE_ABORT, true);
  if (sl_rail_config_channels(rh, (const sl_rail_channel_config_t *)&dsm_raw_channelConfig, NULL) != SL_RAIL_STATUS_NO_ERROR) dsm_tx_diag[DT_CFG_ERR] |= 1;
  sl_rail_prepare_channel(rh, DTX_BIND_RF);
#if DTX_LEMON_BIND || DTX_LEMON_DATA
  if (sl_rail_set_tx_alt_preamble_length(rh, DTX_SHORT_PREAMBLE) != SL_RAIL_STATUS_NO_ERROR) dsm_tx_diag[DT_CFG_ERR] |= 16;   /* after the PHY is loaded */
#endif
  if (sl_rail_set_tx_fifo(rh, t_txfifo, DTX_TX_FIFO_BYTES, 0, 0) != SL_RAIL_STATUS_NO_ERROR) dsm_tx_diag[DT_CFG_ERR] |= 4;
  uint16_t fsz = DTX_RX_FIFO_BYTES;
  if (sl_rail_set_rx_fifo(rh, t_rxfifo, &fsz) != SL_RAIL_STATUS_NO_ERROR) dsm_tx_diag[DT_CFG_ERR] |= 4;
  sl_rail_sync_word_config_t sc = { .sync_word_bits = 32u, .sync_word_0 = DTX_SYNC_PRE, .sync_word_1 = DTX_SYNC_PRE };
  if (sl_rail_config_sync_words(rh, &sc) != SL_RAIL_STATUS_NO_ERROR) dsm_tx_diag[DT_CFG_ERR] |= 32;
  sl_rail_config_events(rh, SL_RAIL_EVENT_RX_PACKET_RECEIVED | SL_RAIL_EVENTS_TX_COMPLETION, SL_RAIL_EVENT_RX_PACKET_RECEIVED | SL_RAIL_EVENTS_TX_COMPLETION);
  sl_rail_set_tx_power_dbm(rh, bind_held ? DTX_BIND_POWER_DDBM : (range_low ? DTX_RANGE_POWER_DDBM : t_flight_ddbm()));   /* v1.13: the bind always at bind power */

#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("DT boot dsm_tx v1.14 mode type %02X %u ch %u ms (%s), bind %u packets every %lu us, preamble bind %u data %u; cfg_err=%ld id %02X %02X %02X %02X sop_col %u power %s (flight %d, bind %d, range %d ddBm; RAIL set %d now); channels:",
          (unsigned)t_type(), (unsigned)t_num_ch(), (unsigned)(t_period() / 1000u), t_mode_from, (unsigned)DTX_BIND_COUNT, (unsigned long)DTX_BIND_EVERY_US,
          (unsigned)DTX_BIND_PREAMBLE_SHOWN, (unsigned)DTX_DATA_PREAMBLE_SHOWN, (long)dsm_tx_diag[DT_CFG_ERR],
          t_id[0], t_id[1], t_id[2], t_id[3], (unsigned)t_sop_col, range_low ? "range" : "normal",
          (int)t_flight_ddbm(), (int)DTX_BIND_POWER_DDBM, (int)DTX_RANGE_POWER_DDBM, (int)sl_rail_get_tx_power_dbm(rh));
  for (uint8_t i = 0; i < DTX_HOPS; i++) app_log(" %u", (unsigned)t_rf_of(i));
  app_log(" (MHz above 2400)\r\n");
#endif
  if (bind_held) {
    t_state = 0; dsm_tx_diag[DT_STATE] = 0; t_bind_sent = 0;
    t_build_bind();
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("DT bind at %u ms (%u channels, type %02X): %u packets on %u MHz, then listening for the receiver's answer\r\n",
            (unsigned)(t_period() / 1000u), (unsigned)t_num_ch(), (unsigned)t_type(), (unsigned)DTX_BIND_COUNT, 2400u + DTX_BIND_RF);
#endif
    t_next = t_now() + 2000u;
    t_launch(DTX_BIND_RF, DTX_BIND_LEN, t_next); t_bind_sent = 1;
  } else t_enter_data();
}

/* v1.12: RANGE follows the input all the time. In data the new power is applied between two packets
 * (dsm_tx_action); during the bind and the answer wait it is taken up when the data starts. */
void dsm_tx_range(bool range_low)
{
  if (range_low == t_range) return;
  t_range = range_low;
  if (t_state == 2u) t_pwr_dirty = true;
}

void dsm_tx_input(bool ppm_present)
{
  if (ppm_present != t_ppm) {
    t_ppm = ppm_present; dsm_tx_diag[DT_PPM] = ppm_present ? 1 : 0;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    if (t_state == 2u) app_log(ppm_present ? "DT PPM present\r\n" : "DT no PPM: sending centre sticks, throttle low\r\n");
#endif
  }
}

void dsm_tx_rx_process(void)
{
  for (;;) {
    sl_rail_rx_packet_info_t info;
    sl_rail_rx_packet_handle_t h = sl_rail_get_rx_packet_info(t_rail, SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &info);
    if (h == SL_RAIL_RX_PACKET_HANDLE_INVALID) return;
    /* v1.10: only the answer wait (state 1, 700-byte frames) copies a frame; anything else -- above all the 1027-byte
     * frames heard between our own bind packets (state 0) -- is counted and dropped without copying.
     * The bug this removes (v1.3..v1.9): n was clamped to 700 by writing info.packet_bytes = 700, but
     * sl_rail_copy_rx_packet copies ALL first_portion_bytes and then (packet_bytes - first_portion_bytes) as a
     * uint16_t from the wrapped part. A 1027-byte frame that wrapped the 2 kB RX FIFO with more than 700 bytes
     * before the wrap made that 700 - (>700) -> ~65000 bytes copied over RAM: the TX froze in the middle of the
     * bind burst (log ends after "DT bind:", LEDs stop, no data until a TX reset). The first frame in the FIFO
     * does not wrap, which is why "1 frames heard in the gaps" survived; the Lemon RX, which transmits during
     * our burst, made the second one likely. */
    if (t_state != 1u || info.packet_bytes > DTX_ANSWER_LEN) {
      sl_rail_release_rx_packet(t_rail, h);
      if (t_state == 0u) t_early_frames++;                      /* heard between our own bind packets */
      continue;
    }
    uint16_t n = info.packet_bytes;                             /* <= DTX_ANSWER_LEN: the whole frame fits, info untouched */
    memset(t_w, 0, sizeof(t_w));
    uint32_t off = 0u;
    if (t_ans_codesync) {                                       /* the sync word was the symbol's first 32 chips: put them back in front */
      memcpy((uint8_t *)t_w, &t_ans_sync, 4u); off = 4u;
    }
    sl_rail_copy_rx_packet(t_rail, (uint8_t *)t_w + off, &info);
    sl_rail_release_rx_packet(t_rail, h);
    t_ans_frames++;
    if (t_decode_answer((uint32_t)(n + off) * 8u)) { dsm_tx_diag[DT_ANSWERS]++; t_enter_data(); return; }
  }
}

void dsm_tx_action(void)
{
  uint32_t now = t_now();
  if (t_state == 0u) {                                            /* bind burst */
    if (t_done) {
      t_done = false; t_busy = false;
      if (t_bind_sent >= DTX_BIND_COUNT) {
        t_state = 1; dsm_tx_diag[DT_STATE] = 1; t_answer_since = now;
        t_listen_answer();
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("DT burst done (%lu packets, %lu frames heard in the gaps); listening on %u MHz for the answer\r\n",
                (unsigned long)t_bind_sent, (unsigned long)t_early_frames, 2400u + DTX_BIND_RF);
#endif
      } else { t_next += DTX_BIND_EVERY_US; t_launch(DTX_BIND_RF, DTX_BIND_LEN, t_next); t_bind_sent++; }
    }
  } else if (t_state == 1u) {                                     /* waiting for the answer */
    t_energy_sample(now);
    if ((now - t_answer_since) >= DTX_ANSWER_WAIT_US) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("DT no answer from a receiver (4.2 s, %lu frames heard); transmitting anyway\r\n", (unsigned long)t_ans_frames);
#endif
      t_enter_data();
    }
  } else {                                                        /* data */
    if (t_busy && t_done) { t_busy = false; t_done = false; if (t_txerr) { t_txerr = false; dsm_tx_diag[DT_TXERR]++; } }
    if (!t_busy && t_pwr_dirty) {                               /* v1.12: RANGE changed: new power while the radio is idle */
      t_pwr_dirty = false;
      sl_rail_idle(t_rail, SL_RAIL_IDLE, true);
      sl_rail_set_tx_power_dbm(t_rail, t_range ? DTX_RANGE_POWER_DDBM : t_flight_ddbm());
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("DT RANGE %s: power %d ddBm (RAIL set %d)\r\n", t_range ? "on" : "off",
              (int)(t_range ? DTX_RANGE_POWER_DDBM : t_flight_ddbm()), (int)sl_rail_get_tx_power_dbm(t_rail));
#endif
    }
    if (!t_busy) {
      if ((int32_t)(now - t_next) >= -(int32_t)3000) {           /* the next launch is within 3 ms (or overdue) */
        if ((int32_t)(now - t_next) > (int32_t)t_period()) {     /* far behind (PPM was off): restart the timeline */
          t_next = now + 1000u; t_hop = (t_hop + 1u) & ~1u;
        }
        t_build_data(t_hop);                                     /* with or without PPM (no PPM: centre, throttle low) */
        t_launch(t_rf_of(t_hop), DTX_DATA_LEN, t_next);
        dsm_tx_diag[DT_TX]++; t_tx_count++; dsm_tx_diag[DT_HOP] = (int32_t)(t_hop % DTX_HOPS);
        t_next += (t_hop & 1u) ? (t_period() - DTX_PAIR_US) : DTX_PAIR_US;
        t_hop++;
      }
    }
  }

  /* LED_B: bind fast blink, data steady with PPM, slow blink without; LED_A toggles per 20 packets */
  static uint32_t tb = 0; static uint32_t la = 0;
  if (t_state == 2u && t_ppm) sl_gpio_set_pin(&t_led_b);   /* no PPM: slow blink, still transmitting */
  else if ((now - tb) > (t_state != 2u ? 100000u : 500000u)) { tb = now; sl_gpio_toggle_pin(&t_led_b); }
  if (t_tx_count / 20u != la) { la = t_tx_count / 20u; sl_gpio_toggle_pin(&t_led_a); }

#if DTX_UART_STATUS && defined(SL_CATALOG_APP_LOG_PRESENT)
  if ((now - t_uart_us) >= 1000000u) {
    t_uart_us = now;
    if (t_state == 2u)
      app_log("DT s2 tx%lu late%ld err%ld cal%lu ppm%d h%lu ch %u %u %u %u %u %u %u %u\r\n", (unsigned long)(t_tx_count - t_tx_last),
              (long)dsm_tx_diag[DT_LATE], (long)dsm_tx_diag[DT_TXERR], (unsigned long)cal_count, t_ppm ? 1 : 0, (unsigned long)(t_hop % DTX_HOPS),
              t_in[0], t_in[1], t_in[2], t_in[3], t_in[4], t_in[5], t_in[6], t_in[7]);
    else if (t_state == 0u)                                     /* quiet during the answer wait: keeps the RSSI sampling continuous */
      app_log("DT s%d bind sent=%lu\r\n", (int)t_state, (unsigned long)t_bind_sent);
    t_tx_last = t_tx_count;
  }
#endif
}
