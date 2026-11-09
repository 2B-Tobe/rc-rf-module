/* dsm_rx.c — see dsm_rx.h.  #included into app_process.c.   v1.8 (SBUS on by default, like the other receivers: DRX_SBUS 1; bind answer 3 s instead of 5 s; v1.7: profiler answers 25 ms after the burst; v1.6: bind-burst profiler: times the TX bind burst, answers after it; v1.5: own PHY dsm_rx at 280 kHz for the Lemon TX; v1.4: pair cadence from the bind type: A2 = 22 ms, B2 = 11 ms;
 *   v1.3: bind response; F/S jumper selects the role; SBUS off: shares EUSART0 with the log)
 *
 * Air format (measured with dsm_cap.c on the Lemon DIY DSMX module, runs 17-30, and cross-checked
 * with the Multi-module source DSM.ino / DSM_cyrf6936.ino):
 *   1 Mchip/s GFSK +-400 kHz, RF = 2400 + (dsmx channel + 1) MHz (the CYRF6936 carrier sits 1 MHz
 *   above its nominal channel).  Every packet: carrier ~25 us, preamble 1001.. (~108 chips), then
 *   bind: 128 SDR symbols of 64 chips, pn[0][8] = bit 1, inverted = bit 0, LSB first: 16 bytes
 *         ~id x2, sum, 01, channels, type (B2 = DSMX 11 ms), 00, sum;  on a random channel, ~1 s
 *   data: SOP pn[row][sop_col] 64 chips | 4 chips | ~SOP 64 chips | 19 8DR symbols of 64 chips:
 *         byte v -> code = pn[row][data_col + (v>>6 & 1)] rotated right by (v-1)&63, all chips
 *         inverted when bit 7 of v is clear.  Bytes: 10, id2, id3, 7 words (bit 15 = upper frame
 *         on word 0, bits 14..11 channel id, 10..0 value), CRC-16 (0xA001 reflected, init =
 *         bit-reversed seed, xor FFFF, low byte first) over the 17 bytes.
 *         row = (dsmx-2)%5, sop_col = (id0+id1+id2+2)&7, data_col = 7-sop_col,
 *         seed = ~(id0<<8|id1) on the first packet of a pair, id0<<8|id1 on the second.
 *   hopping: 23 channels from the id (Multi's DSM_calc_dsmx_channel), hop h on list[h % 23];
 *         h even at t = (h/2) x 11 ms, h odd 4.01 ms later; the same frame goes out on both hops
 *         of a pair; lower (ids 1 5 2 4 6 10) and upper (0 7 3 8 9 11) frames alternate.
 * Receiver: the modem syncs on the preamble (sync word 0x99999999), delivers fixed-length frames
 *   of hard chips; the SOP is found by a 64-chip Hamming search, each symbol by a 12-chip lookup
 *   (verified over 64) with a 128-rotation fallback, then the CRC.  A good packet sets the timeline;
 *   the radio is retuned to the next hop as soon as the SOP is seen.  Misses advance the timeline
 *   on the nominal timing (the frame period is tracked from the packets). */
#include "dsm_rx.h"
#include "servo_pins.h"
#include "rail_config.h"
#include "rx_store.h"
#include "sbus_out.h"
#include "sl_gpio.h"
#include <string.h>
#if defined(SL_CATALOG_APP_LOG_PRESENT)
#include "app_log.h"
#endif

/* v1.5: the receiver has its own PHY, dsm_rx = dsm_raw with 280 kHz deviation (tag dsmx-orange-ok): the Lemon TX
 * (emulated DSMX) is decoded at 280 kHz but not at 300 kHz. The transmitter keeps dsm_raw at 300 kHz, which the
 * AR610 and OrangeRX need (at 280 kHz they bind but take no data). 2GFSK 1 Mbps, 1001 preamble, ch n = 2400 + n MHz */
extern const RAIL_ChannelConfig_t dsm_rx_channelConfig;

#define DRX_HOPS            23u
#define DRX_FRAME_US     11000u      /* nominal; tracked in drx_frame_us (the Lemon runs 11023)      */
#define DRX_PAIR_US       4030u      /* 2nd packet of a pair after the 1st (Multi 4010; the Lemon measured 4030) */
#define DRX_SYNC_PRE 0x99999999ul    /* the preamble as the sync word: any DSM frame is caught      */
#define DRX_DATA_BYTES     184u      /* SOP at chip <= 124 + 132 + 19 x 64 fits in 1472 chips       */
#define DRX_BIND_BYTES    1056u      /* up to 256 chips of preamble rest + 128 x 64 chips           */
#define DRX_SYMS            19u
#define DRX_SOP_ERRS        10u      /* SOP accepted with up to this many wrong chips of 64         */
#define DRX_SOP_SEARCH     300u      /* chips searched for the SOP (seen at 15..35 in the captures)  */
#define DRX_SYM_ERRS        12u      /* a lookup candidate is accepted with up to this many errors  */
#define DRX_MISS_PAD_US   2500u      /* packet this late -> counted as missed                       */
#define DRX_FS_MISSES       90u      /* ~500 ms without packets -> failsafe                         */
#define DRX_LOST_MISSES    360u      /* ~2 s -> search again                                        */
#define DRX_SEARCH_DWELL_US 300000u  /* per channel; the TX visits each every 126.5 ms              */
#define DRX_BIND_LO          2u      /* bind scan RF channels (the Lemon bound on 2408)              */
#define DRX_BIND_HI         80u
#define DRX_BIND_PEEK_US   2500u     /* RSSI look on each channel                                   */
#define DRX_BIND_STAY_US  20000u     /* stay this long when something is there (bind packet 8.4 ms) */
#define DRX_BIND_RSSI       -85
#define DRX_BIND_TIMEOUT_US 8000000u /* BIND at boot with a stored bind: revert after this          */
/* bind response (Multi DSM_RX_BIND2): 10 bytes, 64-chip SDR with the response code, no SOP, no CRC, on the
 * bind channel, every 10 ms for 3 s (v1.8; was 5 s); the TX reads it after its own burst: our DSMX TX and the
 * Lemon TX within ~30 ms, Multi within its 4.2 s window. Shorter = the outputs come alive sooner after a bind. */
#define DRX_RESP_BYTES      10u
#define DRX_RESP_PAD_SYMS    2u      /* two idle symbols after the data: the CYRF ends the packet on 2 non-correlating symbols */
#define DRX_RESP_LEN     ((DRX_RESP_BYTES * 8u + DRX_RESP_PAD_SYMS) * 8u)   /* bytes on the air after the sync: 656 */
#define DRX_RESP_EVERY_US 10000u
#define DRX_RESP_FOR_US 3000000u
#define DRX_TX_FIFO_BYTES 1024u
#define DRX_FS_PRESS_US   300000u    /* BIND held this long while linked -> store preset            */
#define DRX_PPM_CHANNELS     8u
#define DRX_PPM_FRAME_US 22500u
#define DRX_PPM_PULSE_US   300u
#define DRX_THR_FAILSAFE  1000u
#ifndef DRX_UART_STATUS
#define DRX_UART_STATUS      1    /* diagnostic: watching q/c/n for a periodic dip while linked to A */
#endif
/* SBUS (sbus_out.c) is on EUSART1 / PA03 since sbus_out v1.1; the log keeps PA05 in every role.
 * 1 = PPM, servos and SBUS, like the other receivers (default since v1.8).  0 = no SBUS. */
#ifndef DRX_SBUS
#define DRX_SBUS             1
#endif
#define DRX_UART_US     200000u
#define DRX_FIFO_BYTES    4096u

/* DSM channel id 0..7 (T A E R Gear Aux1 Aux2 Aux3) -> output index 0..7: straight through, so servo 1 = throttle,
 * 2 = aileron, 3 = elevator, 4 = rudder, as on a Spektrum receiver and as Tobe's encoder orders its channels (TAER).
 * The nRF24 and RLINK receiver roles put the channels out in the order sent, which is the same order. */
static const uint8_t DRX_OUT_MAP[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };

static const uint8_t drx_pn[5][9][8] = {
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

volatile int32_t dsm_rx_diag[DRX_DIAG_N];
static sl_rail_handle_t d_rail;
static SL_RAIL_DECLARE_FIFO_BUFFER(d_fifo, DRX_FIFO_BYTES);
static SL_RAIL_DECLARE_FIFO_BUFFER(d_txfifo, DRX_TX_FIFO_BYTES);
static const uint8_t drx_resp_code[8] = { 0x98, 0x88, 0x1B, 0xE4, 0x30, 0x79, 0x03, 0x84 };   /* the bind-response data code */
static uint8_t  d_resp[DRX_RESP_LEN];               /* the response as chips, ready for the TX FIFO */
static uint32_t d_resp_next, d_resp_until, d_resp_sent;
static uint8_t  d_id[4];
static uint8_t  d_list[DRX_HOPS];                   /* DSMX channel numbers in hop order; RF = +1 */
static uint8_t  d_sop_col, d_type, d_nch;
static uint16_t d_seed[2];                          /* CRC seed of the 1st / 2nd packet of a pair */
/* lut[0]: chips 20..31 of a symbol -> 1 + code*64 + rotation (0 = none); lut[1]: chips 44..55.  Two keys away
 * from the symbol edges, where the captures show the chip errors sit (the worst symbol has 1 wrong chip). */
#define DRX_KEY_A 20u
#define DRX_KEY_B 44u
typedef struct { uint64_t sop, code[2]; uint8_t lut[2][4096]; } drx_row_t;
static drx_row_t d_row[5];
static uint8_t  d_state;                            /* 0 bind, 1 search, 2 link, 3 bind response, 4 bind-burst profile */

/* v1.6 bind-burst profiler (state 4): after the first bind packet, stay on the bind channel, time every further
 * bind packet (sync-word-end timestamps) and every RF burst (RSSI within DRX_PROF_REL_DB of the first packet),
 * until the channel has been quiet DRX_PROF_QUIET_US; then answer (as a Lemon RX does: after the burst). The
 * summary is printed when the answer phase ends, so the printing never delays anything time-critical. */
#ifndef DRX_PROFILE
#define DRX_PROFILE          1
#endif
#define DRX_PROF_QUIET_US  25000u    /* v1.7: 25 ms = two missing packets at the Lemon's 10.5 ms spacing (150 ms was too late: the Lemon TX showed red, no answer) */
#define DRX_PROF_MAX_US  4000000u
#define DRX_PROF_REL_DB       15
#define DRX_PROF_N           200u
#define DRX_PROF_MERGE_US    200u    /* RSSI reads closer than this belong to the same burst */
static uint32_t p_pkt_t[DRX_PROF_N], p_en_t[DRX_PROF_N], p_en_len[DRX_PROF_N];
static int8_t   p_en_pk[DRX_PROF_N];
static uint32_t p_pkt_n, p_bad_n, p_en_n, p_start_us, p_last_act, p_en_start, p_en_last;
static int16_t  p_thresh, p_en_peak;
static bool     p_en_on, p_valid;
static int32_t  d_last_p0;                          /* chip offset of the first bind symbol after the sync word */
static int32_t  p_p0_min, p_p0_max, p_p0_sum, p_p0_n;
static uint8_t  d_idx, d_pos;                       /* hop index listening on; expected pair position 0/1 */
static uint32_t d_expect, d_arm_us, d_dwell_us, d_bind_deadline, d_uart_us, d_chan_us;
static int32_t  d_frame_us;                         /* tracked frame period */
static int32_t  d_last_err;                         /* last packet's timing error against the timeline, us */
static uint32_t d_misses;
static bool     d_open, d_binding, d_have_saved, d_in_fs, d_btn_done, d_btn_released, d_bind_stay;
static uint32_t d_btn_since;
static uint16_t d_out_us[8];
static uint8_t  d_bind_ch;
static uint16_t d_len;                              /* fixed frame length in use */
static rx_record_t d_rec, d_saved;
static uint64_t d_w[DRX_BIND_BYTES / 8u + 2u];      /* the frame as 64-bit words: chip i = bit i */
static uint8_t  d_bytes[DRX_SYMS];

static sl_gpio_t d_led_a   = { .port = SL_GPIO_PORT_C, .pin = 4 };
static sl_gpio_t d_led_b   = { .port = SL_GPIO_PORT_C, .pin = 5 };
static uint32_t d_last_rx;                 /* time of the last valid data packet (dual receiver) */

static inline uint32_t d_now(void) { return (uint32_t)sl_rail_get_time(d_rail); }

/* ---- bit helpers ---- */
static inline unsigned d_pop32(uint32_t x)
{
  x = x - ((x >> 1) & 0x55555555u);
  x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
  x = (x + (x >> 4)) & 0x0F0F0F0Fu;
  return (x * 0x01010101u) >> 24;
}
static inline unsigned d_pop64(uint64_t x) { return d_pop32((uint32_t)x) + d_pop32((uint32_t)(x >> 32)); }
static inline uint64_t d_rotr64(uint64_t w, unsigned n) { n &= 63u; return n ? ((w >> n) | (w << (64u - n))) : w; }
static uint64_t d_code64(const uint8_t *c) { uint64_t w = 0; for (int i = 0; i < 8; i++) w |= (uint64_t)c[i] << (8 * i); return w; }
static inline uint64_t d_get64(uint32_t p)          /* 64 chips from chip position p of the frame */
{
  uint32_t i = p >> 6, b = p & 63u;
  return b ? ((d_w[i] >> b) | (d_w[i + 1] << (64u - b))) : d_w[i];
}

/* ---- the id and what follows from it ---- */
static void d_calc_channels(void)                   /* Multi-module DSM_calc_dsmx_channel */
{
  uint32_t id = ~(((uint32_t)d_id[0] << 24) | ((uint32_t)d_id[1] << 16) | ((uint32_t)d_id[2] << 8) | d_id[3]);
  uint32_t t = id; uint8_t idx = 0;
  while (idx < DRX_HOPS) {
    t = t * 0x0019660Du + 0x3C6EF35Fu;
    uint8_t next = (uint8_t)(((t >> 8) % 0x49u) + 3u);
    if ((next ^ d_id[3]) & 1u) continue;
    uint8_t c1 = 0, c2 = 0, c3 = 0, i;
    for (i = 0; i < idx; i++) {
      if (d_list[i] == next) break;
      if (d_list[i] <= 27u) c1++; else if (d_list[i] <= 51u) c2++; else c3++;
    }
    if (i != idx) continue;
    if ((next < 28u && c1 < 8u) || (next >= 28u && next < 52u && c2 < 7u) || (next >= 52u && c3 < 8u)) d_list[idx++] = next;
  }
}
/* v1.4: the pair cadence follows the bind type: A2 = DSMX 22 ms (one pair per 22 ms, <= 7 channels,
 * as a DX6i or our TX in its 22 ms mode), otherwise 11 ms (B2: pairs every 11 ms, the Lemon TX). */
static inline int32_t d_nominal_us(void) { return ((d_type & 0xF0u) == 0xA0u) ? 22000 : (int32_t)DRX_FRAME_US; }
static void d_setup_id(void)
{
  d_calc_channels();
  d_sop_col = (uint8_t)((d_id[0] + d_id[1] + d_id[2] + 2u) & 7u);
  uint8_t data_col = (uint8_t)(7u - d_sop_col);
  d_seed[1] = (uint16_t)(((uint16_t)d_id[0] << 8) | d_id[1]);
  d_seed[0] = (uint16_t)~d_seed[1];
  for (uint8_t r = 0; r < 5u; r++) {
    drx_row_t *row = &d_row[r];
    row->sop = d_code64(drx_pn[r][d_sop_col]);
    row->code[0] = d_code64(drx_pn[r][data_col]);
    row->code[1] = d_code64(drx_pn[r][data_col + 1u]);
    memset(row->lut, 0, sizeof(row->lut));
    for (unsigned c = 0; c < 2u; c++)
      for (unsigned rot = 0; rot < 64u; rot++) {
        uint64_t sym = d_rotr64(row->code[c], rot);
        unsigned ka = (unsigned)((sym >> DRX_KEY_A) & 0xFFFu), kb = (unsigned)((sym >> DRX_KEY_B) & 0xFFFu);
        if (!row->lut[0][ka]) row->lut[0][ka] = (uint8_t)(1u + c * 64u + rot);
        if (!row->lut[1][kb]) row->lut[1][kb] = (uint8_t)(1u + c * 64u + rot);
      }
  }
  dsm_rx_diag[DR_ID] = (int32_t)(((uint32_t)d_id[0] << 24) | ((uint32_t)d_id[1] << 16) | ((uint32_t)d_id[2] << 8) | d_id[3]);
  dsm_rx_diag[DR_TYPE] = d_type;
  d_frame_us = d_nominal_us();                                      /* v1.4: 22 ms for type A2 */
}
static inline uint8_t d_row_of(uint8_t idx) { return (uint8_t)((d_list[idx] + 5u - 2u) % 5u); }
static inline uint8_t d_rf_of(uint8_t idx)  { return (uint8_t)(d_list[idx] + 1u); }

/* ---- decoding ---- */
static uint16_t d_crc16(const uint8_t *d, uint32_t n, uint16_t seed)
{
  uint16_t crc = 0; for (int i = 0; i < 16; i++) if (seed & (1u << i)) crc |= (uint16_t)(0x8000u >> i);
  for (uint32_t k = 0; k < n; k++) { crc ^= d[k]; for (int i = 0; i < 8; i++) crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1); }
  return (uint16_t)(crc ^ 0xFFFFu);
}
/* one 8DR symbol -> byte; *dist = chip errors of the match */
static uint8_t d_symbol(const drx_row_t *r, uint64_t sy, unsigned *dist)
{
  for (unsigned t = 0; t < 4u; t++) {                                /* key A / key B, each as is (bit 7 set) and inverted */
    unsigned pol = t & 1u, key = (unsigned)((sy >> ((t < 2u) ? DRX_KEY_A : DRX_KEY_B)) & 0xFFFu);
    uint8_t cand = r->lut[t >> 1][pol ? ((~key) & 0xFFFu) : key];
    if (!cand) continue;
    unsigned c = (unsigned)(cand - 1u) >> 6, rot = (unsigned)(cand - 1u) & 63u;
    uint64_t ref = d_rotr64(r->code[c], rot); if (pol) ref = ~ref;
    unsigned d = d_pop64(sy ^ ref);
    if (d <= DRX_SYM_ERRS) { *dist = d; return (uint8_t)((pol ? 0u : 0x80u) | (c << 6) | ((rot + 1u) & 63u)); }
  }
  dsm_rx_diag[DR_LUT_MISS]++;                                          /* noisy: full search */
  unsigned best = 65, bv = 0;
  for (unsigned c = 0; c < 2u; c++)
    for (unsigned rot = 0; rot < 64u; rot++) {
      unsigned d = d_pop64(sy ^ d_rotr64(r->code[c], rot));
      if (d < best)       { best = d;       bv = 0x80u | (c << 6) | ((rot + 1u) & 63u); }
      if (64u - d < best) { best = 64u - d; bv = (c << 6) | ((rot + 1u) & 63u); }
    }
  *dist = best; return (uint8_t)bv;
}
/* data frame in d_w (bits chips): -1 = no SOP, else the SOP chip position; fills d_bytes, *q = worst match */
static int32_t d_find_sop(uint32_t bits, const drx_row_t *r)
{
  for (uint32_t p = 0; p + 64u <= bits && p < DRX_SOP_SEARCH; p++)
    if (d_pop64(d_get64(p) ^ r->sop) <= DRX_SOP_ERRS) return (int32_t)p;
  return -1;
}
static bool d_decode_syms(uint32_t bits, const drx_row_t *r, uint32_t sop, uint8_t *q)
{
  uint32_t p0 = sop + 64u + 4u + 64u;
  if (p0 + 64u * DRX_SYMS > bits) return false;
  unsigned worst = 0;
  for (uint32_t k = 0; k < DRX_SYMS; k++) { unsigned d; d_bytes[k] = d_symbol(r, d_get64(p0 + 64u * k), &d); if (d > worst) worst = d; }
  *q = (uint8_t)(64u - worst);
  return true;
}
static uint8_t d_check_crc(void)                    /* 0 = 1st of the pair, 1 = 2nd, 2 = bad */
{
  uint16_t got = (uint16_t)(d_bytes[17] | (d_bytes[18] << 8));
  if (got == d_crc16(d_bytes, 17u, d_seed[0])) return 0;
  if (got == d_crc16(d_bytes, 17u, d_seed[1])) return 1;
  return 2;
}
/* bind frame in d_w: 128 SDR symbols with pn[0][8]; true = a valid bind packet, id/type/nch taken */
static bool d_decode_bind(uint32_t bits)
{
  uint64_t code = d_code64(drx_pn[0][8]);
  int32_t p0 = -1;
  for (uint32_t p = 0; p + 64u <= bits && p < 500u; p++) {
    unsigned d = d_pop64(d_get64(p) ^ code);
    if (d <= 8u || d >= 56u) {                                         /* first symbol: take the best of p..p+2 */
      unsigned bd = d < 32u ? d : 64u - d; p0 = (int32_t)p;
      for (uint32_t s = p + 1u; s <= p + 2u; s++) { unsigned e = d_pop64(d_get64(s) ^ code); e = e < 32u ? e : 64u - e; if (e < bd) { bd = e; p0 = (int32_t)s; } }
      break;
    }
  }
  if (p0 < 0 || (uint32_t)p0 + 128u * 64u > bits) return false;
  uint8_t pk[16]; memset(pk, 0, sizeof(pk));
  for (uint32_t k = 0; k < 128u; k++) if (d_pop64(d_get64((uint32_t)p0 + 64u * k) ^ code) < 32u) pk[k >> 3] |= (uint8_t)(1u << (k & 7u));
  if (memcmp(pk, pk + 4, 4) != 0) return false;
  uint16_t sum = 384u - 0x10u;
  for (int i = 0; i < 8; i++) sum += pk[i];
  if (pk[8] != (uint8_t)(sum >> 8) || pk[9] != (uint8_t)sum) return false;
  for (int i = 8; i < 14; i++) sum += pk[i];
  if (pk[14] != (uint8_t)(sum >> 8) || pk[15] != (uint8_t)sum) return false;
  for (int i = 0; i < 4; i++) d_id[i] = (uint8_t)~pk[i];
  d_nch = pk[11]; d_type = pk[12];
  d_last_p0 = p0;                                                   /* v1.6: preamble chips left after the sync point */
  return true;
}

/* ---- radio ---- */
static void d_apply_sync(void)
{
  sl_rail_sync_word_config_t sc = { .sync_word_bits = 32u, .sync_word_0 = DRX_SYNC_PRE, .sync_word_1 = DRX_SYNC_PRE };
  if (sl_rail_config_sync_words(d_rail, &sc) != SL_RAIL_STATUS_NO_ERROR) dsm_rx_diag[DR_CFG_ERR] |= 32;
}
static void d_ensure_sync(void)
{
  sl_rail_sync_word_config_t cur;
  if (sl_rail_get_sync_words(d_rail, &cur) != SL_RAIL_STATUS_NO_ERROR || cur.sync_word_0 != DRX_SYNC_PRE) { dsm_rx_diag[DR_SYNC_FIX]++; d_apply_sync(); }
}
static void d_listen_rf(uint8_t rf, uint16_t len)
{
  dsm_rx_diag[DR_CH] = rf;
  sl_rail_idle(d_rail, SL_RAIL_IDLE_ABORT, true);                  /* abort a frame in progress (a bind frame is 8 ms), keep held ones */
  d_ensure_sync();
  if (len != d_len) { if (sl_rail_set_fixed_length(d_rail, len) != len) dsm_rx_diag[DR_CFG_ERR] |= 2; d_len = len; }
  if (sl_rail_start_rx(d_rail, rf, NULL) != SL_RAIL_STATUS_NO_ERROR) dsm_rx_diag[DR_CFG_ERR] |= 8;
  d_open = true; d_arm_us = d_now();
}
static void d_listen(uint8_t idx)
{
  d_idx = idx; dsm_rx_diag[DR_HOP] = idx;
  d_listen_rf(d_rf_of(idx), DRX_DATA_BYTES);
}
static int16_t d_rssi(void)
{
  int16_t q = sl_rail_get_rssi(d_rail, SL_RAIL_GET_RSSI_NO_WAIT);
  return (q == SL_RAIL_RSSI_INVALID) ? (int16_t)-128 : (int16_t)(q / 4);
}


/* ---- bind response ---- */
static void d_resp_build(void)
{
  uint8_t pk[DRX_RESP_BYTES];
  uint16_t sum = 384u - 0x10u;
  for (int i = 0; i < 4; i++) pk[i] = (uint8_t)~d_id[i];
  pk[4] = 0x01; pk[5] = d_nch; pk[6] = d_type; pk[7] = 0x00;
  for (int i = 0; i < 8; i++) sum += pk[i];
  pk[8] = (uint8_t)(sum >> 8); pk[9] = (uint8_t)sum;
  uint64_t code = d_code64(drx_resp_code);
  memset(d_resp, 0, sizeof(d_resp));
  for (uint32_t k = 0; k < DRX_RESP_BYTES * 8u; k++) {            /* symbol k: the code for a 1, its inverse for a 0, LSB first */
    uint64_t sym = ((pk[k >> 3] >> (k & 7u)) & 1u) ? code : ~code;
    for (int i = 0; i < 8; i++) d_resp[k * 8u + (uint32_t)i] = (uint8_t)(sym >> (8 * i));
  }
}
static void d_resp_send(void)
{
  sl_rail_idle(d_rail, SL_RAIL_IDLE_ABORT, true);
  if (d_len != DRX_RESP_LEN) { if (sl_rail_set_fixed_length(d_rail, DRX_RESP_LEN) != DRX_RESP_LEN) dsm_rx_diag[DR_CFG_ERR] |= 2; d_len = DRX_RESP_LEN; }
  (void)sl_rail_write_tx_fifo(d_rail, d_resp, DRX_RESP_LEN, true);
  if (sl_rail_start_tx(d_rail, d_bind_ch, SL_RAIL_TX_OPTIONS_DEFAULT, NULL) != SL_RAIL_STATUS_NO_ERROR) dsm_rx_diag[DR_CFG_ERR] |= 64;
  d_resp_sent++;
}
static void d_resp_start(uint32_t now)
{
  d_state = 3; dsm_rx_diag[DR_STATE] = 3;
  d_resp_build();
  d_resp_next = now; d_resp_until = now + DRX_RESP_FOR_US; d_resp_sent = 0;
}

/* ---- v1.6 bind-burst profiler ---- */
static void __attribute__((unused)) d_prof_start(uint32_t t_sync, int8_t rssi, uint32_t now)
{
  d_state = 4; dsm_rx_diag[DR_STATE] = 4;
  p_pkt_n = 0; p_bad_n = 0; p_en_n = 0; p_en_on = false; p_valid = true;
  p_pkt_t[p_pkt_n++] = t_sync;
  p_p0_min = p_p0_max = p_p0_sum = d_last_p0; p_p0_n = 1;
  p_start_us = now; p_last_act = now;
  p_thresh = (int16_t)(rssi - DRX_PROF_REL_DB); if (p_thresh < DRX_BIND_RSSI) p_thresh = DRX_BIND_RSSI;
}
static void d_prof_close(void)
{
  p_en_on = false;
  if (p_en_n < DRX_PROF_N) { p_en_t[p_en_n] = p_en_start; p_en_len[p_en_n] = p_en_last - p_en_start; p_en_pk[p_en_n] = (int8_t)p_en_peak; }
  p_en_n++;
}
static void d_prof_sample(uint32_t now)
{
  int16_t dbm = d_rssi();
  if (dbm >= p_thresh) {
    if (!p_en_on) { p_en_on = true; p_en_start = now; p_en_peak = dbm; }
    else if (dbm > p_en_peak) p_en_peak = dbm;
    p_en_last = now; p_last_act = now;
  } else if (p_en_on && (now - p_en_last) > DRX_PROF_MERGE_US) d_prof_close();
}
static void d_prof_report(void)
{
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  uint32_t np = (p_pkt_n < DRX_PROF_N) ? p_pkt_n : DRX_PROF_N;
  uint32_t ne = (p_en_n < DRX_PROF_N) ? p_en_n : DRX_PROF_N;
  app_log("DR profile: %lu bind packets decoded (+%lu frames not decoded), first->last %lu ms, threshold %d dBm\r\n",
          (unsigned long)p_pkt_n, (unsigned long)p_bad_n, (unsigned long)((np > 1u) ? (p_pkt_t[np - 1u] - p_pkt_t[0]) / 1000u : 0u), (int)p_thresh);
  if (np > 1u) {
    uint32_t mn = 0xFFFFFFFFu, mx = 0u, sum = 0u;
    for (uint32_t i = 1; i < np; i++) { uint32_t d = p_pkt_t[i] - p_pkt_t[i - 1u]; if (d < mn) mn = d; if (d > mx) mx = d; sum += d; }
    app_log("DR profile: packet spacing us: min %lu avg %lu max %lu; first:", (unsigned long)mn, (unsigned long)(sum / (np - 1u)), (unsigned long)mx);
    for (uint32_t i = 1; i < np && i <= 12u; i++) app_log(" %lu", (unsigned long)(p_pkt_t[i] - p_pkt_t[i - 1u]));
    app_log("\r\n");
  }
  if (p_p0_n > 0)
    app_log("DR profile: preamble chips after the sync point: min %ld avg %ld max %ld (%ld packets)\r\n",
            (long)p_p0_min, (long)(p_p0_sum / p_p0_n), (long)p_p0_max, (long)p_p0_n);
  if (ne > 0u) {
    uint32_t mn = 0xFFFFFFFFu, mx = 0u, sum = 0u, cnt = 0u;
    for (uint32_t i = 1; i < ne; i++) {                              /* skip burst 0: it began before the profile did */
      uint32_t l = p_en_len[i]; if (l < 1000u) continue;              /* ignore short blips (not bind packets) */
      if (l < mn) mn = l;
      if (l > mx) mx = l;
      sum += l; cnt++;
    }
    app_log("DR profile: RF bursts: %lu, bind-sized %lu, length us: min %lu avg %lu max %lu\r\n", (unsigned long)p_en_n, (unsigned long)cnt,
            (unsigned long)(cnt ? mn : 0u), (unsigned long)(cnt ? sum / cnt : 0u), (unsigned long)(cnt ? mx : 0u));
    app_log("DR profile: bursts (start ms, length us, peak dBm):");
    for (uint32_t i = 0; i < ne && i < 12u; i++)
      app_log(" +%lu.%01lu/%lu/%d", (unsigned long)((p_en_t[i] - p_en_t[0]) / 1000u), (unsigned long)(((p_en_t[i] - p_en_t[0]) / 100u) % 10u),
              (unsigned long)p_en_len[i], (int)p_en_pk[i]);
    app_log("\r\n");
    uint32_t last_end = p_en_t[ne - 1u] + p_en_len[ne - 1u];
    app_log("DR profile: burst ended %lu ms after it started; answer started %lu ms after the last RF\r\n",
            (unsigned long)((last_end - p_en_t[0]) / 1000u), (unsigned long)(DRX_PROF_QUIET_US / 1000u));
  }
#endif
  p_valid = false;
}

/* ---- state changes ---- */
static void d_log_state(const char *s)
{
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("DR %s\r\n", s);
#else
  (void)s;
#endif
}
static void d_bind_start(uint32_t now)
{
  d_state = 0; dsm_rx_diag[DR_STATE] = 0;
  d_bind_ch = DRX_BIND_LO; d_bind_stay = false;
  d_listen_rf(d_bind_ch, DRX_BIND_BYTES);
  (void)now;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("DR bind: scanning %u..%u MHz for the bind burst (hold the TX module's bind button, switch the transmitter on)\r\n",
          2400u + DRX_BIND_LO, 2400u + DRX_BIND_HI);
#endif
}
static void d_search(uint32_t now)
{
  d_state = 1; dsm_rx_diag[DR_STATE] = 1; dsm_rx_diag[DR_SEARCHES]++;
  d_idx = 0; d_dwell_us = now; d_misses = 0;
  d_listen(d_idx);
  d_log_state("search");
}
static void d_failsafe(void)
{
  d_in_fs = true; dsm_rx_diag[DR_FS] = 1;
  if (d_rec.fs_valid) memcpy(d_out_us, d_rec.fs_us, sizeof(d_out_us));
  else { for (uint8_t i = 0; i < 8; i++) d_out_us[i] = 1500u; d_out_us[DRX_OUT_MAP[0]] = DRX_THR_FAILSAFE; }
}
static void d_store_fs(void)
{
  if (d_state != 2u || d_in_fs) return;
  if (d_rec.fs_valid && memcmp(d_rec.fs_us, d_out_us, sizeof(d_out_us)) == 0) return;
  memcpy(d_rec.fs_us, d_out_us, sizeof(d_rec.fs_us)); d_rec.fs_valid = 1;
  if (rx_store_save(RX_SLOT_DSM, &d_rec)) { d_saved = d_rec; dsm_rx_diag[DR_FS_STORE]++; }
}
static void d_adopt_record(void)
{
  memcpy(d_id, d_rec.hop, 4); d_type = d_rec.hop[4]; d_nch = d_rec.hop[5];
  d_setup_id();
}

/* ---- PPM + 8 servo pulses: servo_out.c (TIMER2) reads d_out_us ---- */
static void d_ppm_enable(bool on) { servo_out_enable(on); }
bool dsm_rx_fresh(uint32_t now, uint32_t within_us) { return d_last_rx != 0u && (now - d_last_rx) <= within_us; }

void dsm_rx_start(sl_rail_handle_t rh, bool bind_held)
{
  d_rail = rh;
  for (uint8_t i = 0; i < DRX_DIAG_N; i++) dsm_rx_diag[i] = 0;
  uint32_t now = (uint32_t)sl_rail_get_time(rh);
  for (uint8_t i = 0; i < 8; i++) d_out_us[i] = 1500u;
  d_out_us[DRX_OUT_MAP[0]] = DRX_THR_FAILSAFE;
  d_misses = 0; d_in_fs = false; d_btn_since = 0; d_btn_done = false; d_last_rx = 0;
  d_uart_us = now; d_chan_us = now; d_frame_us = (int32_t)DRX_FRAME_US; d_len = 0;
  d_btn_released = !bind_held;

  sl_gpio_set_pin_mode(&d_led_a, SL_GPIO_MODE_PUSH_PULL, 0);
  sl_gpio_set_pin_mode(&d_led_b, SL_GPIO_MODE_PUSH_PULL, 0);

  sl_rail_idle(rh, SL_RAIL_IDLE_ABORT, true);
  if (sl_rail_config_channels(rh, (const sl_rail_channel_config_t *)&dsm_rx_channelConfig, NULL) != SL_RAIL_STATUS_NO_ERROR) dsm_rx_diag[DR_CFG_ERR] |= 1;
  sl_rail_prepare_channel(rh, DRX_BIND_LO);                          /* apply the PHY now: later sync-word writes stick */
  uint16_t fsz = DRX_FIFO_BYTES;
  if (sl_rail_set_rx_fifo(rh, d_fifo, &fsz) != SL_RAIL_STATUS_NO_ERROR) dsm_rx_diag[DR_CFG_ERR] |= 4;
  d_apply_sync();
  if (sl_rail_set_tx_fifo(rh, d_txfifo, DRX_TX_FIFO_BYTES, 0, 0) != SL_RAIL_STATUS_NO_ERROR) dsm_rx_diag[DR_CFG_ERR] |= 4;
  sl_rail_config_events(rh, SL_RAIL_EVENT_RX_PACKET_RECEIVED | SL_RAIL_EVENTS_TX_COMPLETION, SL_RAIL_EVENT_RX_PACKET_RECEIVED | SL_RAIL_EVENTS_TX_COMPLETION);

#if DRX_SBUS
  sbus_init();
#endif
  servo_out_init(); servo_out_source(d_out_us);                     /* pulses: servo_out.c */

  d_have_saved = (rx_store_load(RX_SLOT_DSM, &d_saved) && d_saved.bound);
  if (bind_held) dsm_rx_diag[DR_CFG_ERR] |= 128;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("DR boot dsm_rx v1.8 (PHY dsm_rx 280 kHz, bind profiler, answer +25 ms for 3 s, SBUS %s) cfg_err=%ld fifo=%u stored=%d bind_held=%d\r\n", DRX_SBUS ? "on" : "off", (long)dsm_rx_diag[DR_CFG_ERR], (unsigned)fsz, d_have_saved ? 1 : 0, bind_held ? 1 : 0);
#endif
  if (bind_held || !d_have_saved) {
    d_binding = bind_held && d_have_saved;                           /* stored bind stays the fallback */
    d_bind_deadline = now + DRX_BIND_TIMEOUT_US;
    if (d_have_saved) d_rec = d_saved; else memset(&d_rec, 0, sizeof(d_rec));
    d_bind_start(now);
  } else {
    d_rec = d_saved; d_adopt_record();
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("DR stored id %02X %02X %02X %02X type %02X %u ch; channels:", d_id[0], d_id[1], d_id[2], d_id[3], d_type, (unsigned)d_nch);
    for (uint8_t i = 0; i < DRX_HOPS; i++) app_log(" %u", (unsigned)d_rf_of(i));
    app_log(" (MHz above 2400)\r\n");
#endif
    d_search(now);
  }
}

/* BIND held >300 ms while linked: store the current positions as the failsafe preset. */
static void d_button(bool bind_held, uint32_t now)
{
  if (!bind_held) d_btn_released = true;
  if (!bind_held || !d_btn_released || d_state == 0u) { d_btn_since = 0; d_btn_done = false; return; }
  if (d_btn_since == 0) { d_btn_since = now ? now : 1u; return; }
  if (d_btn_done || (now - d_btn_since) < DRX_FS_PRESS_US) return;
  d_btn_done = true; d_store_fs();
}

/* the timeline: the packet just received sat on hop d_idx at position pos; move to the next hop */
static void d_advance(uint32_t from, uint8_t pos)
{
  d_expect = from + ((pos == 0u) ? (uint32_t)DRX_PAIR_US : (uint32_t)(d_frame_us - (int32_t)DRX_PAIR_US));
  d_pos = (uint8_t)(pos ^ 1u);
  d_idx = (uint8_t)((d_idx + 1u) % DRX_HOPS);
}

void dsm_rx_service(bool bind_held)
{
#if DRX_SBUS
  sbus_service();
#endif
  uint32_t now = d_now();

  if (d_state == 0u) {                                              /* bind scan */
    if (d_binding && (int32_t)(now - d_bind_deadline) >= 0) {      /* nothing heard: back to the stored TX */
      d_binding = false; d_rec = d_saved; d_adopt_record(); d_search(now);
    } else {
      uint32_t stay = d_bind_stay ? DRX_BIND_STAY_US : DRX_BIND_PEEK_US;
      if (!d_bind_stay && (now - d_arm_us) > 300u && d_rssi() >= DRX_BIND_RSSI) d_bind_stay = true;
      if ((now - d_arm_us) >= stay) {
        d_bind_ch = (uint8_t)(d_bind_ch + 1u);
        if (d_bind_ch > DRX_BIND_HI) { d_bind_ch = DRX_BIND_LO; dsm_rx_diag[DR_SCANS]++; }
        d_bind_stay = false;
        d_listen_rf(d_bind_ch, DRX_BIND_BYTES);
      }
    }
  } else if (d_state == 3u) {                                       /* bind response: 10 ms cadence on the bind channel */
    if ((int32_t)(now - d_resp_until) >= 0) {
      sl_rail_idle(d_rail, SL_RAIL_IDLE_ABORT, true);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("DR bind response: %lu packets sent on %u MHz\r\n", (unsigned long)d_resp_sent, 2400u + d_bind_ch);
#endif
      if (p_valid) d_prof_report();                                 /* v1.6: the bind-burst profile, now that timing no longer matters */
      d_search(now);
    } else if ((int32_t)(now - d_resp_next) >= 0) { d_resp_next += DRX_RESP_EVERY_US; d_resp_send(); }
  } else if (d_state == 4u) {                                       /* v1.6: bind-burst profile, then answer */
    d_prof_sample(now);
    if ((now - p_last_act) >= DRX_PROF_QUIET_US || (now - p_start_us) >= DRX_PROF_MAX_US) {
      if (p_en_on) d_prof_close();
      d_resp_start(now);
    }
  } else if (d_state == 1u) {                                       /* search: dwell per channel */
    if ((now - d_dwell_us) >= DRX_SEARCH_DWELL_US) {
      d_idx = (uint8_t)((d_idx + 1u) % DRX_HOPS); d_dwell_us = now; d_listen(d_idx);
    }
  } else {                                                          /* link */
    if ((int32_t)(now - (d_expect + DRX_MISS_PAD_US)) >= 0) {       /* missed this hop */
      d_misses++; dsm_rx_diag[DR_MISS] = (int32_t)d_misses; dsm_rx_diag[DR_LOST]++;
      d_advance(d_expect, d_pos);
      d_listen(d_idx);
      if (d_misses == DRX_FS_MISSES) { d_failsafe(); d_log_state("lost: failsafe"); }
#if DRX_SBUS
      (void)sbus_send(d_out_us, true, d_in_fs);
#endif
      if (d_misses >= DRX_LOST_MISSES) d_search(now);
    }
  }
  d_button(bind_held, now);

  /* LED_B: bind fast blink, search slow blink, link steady.  LED_A: toggles per 10 packets, off without link. */
  static uint32_t tb = 0; static int32_t la = 0;
  if (d_state == 2u) sl_gpio_set_pin(&d_led_b);
  else if ((now - tb) > (d_state != 1u ? 100000u : 500000u)) { tb = now; sl_gpio_toggle_pin(&d_led_b); }
  if (d_state != 2u) sl_gpio_clear_pin(&d_led_a);
  else { int32_t a = dsm_rx_diag[DR_PKT] / 10; if (a != la) { la = a; sl_gpio_toggle_pin(&d_led_a); } }

#if DRX_UART_STATUS && defined(SL_CATALOG_APP_LOG_PRESENT)
  if ((now - d_uart_us) >= DRX_UART_US && (!DRX_SBUS || sbus_idle())) {   /* ~40 chars = 3.5 ms; only while SBUS is quiet */
    d_uart_us = now;
    static int32_t ppk = 0, plost = 0;
    if (d_state == 0u)
      app_log("DR s0 scan ch=%u sweeps=%ld rx=%ld\r\n", (unsigned)d_bind_ch, (long)dsm_rx_diag[DR_SCANS], (long)dsm_rx_diag[DR_RX]);
    else if (d_state == 3u)
      app_log("DR s3 responding sent=%lu\r\n", (unsigned long)d_resp_sent);
    else if (d_state == 4u) { }                                      /* profiling: no UART, the RSSI sampling must not stall */
    else
      app_log("DR s%d r%ld c%ld n%ld m%ld l%ld h%d q%ld f%d\r\n", (int)d_state, (long)(dsm_rx_diag[DR_PKT] - ppk),
              (long)dsm_rx_diag[DR_CRC_BAD], (long)dsm_rx_diag[DR_NOSOP], (long)d_misses, (long)(dsm_rx_diag[DR_LOST] - plost), (int)d_idx,
              (long)dsm_rx_diag[DR_RSSI], d_in_fs ? 1 : 0);
    ppk = dsm_rx_diag[DR_PKT]; plost = dsm_rx_diag[DR_LOST];
  }
  if (d_state == 2u && (now - d_chan_us) >= 1000000u && (!DRX_SBUS || sbus_idle())) {
    d_chan_us = now;
    app_log("DR ch %u %u %u %u %u %u %u %u  q=%ld P=%ld e=%ld\r\n", d_out_us[0], d_out_us[1], d_out_us[2], d_out_us[3],
            d_out_us[4], d_out_us[5], d_out_us[6], d_out_us[7], (long)dsm_rx_diag[DR_Q], (long)d_frame_us, (long)d_last_err);
  }
#endif
}

static inline uint16_t d_us(uint16_t v) { return (uint16_t)(1500 + ((int32_t)v - 1024) * 1200 / 2046); }   /* 1024 -> 1500, 342..1706 -> 1100..1900 */

/* a good packet on hop d_idx at time t (sync end), pair position pos: outputs and the timeline */
static void d_good(uint32_t t, uint8_t pos, bool hopped)
{
  dsm_rx_diag[DR_PKT]++;
  for (int i = 0; i < 7; i++) {
    uint16_t w = (uint16_t)((d_bytes[3 + 2 * i] << 8) | d_bytes[4 + 2 * i]);
    uint8_t cid = (uint8_t)((w >> 11) & 15u);
    if (cid < 8u) d_out_us[DRX_OUT_MAP[cid]] = d_us((uint16_t)(w & 0x7FFu));
  }
  if (d_state == 2u) {                                              /* track the frame period from the timing error */
    int32_t err = (int32_t)(t - d_expect); d_last_err = err;
    if (pos != d_pos) { d_log_state("pair position resync"); }
    else if (err > -1500 && err < 1500) {
      int32_t nom = d_nominal_us();                                 /* v1.4: clamp around 11 or 22 ms by type */
      d_frame_us += err / 8; if (d_frame_us < nom - 100) d_frame_us = nom - 100; if (d_frame_us > nom + 100) d_frame_us = nom + 100;
    }
  }
  d_last_rx = (uint32_t)sl_rail_get_time(d_rail);
  if (d_state != 2u) {
    dsm_rx_diag[DR_LINKS]++; d_ppm_enable(true);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("DR link idx=%u (%u MHz) pos=%u q=%ld\r\n", (unsigned)d_idx, 2400u + d_rf_of(d_idx), (unsigned)pos, (long)dsm_rx_diag[DR_Q]);
#endif
  }
  d_state = 2; dsm_rx_diag[DR_STATE] = 2;
  d_in_fs = false; dsm_rx_diag[DR_FS] = 0; d_misses = 0; dsm_rx_diag[DR_MISS] = 0;
  d_advance(t, pos);                                                 /* d_idx -> next hop, d_expect, d_pos */
  if (!hopped) d_listen(d_idx);
#if DRX_SBUS
  if (sbus_send(d_out_us, false, false)) dsm_rx_diag[DR_SBUS]++;
#endif
}

void dsm_rx_process(void)
{
  for (;;) {
    sl_rail_rx_packet_info_t info;
    sl_rail_rx_packet_handle_t h = sl_rail_get_rx_packet_info(d_rail, SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &info);
    if (h == SL_RAIL_RX_PACKET_HANDLE_INVALID) return;
    uint16_t n = info.packet_bytes; if (n > DRX_BIND_BYTES) n = DRX_BIND_BYTES;
    sl_rail_rx_packet_details_t det; memset(&det, 0, sizeof(det));
    det.time_received.time_position = SL_RAIL_PACKET_TIME_DEFAULT;
    (void)sl_rail_get_rx_packet_details(d_rail, h, &det);
    det.time_received.total_packet_bytes = info.packet_bytes;
    (void)sl_rail_get_rx_time_sync_word_end(d_rail, &det);
    info.packet_bytes = n;
    memset(d_w, 0, sizeof(d_w));
    sl_rail_copy_rx_packet(d_rail, (uint8_t *)d_w, &info);
    sl_rail_release_rx_packet(d_rail, h);
    dsm_rx_diag[DR_RX]++; dsm_rx_diag[DR_RSSI] = det.rssi_dbm;
    uint32_t t = (uint32_t)det.time_received.packet_time;
    uint32_t bits = (uint32_t)n * 8u;

    if (d_state == 3u) continue;                                    /* responding: nothing to receive */
    if (d_state == 4u) {                                            /* v1.6 profile: time every further bind packet */
      uint8_t sid[4]; memcpy(sid, d_id, 4); uint8_t st = d_type, sn = d_nch;
      bool ok = d_decode_bind(bits) && memcmp(sid, d_id, 4) == 0;
      memcpy(d_id, sid, 4); d_type = st; d_nch = sn;               /* the bound id stays as it is */
      if (ok) {
        if (p_pkt_n < DRX_PROF_N) p_pkt_t[p_pkt_n] = t;
        p_pkt_n++; p_last_act = d_now();
        if (d_last_p0 < p_p0_min) p_p0_min = d_last_p0;
        if (d_last_p0 > p_p0_max) p_p0_max = d_last_p0;
        p_p0_sum += d_last_p0; p_p0_n++;
      } else p_bad_n++;
      continue;
    }
    if (d_state == 0u) {                                            /* bind: a bind packet gives the id */
      if (!d_decode_bind(bits)) continue;
      dsm_rx_diag[DR_BINDS]++;
      if (!d_have_saved) memset(&d_rec, 0, sizeof(d_rec));
      memcpy(d_rec.hop, d_id, 4); d_rec.hop[4] = d_type; d_rec.hop[5] = d_nch; d_rec.id2 = d_id[2]; d_rec.id3 = d_id[3]; d_rec.bound = 1;
      d_rec.fs_valid = 0;                                           /* a new bind: default failsafe until stored again */
      bool stored = rx_store_save(RX_SLOT_DSM, &d_rec);
      if (stored) { d_saved = d_rec; d_have_saved = true; }
      d_binding = false;
      d_setup_id();
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("DR bound: id %02X %02X %02X %02X type %02X %u ch on %u MHz rssi %d (%s); channels:", d_id[0], d_id[1], d_id[2], d_id[3],
              d_type, (unsigned)d_nch, 2400u + d_bind_ch, (int)det.rssi_dbm, stored ? "stored" : "STORE FAILED");
      for (uint8_t i = 0; i < DRX_HOPS; i++) app_log(" %u", (unsigned)d_rf_of(i));
      app_log(" (MHz above 2400)\r\n");
      if ((d_type & 0xF0u) != 0xB0u && (d_type & 0xF0u) != 0xA0u) app_log("DR type %02X is not DSMX - this role follows DSMX only\r\n", d_type);
#endif
#if DRX_PROFILE
      d_prof_start(t, det.rssi_dbm, d_now());                       /* v1.6: time the rest of the burst, answer once it is quiet */
#else
      d_resp_start(d_now());                                        /* answer the transmitter, then search */
#endif
      return;
    }

    /* data: our SOP for this channel's row, then the symbols and the CRC */
    const drx_row_t *row = &d_row[d_row_of(d_idx)];
    int32_t sop = d_find_sop(bits, row);
    if (sop < 0) { dsm_rx_diag[DR_NOSOP]++; continue; }
    bool hopped = false;
    uint8_t listened = d_idx;
    if (d_state == 2u) {                                            /* ours: move to the next channel first, decode after */
      d_listen((uint8_t)((d_idx + 1u) % DRX_HOPS)); d_idx = listened; hopped = true;
    }
    uint8_t q = 0;
    if (!d_decode_syms(bits, row, (uint32_t)sop, &q)) { dsm_rx_diag[DR_NOSOP]++; if (hopped) d_advance(t, d_pos); continue; }
    uint8_t pos = d_check_crc();
    if (pos == 2u) {
      dsm_rx_diag[DR_CRC_BAD]++;
      if (hopped) d_advance(t, d_pos);                              /* keep the timeline on the expected position */
      continue;
    }
    dsm_rx_diag[DR_Q] = q; dsm_rx_diag[DR_SOP_AT] = sop;
    d_good(t, pos, hopped);
  }
}
