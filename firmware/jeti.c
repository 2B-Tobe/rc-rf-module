/* jeti.c — see jeti.h.  #included into app_process.c.   v0.5 (EXPERIMENTAL)
 * The framing (header, scrambler, hop list, channel slots, the bind sequence) follows mpm-v2's src/core/jeti_frame.c
 * and src/mpm/proto_jeti.c by Raphael Coeffic (GPL v3); checked against captures of a Jeti TU2 module. */
#include "jeti.h"
#include <string.h>

#define JETI_HDR          7u              /* 01 08 seq PAN(2) dst(2) */
#define JETI_RING         5u
#define JETI_CHANNELS     16u             /* IEEE 11..26 = 2405 + 5 n MHz */
#define JETI_STEP_US      10000u
#define JETI_PLACE_EARLY  3u
#define JETI_PLACE_MIDDLE 2u
#define JETI_PLACE_LATE   1u
#define JETI_TYPE_TX      0xB1u
#define JETI_ID_BYTE      0x58u
#define JETI_PSDU_MAX     127u

/* ------------------------------------------------------------------ the pure pieces (host-testable) */
static uint8_t jq_key(uint8_t k, uint8_t p) { return (uint8_t)(((uint8_t)((k << 1) + p) ^ ((k & 0x80u) ? 0xC8u : 0u)) ^ 0x93u); }

void jeti_scramble(const uint8_t *plain, uint8_t *out, uint8_t n)
{
  uint8_t k = 0;
  for (uint8_t i = 0; i < n; i++) { uint8_t p = plain[i]; out[i] = (i <= JETI_HDR) ? p : (uint8_t)(p ^ k); k = jq_key(k, p); }
}

void jeti_descramble(const uint8_t *in, uint8_t *out, uint8_t n)
{
  uint8_t k = 0;
  for (uint8_t i = 0; i < n; i++) { uint8_t p = (i <= JETI_HDR) ? in[i] : (uint8_t)(in[i] ^ k); out[i] = p; k = jq_key(k, p); }
}

uint16_t jeti_slot(uint16_t us)
{
  int32_t v = ((int32_t)us - 1500) * 8 - 1;
  if (v < -8192) v = -8192;
  if (v > 8191) v = 8191;
  return (uint16_t)((uint32_t)v & 0x3FFFu);
}

uint16_t jeti_slot_us(uint16_t slot)
{
  int32_t v = (int32_t)(slot & 0x3FFFu);
  if (v & 0x2000) v -= 0x4000;
  int32_t us = 1500 + (v + 1) / 8;
  if (us < 800) us = 800;
  if (us > 2200) us = 2200;
  return (uint16_t)us;
}

static uint8_t *jq_header(uint8_t *f, uint8_t seq, uint16_t pan, uint16_t dst)
{
  f[0] = 0x01; f[1] = 0x08; f[2] = seq; f[3] = (uint8_t)pan; f[4] = (uint8_t)(pan >> 8); f[5] = (uint8_t)dst; f[6] = (uint8_t)(dst >> 8);
  return f + JETI_HDR;
}

/* the ring rotated: the next step's channel first, the current step's last; the place in bits 5-6 of the first byte */
static uint8_t *jq_list(uint8_t *p, const uint8_t *ring, uint8_t si, uint8_t place)
{
  uint8_t l[JETI_RING];
  for (uint8_t j = 0; j < JETI_RING; j++) l[j] = ring[(si + 1u + j) % JETI_RING] & 0x0Fu;
  *p++ = (uint8_t)((place << 5) | l[0]);
  *p++ = (uint8_t)((l[1] << 4) | l[2]);
  *p++ = (uint8_t)((l[3] << 4) | l[4]);
  return p;
}

static uint8_t *jq_slots(uint8_t *p, const uint16_t *us, uint8_t n)
{
  *p++ = (uint8_t)(0x30u | (n - 1u));
  for (uint8_t i = 0; i < n; i++) { uint16_t v = jeti_slot(us[i]); *p++ = (uint8_t)v; *p++ = (uint8_t)(v >> 8); }
  return p;
}

/* A data frame (0x21), plain, without the FCS. Returns its length. */
static uint8_t jeti_data_frame(uint8_t *f, uint8_t seq, uint16_t dst_pan, uint16_t dst_short, uint16_t own_short,
                        const uint8_t *ring, uint8_t si, uint8_t place, uint8_t count, const uint16_t *us)
{
  uint8_t *p = jq_header(f, seq, dst_pan, dst_short);
  *p++ = JETI_TYPE_TX; *p++ = 0x21;
  *p++ = (uint8_t)own_short; *p++ = (uint8_t)(own_short >> 8); *p++ = JETI_ID_BYTE;
  p = jq_list(p, ring, si, place);
  *p++ = 0x11; *p++ = (uint8_t)((count & 3u) << 6); *p++ = 0xB6;
  p = jq_slots(p, us, 8u);
  return (uint8_t)(p - f);
}

/* A bind frame to the broadcast address: boot = 0x23 (addresses and the list), else 0x22 (the same + channel data). */
static uint8_t jeti_bind_frame(uint8_t *f, uint8_t seq, bool boot, uint16_t own_pan, uint16_t own_short, uint8_t bind_ch, const uint16_t *us)
{
  uint8_t ring[JETI_RING]; memset(ring, bind_ch, sizeof(ring));
  uint8_t *p = jq_header(f, seq, 0xFFFFu, 0xFFFFu);
  *p++ = JETI_TYPE_TX;
  if (boot) { *p++ = 0x23; *p++ = 0x01; *p++ = 0x00; } else *p++ = 0x22;
  *p++ = (uint8_t)own_pan; *p++ = (uint8_t)(own_pan >> 8);
  *p++ = (uint8_t)own_short; *p++ = (uint8_t)(own_short >> 8); *p++ = JETI_ID_BYTE;
  p = jq_list(p, ring, 0, JETI_PLACE_MIDDLE);
  if (!boot) { *p++ = 0x11; *p++ = 0x80; *p++ = 0x00; p = jq_slots(p, us, 8u); }
  return (uint8_t)(p - f);
}

typedef struct {
  uint8_t  sub;                 /* 0x21 or 0x22 */
  uint16_t tx_short, tx_pan;    /* tx_pan: 0x22 only (else 0) */
  uint8_t  l[JETI_RING];        /* l[4]: this step's channel, l[0]: the next step's */
  uint8_t  place;
  uint8_t  n;                   /* slots */
  uint16_t slot[16];
} jeti_txframe_t;

/* A transmitter's frame with channel data (plain PSDU without the FCS). */
static bool jeti_parse(const uint8_t *f, uint8_t len, jeti_txframe_t *o)
{
  if (len < JETI_HDR + 2u || f[0] != 0x01 || f[1] != 0x08 || (f[JETI_HDR] & 0x0Fu) != 1u) return false;
  uint8_t sub = f[JETI_HDR + 1u], a;                        /* a: the list's first byte */
  if (sub == 0x21) { a = 12u; o->tx_pan = 0; o->tx_short = (uint16_t)(f[9] | ((uint16_t)f[10] << 8)); }
  else if (sub == 0x22) { a = 14u; if (len < 13u) return false;
                          o->tx_pan = (uint16_t)(f[9] | ((uint16_t)f[10] << 8)); o->tx_short = (uint16_t)(f[11] | ((uint16_t)f[12] << 8)); }
  else return false;
  if (len < a + 7u || f[a - 1u] != JETI_ID_BYTE || f[a + 3u] != 0x11 || (f[a + 6u] & 0xF0u) != 0x30u) return false;
  o->sub = sub;
  o->place = (uint8_t)((f[a] >> 5) & 3u);
  o->l[0] = f[a] & 0x0Fu; o->l[1] = f[a + 1u] >> 4; o->l[2] = f[a + 1u] & 0x0Fu; o->l[3] = f[a + 2u] >> 4; o->l[4] = f[a + 2u] & 0x0Fu;
  o->n = (uint8_t)((f[a + 6u] & 0x0Fu) + 1u);
  if (len < a + 7u + 2u * o->n) return false;
  for (uint8_t i = 0; i < o->n; i++) o->slot[i] = (uint16_t)((f[a + 7u + 2u * i] | ((uint16_t)f[a + 8u + 2u * i] << 8)) & 0x3FFFu);
  return true;
}

/* id -> PAN, short address and the ring of 5 channels (as mpm-v2) */
static void jeti_from_id(uint32_t id, uint16_t *pan, uint16_t *sh, uint8_t *ring)
{
  *pan = (uint16_t)(0xA000u | ((id >> 16) & 0x0FFFu));
  *sh = (uint16_t)(id & 0xFFFFu);
  if (*sh == 0u || *sh >= 0xFFFEu) *sh ^= 0x5A5Au;
  uint32_t r = id * 2654435761u;
  for (uint8_t k = 0; k < JETI_RING;) {
    r = r * 1103515245u + 12345u;
    uint8_t c = (uint8_t)((r >> 20) & 0x0Fu), j;
    for (j = 0; j < k && ring[j] != c; j++) {}
    if (j == k) ring[k++] = c;
  }
}

#ifndef JETI_HOST_TEST
/* ------------------------------------------------------------------ the radio */
#include "sl_core.h"
#include "sl_rail_ieee802154.h"
#include "rx_store.h"
#include "servo_out.h"
#include "sbus_out.h"

#define JQ_VERSION           "v0.5"
#define JQ_TX_POWER_DDBM     200
#define JQ_RANGE_POWER_DDBM  (-100)
#define JQ_LOG_US            5000000u
#define JQ_TX_FIFO_BYTES     128u
#define JQ_CCA_DBM           (-70)        /* at and above: busy (mpm-v2: EN 300 328's threshold at 20 dBm) */
#define JQ_CCA_LEAD_US       130u         /* RAIL: check 128 us + turnaround 192 us; mpm-v2 sends 450 us after the check's start */

/* transmitter: the times are mpm-v2's (proto_jeti.c), from the start of a 10 ms step */
#define JT_TX_AT_US          4750u        /* the frame's start; its SFD 150 us later = the "middle" place */
#define JT_CCA_TX_US         450u         /* from a clear-channel check to the frame's start */
#define JT_CCA_AT_US         (JT_TX_AT_US - JT_CCA_TX_US)
#define JT_MEAN_US           4400         /* a step's mean frame time (SFD) */
#define JT_MIDDLE_FROM_US    (-350)       /* SFD from the mean: below = early, */
#define JT_LATE_FROM_US      1600         /* ... below = middle, above = late, */
#define JT_WINDOW_END_US     4000         /* ... the latest a frame is sent in its step */
#define JT_EARLY_US          (-1200)
#define JT_BACKOFF_US        320u         /* 802.15.4's unit: a random 0..2^BE-1 of them before each check */
#define JT_MIN_BE            1u
#define JT_MAX_BE            3u
#define JT_LEAD_US           700u         /* the frame is prepared this long before its first check */
#define JT_MIN_SLACK_US      250u
#define JT_SWEEP_FRAMES      12u          /* 240 ms: one ring slot moves to another channel */
#define JT_BAD_TICKS         40u          /* ~10 s: a channel with a frame skipped or unanswered is passed over */
#define JT_LQI_FRAMES        50u
#define JT_BIND_DWELL        15u          /* steps per channel at bind */
#define JT_BIND_BOOT         5u           /* 0x23 frames first on each channel */
#define JT_BIND_TIMEOUT_US   60000000u
#define JT_LINK_US           1000000u     /* a reply within this: linked */

/* receiver */
#define JR_DWELL_US          220000u      /* search: per channel (TU2 captures on one ring channel: a frame every 100 ms, gaps to 200 ms) */
#define JR_MEAN_US           4400u        /* a step's mean frame time, from the step's start (the step ends 5.6 ms after it) */
#define JR_RESYNC_US         2500         /* a frame this far off the hop clock: the clock is set anew */
#define JR_SWITCH_US         500u         /* the channel changes this long after the step's start */
#define JR_UNLINK_US         500000u      /* without a frame: search again */
#define JR_FS_US             1000000u     /* without a frame: failsafe */
#define JR_REPLY_AT_US       2500u        /* after the frame's SFD: the reply's clear-channel check (reply SFD ~3.0 ms, a Jeti receiver's 2.8..3.4) */
#define JR_REPLY_HOLD_US     6000u        /* after the frame's SFD: stay on its channel (the reply) */
#define JR_BIND_RSSI_DBM     (-70)
#define JR_BIND_FRAMES       10u
#define JR_BIND_TIMEOUT_US   8000000u
#define JR_FS_PRESS_US       300000u
#define JR_TRIM_US           0            /* added to every channel at the outputs (a Jeti module fed 1500 us sends slot -1 = 1500 us: no trim needed) */
static const int16_t jr_place_us[4] = { 0, 2600, 500, -1200 };   /* SFD from the step's mean, by the place bits: 1 late, 2 middle, 3 early */

static sl_rail_handle_t jq_rail;
static const volatile uint16_t *jq_in;
static SL_RAIL_DECLARE_FIFO_BUFFER(jq_txfifo, JQ_TX_FIFO_BYTES);
static bool     jq_run, jq_rx, jq_input, jq_range;
static volatile bool jq_pwr_dirty, jq_pwr_log, jq_ovf;
static uint16_t jq_pan, jq_short;         /* ours (either role) */
static uint32_t jq_uart_us;
static int32_t  jq_cfg_err;               /* 1 init 2 phy 4 tx fifo 8 rx options 16 events */
static sl_gpio_t jq_led_a = { .port = SL_GPIO_PORT_C, .pin = 4 };
static sl_gpio_t jq_led_b = { .port = SL_GPIO_PORT_C, .pin = 5 };
static const sl_rail_csma_config_t jq_cca = { .csma_min_bo_exp = 0, .csma_max_bo_exp = 0, .csma_tries = 1, .cca_threshold_dbm = JQ_CCA_DBM,
                                              .cca_backoff_us = 0, .cca_duration_us = 128, .csma_timeout_us = 0 };     /* one check, no wait of RAIL's own (v0.3: 320 = a fixed 320 us wait before every check) */

/* transmitter state: everything below is touched from the RAIL timer and the RAIL events (interrupts) */
static uint8_t  jt_ring[JETI_RING];
static uint8_t  jt_miss[JETI_RING];       /* per ring slot: frames unanswered since it moved */
static uint32_t jt_moved[JETI_RING];      /* ... and the frame count when it moved */
static uint8_t  jt_bad[JETI_CHANNELS];    /* sweep ticks left */
static volatile uint8_t  jt_si, jt_seq, jt_bind_ch, jt_dwell, jt_last_ch;
static volatile bool     jt_bind;
static volatile uint8_t  jt_stage;        /* 0 idle, 1 a clear-channel check / frame under way, 2 sent: the reply awaited */
static volatile bool     jt_got;          /* a frame came after ours */
static volatile uint32_t jt_step_t;       /* start of the step the next frame is planned in */
static volatile uint32_t jt_n, jt_sent, jt_busy, jt_err, jt_skipped, jt_late, jt_next, jt_lost, jt_moves;
static uint8_t  jt_plain[JETI_PSDU_MAX], jt_len;   /* the frame under way */
static uint32_t jt_base;                  /* start of the step it was planned in */
static uint16_t jt_u;                     /* its check's time from there */
static uint8_t  jt_slide, jt_be, jt_place, jt_slot, jt_tx_slot;
static bool     jt_data;                  /* the frame under way is a data frame */
static uint32_t jt_rnd;
static uint8_t  jt_lqi_frames, jt_lqi_replies, jt_lqi_sent;
static volatile uint8_t jt_lqi_rx = 0, jt_lqi_tx = 0;   /* %: frames answered / frames sent (last 50) */
static uint16_t jt_rx_pan = 0xFFFFu, jt_rx_short = 0xFFFFu;
static uint32_t jt_replies, jt_bind_since, jt_last_sent, jt_last_replies, jt_reply_t;
static uint16_t jt_volt;                  /* the receiver's voltage, 10 mV */
static uint8_t  jt_signal;                /* the receiver's signal level byte */
static int8_t   jt_rssi;
static bool     jt_bind_armed;

/* receiver state */
static uint8_t  jr_state;                 /* 0 bind, 1 search, 2 link */
static bool     jr_have, jr_binding, jr_fs_valid, jr_in_fs, jr_out_on, jr_btn_done, jr_btn_armed, jr_quiet;
static uint16_t jr_tx, jr_tx_pan, jr_cand, jr_cand_pan;   /* the transmitter's short address and PAN (0 = not known: no reply) */
static uint8_t  jr_hits, jr_ch, jr_l[JETI_RING];
static uint32_t jr_mref;                  /* a step's mean frame time (the hop clock's phase) */
static uint32_t jr_step0;                 /* start of the step of the last frame */
static uint32_t jr_hold;                  /* stay on the channel until then (the reply), 0 = no */
static uint32_t jr_dwell_us, jr_bind_since, jr_last_rx, jr_last_t, jr_btn_since, jr_sbus_us, jr_frames, jr_other, jr_resyncs;
static uint32_t jr_reply_n, jr_reply_late;
static volatile uint32_t jr_reply_sent, jr_reply_busy;
static uint16_t jr_out_us[8], jr_fs_us[8];
static int8_t   jr_rssi;
static uint8_t  jr_ftype, jr_fsub;        /* the last frame of our transmitter: type byte, sub, and where it was sent to */
static uint16_t jr_fpan, jr_fdst;
static uint32_t jr_n21, jr_n22;           /* its 0x21 / 0x22 frames */
static rx_record_t jr_rec;

#if defined(SL_CATALOG_APP_LOG_PRESENT)
#define JQ_TRACE(...) app_log(__VA_ARGS__)
#else
#define JQ_TRACE(...) do {} while (0)
#endif
static volatile uint8_t jq_first;          /* start-up trace: bit 0 first frame prepared, 1 sent, 2 busy, 3 failed; bits 4-7 the same, printed */
static inline uint32_t jq_now(void) { return (uint32_t)sl_rail_get_time(jq_rail); }
static void jr_start(bool bind_held);
static void jt_timer(sl_rail_handle_t rh);

static uint32_t jq_chip_id(void)
{
  uint64_t u = ((uint64_t)DEVINFO->EUI64H << 32) | DEVINFO->EUI64L;
  return (uint32_t)(u ^ (u >> 16) ^ (u >> 32) ^ (u >> 48));
}

/* PHR + the PSDU (scrambled here) into the TX FIFO, then one clear-channel check at `when` and the frame; a time
 * already past: at once */
static sl_rail_status_t jq_send(sl_rail_handle_t rh, const uint8_t *plain, uint8_t len, uint8_t ch, uint32_t when, bool cca)
{
  static uint8_t f[1u + JETI_PSDU_MAX];
  jeti_scramble(plain, f + 1, len);
  f[0] = (uint8_t)(len + 2u);                                 /* RAIL appends the FCS */
  (void)sl_rail_write_tx_fifo(rh, f, (uint16_t)(len + 1u), true);
  sl_rail_scheduled_tx_config_t sc = { .when = when, .mode = SL_RAIL_TIME_ABSOLUTE, .tx_during_rx = SL_RAIL_SCHEDULED_TX_DURING_RX_POSTPONE_TX };
  sl_rail_status_t s;
  if (cca) {
    s = sl_rail_start_scheduled_cca_csma_tx(rh, (uint16_t)(11u + ch), SL_RAIL_TX_OPTIONS_DEFAULT, &sc, &jq_cca, NULL);
    if (s != SL_RAIL_STATUS_NO_ERROR) s = sl_rail_start_cca_csma_tx(rh, (uint16_t)(11u + ch), SL_RAIL_TX_OPTIONS_DEFAULT, &jq_cca, NULL);
  } else {
    s = sl_rail_start_scheduled_tx(rh, (uint16_t)(11u + ch), SL_RAIL_TX_OPTIONS_DEFAULT, &sc, NULL);
    if (s != SL_RAIL_STATUS_NO_ERROR) s = sl_rail_start_tx(rh, (uint16_t)(11u + ch), SL_RAIL_TX_OPTIONS_DEFAULT, NULL);
  }
  return s;
}

/* ============================== the transmitter role ============================== */
static uint16_t jt_ch_us(uint8_t i)                           /* the input, or centre sticks with the throttle low */
{
  if (!jq_input) return (i == 2u) ? 1000u : 1500u;
  uint16_t v = jq_in[i];
  return (v < 800u) ? 800u : (v > 2200u) ? 2200u : v;
}

static uint16_t jt_backoff(void)
{
  jt_rnd = jt_rnd * 1103515245u + 12345u;
  return (uint16_t)(((jt_rnd >> 16) & ((1u << jt_be) - 1u)) * JT_BACKOFF_US);
}

/* the frame's SFD from its step's mean frame time, were its check at jt_u clear */
static int32_t jt_offset(void) { return (int32_t)jt_u + (int32_t)JT_CCA_TX_US + 150 - JT_MEAN_US - (int32_t)jt_slide * (int32_t)JETI_STEP_US; }

static void jt_set_place(void)                                /* the place bits from the send time; after a slide the list from the new slot */
{
  int32_t off = jt_offset();
  uint8_t place = off < JT_MIDDLE_FROM_US ? JETI_PLACE_EARLY : off < JT_LATE_FROM_US ? JETI_PLACE_MIDDLE : JETI_PLACE_LATE;
  if (place == jt_place && !jt_slide) return;
  jt_place = place;
  (void)jq_list(jt_plain + JETI_HDR + 5u, jt_ring, jt_slot, place);
}

static void jt_lqi(bool sent, bool reply)
{
  jt_lqi_sent += sent ? 1u : 0u; jt_lqi_replies += reply ? 1u : 0u;
  if (++jt_lqi_frames >= JT_LQI_FRAMES) {
    jt_lqi_rx = (uint8_t)(jt_lqi_replies * 100u / JT_LQI_FRAMES); jt_lqi_tx = (uint8_t)(jt_lqi_sent * 100u / JT_LQI_FRAMES);
    jt_lqi_frames = 0; jt_lqi_replies = 0; jt_lqi_sent = 0;
  }
}

static void jt_miss_now(void)                                 /* the last data frame: skipped (busy) or not answered */
{
  if (jt_miss[jt_tx_slot] < 0xFFu) jt_miss[jt_tx_slot]++;
  jt_bad[jt_ring[jt_tx_slot]] = JT_BAD_TICKS;
}

/* The channel sweep, as mpm-v2's (and the Jeti module's): one slot - not this frame's nor the next one's - moves to
 * the next channel above its own that is not in the ring and not marked bad: the slot with the most frames
 * unanswered since it moved, else the one unmoved longest. */
static void jt_sweep(void)
{
  uint8_t next = (uint8_t)((jt_si + 2u) % JETI_RING), best = 0xFFu;
  for (uint8_t j = 0; j < JETI_RING; j++) {
    if (j == jt_si || j == next) continue;
    if (best == 0xFFu || jt_miss[j] > jt_miss[best] || (jt_miss[j] == jt_miss[best] && jt_moved[j] < jt_moved[best])) best = j;
  }
  uint8_t c = jt_ring[best], any = 0xFFu;
  for (uint8_t k = 1; k < JETI_CHANNELS; k++) {
    uint8_t t = (uint8_t)((c + k) % JETI_CHANNELS), j;
    for (j = 0; j < JETI_RING && jt_ring[j] != t; j++) {}
    if (j < JETI_RING) continue;
    if (any == 0xFFu) any = t;
    if (!jt_bad[t]) { c = t; break; }
  }
  if (c == jt_ring[best]) c = any;
  jt_ring[best] = c;
  for (uint8_t k = 0; k < JETI_CHANNELS; k++) if (jt_bad[k]) jt_bad[k]--;
  jt_miss[best] = 0; jt_moved[best] = jt_n; jt_moves++;
}

static void jt_attempt(sl_rail_handle_t rh)                   /* the data frame's next clear-channel check, then the frame */
{
  jt_stage = 1u; jt_got = false; jq_first |= 1u;
  jt_last_ch = jt_ring[jt_slot];
  if (jq_send(rh, jt_plain, jt_len, jt_last_ch, jt_base + jt_u + JQ_CCA_LEAD_US, true) != SL_RAIL_STATUS_NO_ERROR) { jt_err++; jt_stage = 0u; }
}

/* The next frame: prepared JT_LEAD_US before its first check, from the RAIL timer (interrupt). The step grid runs by
 * the clock: a frame that cannot be met is skipped with its step, so a receiver finds us where it expects. */
static void jt_timer(sl_rail_handle_t rh)
{
  if (!jq_run || jq_rx) return;
  uint32_t now = (uint32_t)sl_rail_get_time(rh);
  if (jt_stage == 2u && jt_data) { jt_lqi(true, jt_got); if (!jt_got) jt_miss_now(); }   /* the last frame's account */
  else if (jt_stage == 1u) { sl_rail_idle(rh, SL_RAIL_IDLE_ABORT, true); jt_err++; }
  jt_stage = 0u;
  uint8_t steps = jt_bind ? 1u : 2u;
  while ((int32_t)(jt_step_t + JT_CCA_AT_US - now) < (int32_t)JT_MIN_SLACK_US) {
    jt_step_t += steps * JETI_STEP_US; jt_si = (uint8_t)((jt_si + steps) % JETI_RING); jt_skipped++;
  }
  if (jq_pwr_dirty) {
    jq_pwr_dirty = false;
    sl_rail_idle(rh, SL_RAIL_IDLE, true);
    sl_rail_set_tx_power_dbm(rh, jq_range ? JQ_RANGE_POWER_DDBM : JQ_TX_POWER_DDBM);
    jq_pwr_log = true;
  }
  uint16_t us[8]; for (uint8_t i = 0; i < 8u; i++) us[i] = jt_ch_us(i);
  jt_base = jt_step_t;
  if (jt_bind) {                                              /* a frame every step on the bind channel, no check (as mpm-v2) */
    uint8_t ch = jt_bind_ch;
    jt_data = false;
    jt_len = jeti_bind_frame(jt_plain, jt_seq++, jt_dwell < JT_BIND_BOOT, jq_pan, jq_short, ch, us);
    if (++jt_dwell >= JT_BIND_DWELL) { jt_dwell = 0; jt_bind_ch = (uint8_t)((jt_bind_ch + 1u) % JETI_CHANNELS); }
    jt_last_ch = ch; jt_stage = 1u; jt_got = false;
    jq_first |= 1u;
    if (jq_send(rh, jt_plain, jt_len, ch, jt_base + JT_TX_AT_US, false) != SL_RAIL_STATUS_NO_ERROR) { jt_err++; jt_stage = 0u; }
    jt_n++;
  } else {
    if (jt_n % JT_SWEEP_FRAMES == 0u) jt_sweep();
    jt_data = true;
    jt_tx_slot = jt_si; jt_slot = jt_si; jt_slide = 0; jt_be = JT_MIN_BE; jt_place = JETI_PLACE_MIDDLE;
    jt_len = jeti_data_frame(jt_plain, jt_seq++, jt_rx_pan, jt_rx_short, jq_short, jt_ring, jt_si, JETI_PLACE_MIDDLE, (uint8_t)jt_n, us);
    jt_n++;
    jt_u = (uint16_t)(JT_CCA_AT_US + jt_backoff());
    jt_set_place();
    jt_attempt(rh);
  }
  jt_step_t += steps * JETI_STEP_US; jt_si = (uint8_t)((jt_si + steps) % JETI_RING);
  (void)sl_rail_set_timer(rh, jt_step_t + JT_CCA_AT_US - JT_LEAD_US, SL_RAIL_TIME_ABSOLUTE, jt_timer);
}

/* A busy channel, as mpm-v2 (802.15.4 CSMA-CA, the Jeti module's): wait a random time and check again, the place bits
 * following the send time; past the step's window the frame goes on the next step (its channel, the early place) and
 * the next frame 2 steps after that; past that window too it is lost and counts as unanswered for the sweep. */
static void jt_busy_now(sl_rail_handle_t rh)
{
  jt_busy++;
  if (jt_be < JT_MAX_BE) jt_be++;
  jt_u = (uint16_t)(jt_u + JT_CCA_TX_US + jt_backoff());
  if (jt_offset() > JT_WINDOW_END_US) {
    uint8_t s1 = (uint8_t)((jt_slot + 1u) % JETI_RING);
    if (jt_slide || jt_moved[s1] + 1u == jt_n) {              /* (a slot the sweep moved at this frame: not yet announced) */
      jt_lost++; jt_lqi(false, false); jt_miss_now(); jt_stage = 0u;
      return;
    }
    jt_slide = 1u; jt_slot = s1; jt_tx_slot = s1; jt_be = JT_MIN_BE; jt_next++;
    jt_si = (uint8_t)((jt_si + 1u) % JETI_RING);
    jt_step_t += JETI_STEP_US;
    (void)sl_rail_set_timer(rh, jt_step_t + JT_CCA_AT_US - JT_LEAD_US, SL_RAIL_TIME_ABSOLUTE, jt_timer);
    jt_u = (uint16_t)(JETI_STEP_US + JT_MEAN_US + JT_EARLY_US - 150 - JT_CCA_TX_US + jt_backoff());
  }
  jt_set_place();
  if (jt_place == JETI_PLACE_LATE && !jt_slide) jt_late++;
  jt_attempt(rh);
}

void jeti_on_events(sl_rail_events_t events)
{
  if (!jq_run) return;
  if (events & SL_RAIL_EVENT_RX_FIFO_OVERFLOW) jq_ovf = true;
  if (jq_rx) {                                                /* the receiver's replies */
    if (events & SL_RAIL_EVENT_TX_PACKET_SENT) jr_reply_sent++;
    else if (events & SL_RAIL_EVENT_TX_CHANNEL_BUSY) jr_reply_busy++;
    return;
  }
  if ((events & SL_RAIL_EVENT_RX_PACKET_RECEIVED) && jt_stage == 2u) jt_got = true;
  if (events & SL_RAIL_EVENT_TX_PACKET_SENT) { jt_sent++; jq_first |= 2u; if (jt_stage == 1u) jt_stage = 2u; }
  else if (events & SL_RAIL_EVENT_TX_CHANNEL_BUSY) { jq_first |= 4u; if (jt_stage == 1u && jt_data) jt_busy_now(jq_rail); else jt_stage = 0u; }
  else if (events & SL_RAIL_EVENTS_TX_COMPLETION) { jt_err++; jq_first |= 8u; jt_stage = 0u; }
}

bool jeti_start(sl_rail_handle_t rh, const volatile uint16_t *ch_us, bool range_low, bool rx_role, bool bind_held)
{
  jq_rail = rh; jq_in = ch_us; jq_rx = rx_role; jq_range = range_low;
  jq_run = false; jq_input = false; jq_pwr_dirty = false; jq_pwr_log = false; jq_ovf = false; jq_cfg_err = 0; jq_first = 0;
  sl_gpio_set_pin_mode(&jq_led_a, SL_GPIO_MODE_PUSH_PULL, 0);
  sl_gpio_set_pin_mode(&jq_led_b, SL_GPIO_MODE_PUSH_PULL, 0);

  JQ_TRACE("JQ start 1: role %s, radio to idle\r\n", rx_role ? "rx" : "tx");
  sl_rail_idle(rh, SL_RAIL_IDLE_ABORT, true);
  sl_rail_ieee802154_config_t cfg;                            /* as jeti_cap.c: no addresses, no auto-ACK, promiscuous */
  memset(&cfg, 0, sizeof(cfg));
  cfg.p_addresses = NULL;
  cfg.ack_config.enable = false;
  cfg.ack_config.rx_transitions.success = SL_RAIL_RF_STATE_RX; cfg.ack_config.rx_transitions.error = SL_RAIL_RF_STATE_RX;
  cfg.ack_config.tx_transitions.success = SL_RAIL_RF_STATE_RX; cfg.ack_config.tx_transitions.error = SL_RAIL_RF_STATE_RX;
  cfg.timings.idle_to_rx = 100; cfg.timings.tx_to_rx = 182; cfg.timings.idle_to_tx = 100; cfg.timings.rx_to_tx = 192;
  cfg.frames_mask = SL_RAIL_IEEE802154_ACCEPT_STANDARD_FRAMES | SL_RAIL_IEEE802154_ACCEPT_MULTIPURPOSE_FRAMES;
  cfg.promiscuous_mode = true;
  if (sl_rail_ieee802154_init(rh, &cfg) != SL_RAIL_STATUS_NO_ERROR) jq_cfg_err |= 1;
  else if (sl_rail_ieee802154_config_2p4_ghz_radio(rh) != SL_RAIL_STATUS_NO_ERROR) jq_cfg_err |= 2;
  JQ_TRACE("JQ start 2: 802.15.4 radio loaded (err %ld)\r\n", (long)jq_cfg_err);
  if (jq_cfg_err == 0) {
    (void)sl_rail_ieee802154_set_promiscuous_mode(rh, true);
    /* the FCS stays in the packet (PHR, PSDU with FCS: the layout jeti_cap.c prints); frames with a bad FCS are dropped */
    if (sl_rail_config_rx_options(rh, SL_RAIL_RX_OPTION_STORE_CRC | SL_RAIL_RX_OPTION_IGNORE_CRC_ERRORS, SL_RAIL_RX_OPTION_STORE_CRC)
        != SL_RAIL_STATUS_NO_ERROR) jq_cfg_err |= 8;
    if (sl_rail_set_tx_fifo(rh, jq_txfifo, JQ_TX_FIFO_BYTES, 0, 0) != SL_RAIL_STATUS_NO_ERROR) jq_cfg_err |= 4;
    sl_rail_events_t ev = SL_RAIL_EVENT_RX_PACKET_RECEIVED | SL_RAIL_EVENT_RX_FIFO_OVERFLOW | SL_RAIL_EVENTS_TX_COMPLETION;
    if (sl_rail_config_events(rh, ev, ev) != SL_RAIL_STATUS_NO_ERROR) jq_cfg_err |= 16;
  }
  if (jq_cfg_err & 3) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("JQ the 802.15.4 radio setting could not be loaded (cfg_err=%ld). RF off\r\n", (long)jq_cfg_err);
#endif
    return false;
  }
  JQ_TRACE("JQ start 3: options, FIFO, events set (err %ld)\r\n", (long)jq_cfg_err);
  jq_uart_us = jq_now();
  uint32_t id = jq_chip_id();
  if (jq_rx) {                                                /* the receiver's own address: another one than the transmitter role's */
    uint8_t ring[JETI_RING];
    jeti_from_id(id * 2246822519u + 0x52583031u, &jq_pan, &jq_short, ring);
    jr_quiet = range_low;                                     /* RANGE at power-up: listen only */
    jr_start(bind_held);
    return true;
  }

  id ^= (uint32_t)rc_model * 0x9E5u;                          /* per-model binding (0 = unchanged) */
  jeti_from_id(id, &jq_pan, &jq_short, jt_ring);
  { rx_record_t r;
    if (rx_store_load(RX_SLOT_JETITX, &r) && r.bound) {
      jt_rx_pan = (uint16_t)(r.hop[0] | ((uint16_t)r.hop[1] << 8)); jt_rx_short = (uint16_t)(r.hop[2] | ((uint16_t)r.hop[3] << 8));
    } else { jt_rx_pan = 0xFFFFu; jt_rx_short = 0xFFFFu; } }
  memset(jt_miss, 0, sizeof(jt_miss)); memset(jt_moved, 0, sizeof(jt_moved)); memset(jt_bad, 0, sizeof(jt_bad));
  jt_si = 0; jt_seq = 0; jt_n = 0; jt_sent = 0; jt_busy = 0; jt_err = 0; jt_skipped = 0; jt_late = 0; jt_next = 0; jt_lost = 0; jt_moves = 0;
  jt_replies = 0; jt_last_sent = 0; jt_last_replies = 0; jt_reply_t = 0; jt_stage = 0; jt_got = false; jt_data = false;
  jt_lqi_frames = 0; jt_lqi_replies = 0; jt_lqi_sent = 0; jt_lqi_rx = 0; jt_lqi_tx = 0;
  jt_volt = 0; jt_signal = 0; jt_rssi = 0; jt_dwell = 0; jt_bind_ch = jt_ring[0];
  jt_rnd = id ^ jq_now();
  jt_bind = bind_held; jt_bind_armed = !bind_held; jt_bind_since = jq_now();
  sl_rail_set_tx_power_dbm(rh, range_low ? JQ_RANGE_POWER_DDBM : JQ_TX_POWER_DDBM);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("JQ boot jeti " JQ_VERSION " transmitter (quasi Jeti Duplex, experimental): PAN %04X short %04X, ring ch %u %u %u %u %u (sweeping), a frame every 20 ms, 8 channels, receiver %s%04X/%04X, power %s (RAIL set %d ddBm); cfg_err=%ld\r\n",
          (unsigned)jq_pan, (unsigned)jq_short, 11u + jt_ring[0], 11u + jt_ring[1], 11u + jt_ring[2], 11u + jt_ring[3], 11u + jt_ring[4],
          jt_rx_short == 0xFFFFu ? "none stored: broadcast " : "", (unsigned)jt_rx_pan, (unsigned)jt_rx_short,
          range_low ? "range" : "normal", (int)sl_rail_get_tx_power_dbm(rh), (long)jq_cfg_err);
  if (jt_bind) app_log("JQ tx bind: 150 ms on each channel in turn, for a receiver in pairing; up to 60 s\r\n");
#endif
  jt_step_t = jq_now() + 3000u;
  jq_run = true;
  sl_rail_status_t ts = sl_rail_set_timer(rh, jt_step_t + JT_CCA_AT_US - JT_LEAD_US, SL_RAIL_TIME_ABSOLUTE, jt_timer);
  JQ_TRACE("JQ start 4: frame timer set (status %lu)\r\n", (unsigned long)ts);
  (void)ts;
  return true;
}

void jeti_range(bool range_low)
{
  if (range_low == jq_range) return;
  jq_range = range_low; jq_pwr_dirty = true;
}

void jeti_input(bool input_present)
{
  if (input_present == jq_input) return;
  jq_input = input_present;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (jq_run) app_log(input_present ? "JQ input present\r\n" : "JQ no input: sending centre sticks, throttle low\r\n");
#endif
}

void jeti_bind(bool want)                                     /* transmitter: BIND from the pin or the encoder, while running */
{
  if (!jq_run || jq_rx) return;
  if (!want) { jt_bind_armed = true; return; }
  if (!jt_bind_armed || jt_bind) return;
  jt_bind_armed = false;
  CORE_DECLARE_IRQ_STATE; CORE_ENTER_CRITICAL();
  jt_dwell = 0; jt_bind_ch = jt_ring[jt_si]; jt_bind = true;
  CORE_EXIT_CRITICAL();
  jt_bind_since = jq_now();
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("JQ tx bind: 150 ms on each channel in turn, for a receiver in pairing; up to 60 s\r\n");
#endif
}

void jeti_telem(uint32_t now, jeti_telem_t *t)                /* transmitter: what the receiver's replies say */
{
  memset(t, 0, sizeof(*t));
  if (!jq_run || jq_rx) return;
  t->binding = jt_bind;
  t->bound = jt_rx_short != 0xFFFFu;
  t->link = jt_reply_t != 0u && (now - jt_reply_t) <= JT_LINK_US;
  t->rssi_tx = jt_rssi; t->signal = jt_signal; t->lq = jt_lqi_rx; t->vbat_10mv = jt_volt;
}

void jeti_action(void)                                        /* transmitter: every main-loop pass */
{
  if (!jq_run || jq_rx) return;
  uint32_t now = jq_now();
  { uint8_t f = jq_first, nw = (uint8_t)((f & 0x0Fu) & ~(f >> 4));        /* start-up trace: each first, once */
    if (nw) {
      CORE_DECLARE_IRQ_STATE; CORE_ENTER_CRITICAL(); jq_first |= (uint8_t)(nw << 4); CORE_EXIT_CRITICAL();
      if (nw & 1u) JQ_TRACE("JQ start 5: first frame prepared (timer runs)\r\n");
      if (nw & 2u) JQ_TRACE("JQ start 6: first frame sent\r\n");
      if (nw & 4u) JQ_TRACE("JQ start 6: first busy channel seen\r\n");
      if (nw & 8u) JQ_TRACE("JQ start 6: first send FAILED (RAIL reported a transmit error)\r\n");
    } }
  if ((int32_t)(now - jt_step_t) > 100000) {                  /* the timer chain stopped: start it again */
    CORE_DECLARE_IRQ_STATE; CORE_ENTER_CRITICAL();
    jt_err++; jt_stage = 0u;
    jt_step_t = jq_now() + 3000u;
    (void)sl_rail_set_timer(jq_rail, jt_step_t + JT_CCA_AT_US - JT_LEAD_US, SL_RAIL_TIME_ABSOLUTE, jt_timer);
    CORE_EXIT_CRITICAL();
  }
  if (jq_ovf) { jq_ovf = false; sl_rail_reset_fifo(jq_rail, false, true); }
  if (jt_bind && (now - jt_bind_since) >= JT_BIND_TIMEOUT_US) {
    jt_bind = false;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("JQ tx bind: no receiver answered in 60 s; data frames to %04X/%04X\r\n", (unsigned)jt_rx_pan, (unsigned)jt_rx_short);
#endif
  }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (jq_pwr_log) {
    jq_pwr_log = false;
    app_log("JQ RANGE %s: power %d ddBm (RAIL set %d)\r\n", jq_range ? "on" : "off",
            (int)(jq_range ? JQ_RANGE_POWER_DDBM : JQ_TX_POWER_DDBM), (int)sl_rail_get_tx_power_dbm(jq_rail));
  }
#endif

  /* LED_B: steady with input, slow blink without (fast: bind); LED_A: steady while the receiver answers, else toggles per 20 frames */
  static uint32_t tb = 0; static uint32_t la = 0;
  bool link = jt_reply_t != 0u && (now - jt_reply_t) <= JT_LINK_US;
  if (jq_input && !jt_bind) sl_gpio_set_pin(&jq_led_b);
  else if ((now - tb) > (jt_bind ? 100000u : 500000u)) { tb = now; sl_gpio_toggle_pin(&jq_led_b); }
  if (link) sl_gpio_set_pin(&jq_led_a);
  else if (jt_sent / 20u != la) { la = jt_sent / 20u; sl_gpio_toggle_pin(&jq_led_a); }

#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if ((now - jq_uart_us) >= JQ_LOG_US) {
    jq_uart_us = now;
    uint32_t sent = jt_sent;
    uint8_t r[JETI_RING];
    { CORE_DECLARE_IRQ_STATE; CORE_ENTER_CRITICAL(); memcpy(r, jt_ring, JETI_RING); CORE_EXIT_CRITICAL(); }
    app_log("JQ tx %ssent%lu replies%lu (answered %u%%, sent %u%%) busy%lu late%lu next%lu lost%lu skipped%lu err%lu moves%lu", jt_bind ? "BIND " : "",
            (unsigned long)(sent - jt_last_sent), (unsigned long)(jt_replies - jt_last_replies), (unsigned)jt_lqi_rx, (unsigned)jt_lqi_tx,
            (unsigned long)jt_busy, (unsigned long)jt_late, (unsigned long)jt_next, (unsigned long)jt_lost, (unsigned long)jt_skipped,
            (unsigned long)jt_err, (unsigned long)jt_moves);
    if (link) app_log(" | rx %d dBm here, signal %u, %u.%02u V", (int)jt_rssi, (unsigned)jt_signal, (unsigned)(jt_volt / 100u), (unsigned)(jt_volt % 100u));
    app_log(" | in%d ring %u %u %u %u %u ch %u %u %u %u %u %u %u %u\r\n", jq_input ? 1 : 0,
            11u + r[0], 11u + r[1], 11u + r[2], 11u + r[3], 11u + r[4],
            jt_ch_us(0), jt_ch_us(1), jt_ch_us(2), jt_ch_us(3), jt_ch_us(4), jt_ch_us(5), jt_ch_us(6), jt_ch_us(7));
    jt_last_sent = sent; jt_last_replies = jt_replies;
  }
#endif
}

/* a receiver's reply to us, plain: payload 0 type, 1 0x13, 2-3 a frame counter, 4-5 B (bits 14-15 = 01: the voltage in
 * 10 mV), 6 flags, 7 signal level; then 0x22 + the receiver's PAN and short address (pairing) */
static void jt_reply(const uint8_t *plain, uint8_t len, int8_t rssi, uint32_t now)
{
  if (len < JETI_HDR + 8u || (plain[JETI_HDR] & 0x0Fu) != 2u || plain[JETI_HDR + 1u] != 0x13) return;
  if ((uint16_t)(plain[3] | ((uint16_t)plain[4] << 8)) != jq_pan || (uint16_t)(plain[5] | ((uint16_t)plain[6] << 8)) != jq_short) return;
  const uint8_t *p = plain + JETI_HDR;
  uint16_t b = (uint16_t)(p[4] | ((uint16_t)p[5] << 8));
  if ((b & 0xC000u) == 0x4000u) jt_volt = b & 0x3FFFu;
  jt_signal = p[7]; jt_rssi = rssi; jt_replies++; jt_reply_t = now | 1u;
  if (jt_bind && len >= JETI_HDR + 13u && p[8] == 0x22) {     /* pairing: the receiver's address */
    uint16_t pan = (uint16_t)(p[9] | ((uint16_t)p[10] << 8)), sh = (uint16_t)(p[11] | ((uint16_t)p[12] << 8));
    CORE_DECLARE_IRQ_STATE; CORE_ENTER_CRITICAL();
    uint8_t bc = jt_last_ch;                                  /* the receiver waits on the channel it answered on */
    jt_rx_pan = pan; jt_rx_short = sh;
    jt_si = (uint8_t)((jt_si + 1u) % JETI_RING);
    for (uint8_t j = 0; j < JETI_RING; j++) if (jt_ring[j] == bc) jt_ring[j] = jt_ring[jt_si];
    jt_ring[jt_si] = bc;
    jt_bind = false;
    CORE_EXIT_CRITICAL();
    rx_record_t r; memset(&r, 0, sizeof(r)); r.bound = 1;
    r.hop[0] = (uint8_t)pan; r.hop[1] = (uint8_t)(pan >> 8); r.hop[2] = (uint8_t)sh; r.hop[3] = (uint8_t)(sh >> 8);
    bool ok = rx_store_save(RX_SLOT_JETITX, &r);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("JQ tx bound: receiver %04X/%04X on ch %u (%s)\r\n", (unsigned)pan, (unsigned)sh, 11u + bc, ok ? "stored" : "NOT stored: flash write failed");
#else
    (void)ok;
#endif
  }
}

/* ============================== the receiver role ============================== */
bool jeti_rx_fresh(uint32_t now, uint32_t within_us) { return jr_last_rx != 0u && (now - jr_last_rx) <= within_us; }

static void jr_listen(uint8_t ch)                             /* ch 0..15 */
{
  jr_ch = ch & 0x0Fu;
  sl_rail_idle(jq_rail, SL_RAIL_IDLE, false);
  (void)sl_rail_start_rx(jq_rail, (uint16_t)(11u + jr_ch), NULL);
}

static void jr_set_state(uint8_t st, uint32_t now)
{
  jr_state = st; jr_dwell_us = now; jr_hold = 0;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (st == 0u) app_log("JQ rx bind: waiting for a transmitter close by (%d dBm or stronger)\r\n", (int)JR_BIND_RSSI_DBM);
  else if (st == 1u) app_log("JQ rx searching for transmitter %04X\r\n", (unsigned)jr_tx);
#endif
}

static void jr_start(bool bind_held)
{
  uint32_t now = jq_now();
  for (uint8_t i = 0; i < 8u; i++) jr_out_us[i] = 1500u;
  jr_out_us[2] = 1000u;                                       /* throttle low until heard */
  jr_frames = 0; jr_other = 0; jr_resyncs = 0; jr_last_rx = 0; jr_last_t = now; jr_sbus_us = now; jr_hold = 0;
  jr_reply_n = 0; jr_reply_late = 0; jr_reply_sent = 0; jr_reply_busy = 0; jr_n21 = 0; jr_n22 = 0; jr_ftype = 0; jr_fsub = 0; jr_fpan = 0; jr_fdst = 0;
  jr_in_fs = false; jr_out_on = false; jr_binding = false; jr_hits = 0; jr_cand = 0; jr_cand_pan = 0;
  jr_btn_armed = false; jr_btn_done = false; jr_btn_since = 0;
  jr_have = rx_store_load(RX_SLOT_JETI, &jr_rec) && jr_rec.bound;
  jr_tx = jr_have ? (uint16_t)(jr_rec.hop[0] | ((uint16_t)jr_rec.hop[1] << 8)) : 0u;
  jr_tx_pan = jr_have ? (uint16_t)(jr_rec.hop[2] | ((uint16_t)jr_rec.hop[3] << 8)) : 0u;
  jr_fs_valid = jr_have && jr_rec.fs_valid;
  if (jr_fs_valid) for (uint8_t i = 0; i < 8u; i++) jr_fs_us[i] = jr_rec.fs_us[i];
  servo_out_init(); servo_out_source(jr_out_us);
  sbus_init();
  r_vb_init(now);                                             /* the receiver's battery (rlink.c: the divider on PA04), for the replies */
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("JQ boot jeti " JQ_VERSION " receiver (quasi Jeti Duplex, experimental): own address %04X/%04X, %s%s, replies %s, failsafe %s; channels 11..26; cfg_err=%ld\r\n",
          (unsigned)jq_pan, (unsigned)jq_short, jr_have ? "bound" : "not bound", bind_held ? ", BIND held: binding" : "",
          jr_quiet ? "off (RANGE at power-up: listens only)" : "on", jr_fs_valid ? "preset stored" : "hold", (long)jq_cfg_err);
#endif
  jr_bind_since = now; jq_run = true;
  jr_set_state((bind_held || !jr_have) ? 0u : 1u, now);
  jr_listen(0);
  JQ_TRACE("JQ start 4: listening on ch 11\r\n");
}

static void jr_save(void)
{
  jr_rec.bound = 1; jr_rec.hop[0] = (uint8_t)jr_tx; jr_rec.hop[1] = (uint8_t)(jr_tx >> 8);
  jr_rec.hop[2] = (uint8_t)jr_tx_pan; jr_rec.hop[3] = (uint8_t)(jr_tx_pan >> 8);
  jr_rec.fs_valid = jr_fs_valid ? 1u : 0u;
  for (uint8_t i = 0; i < 8u; i++) jr_rec.fs_us[i] = jr_fs_valid ? jr_fs_us[i] : 0u;
  jr_have = rx_store_save(RX_SLOT_JETI, &jr_rec);
}

/* The reply to a transmitter's frame, as a Jeti receiver's (mpm-v2's notes): 17 bytes on the air, the frame's own
 * sequence number, to the transmitter's PAN and short address, about 3 ms after the frame's SFD on its channel, after
 * a clear-channel check; at pairing the receiver's own address appended (22 bytes). The type byte's upper half is not
 * documented: the frame's own is sent back. */
static void jr_reply(const uint8_t *frame, uint16_t pan, uint16_t sh, uint32_t sfd, int8_t rssi, bool pairing, uint32_t now)
{
  if (jr_quiet || pan == 0u) return;
  if ((int32_t)(sfd + JR_REPLY_AT_US - now) < 250) { jr_reply_late++; return; }   /* the main loop was busy (a log line): no reply this time */
  uint8_t f[JETI_HDR + 13u];
  uint8_t *p = jq_header(f, frame[2], pan, sh);
  uint32_t mv = r_vb_mv;
  uint16_t b = (mv != 0u && (jr_reply_n & 3u) == 0u) ? (uint16_t)(0x4000u | ((mv / 10u) & 0x3FFFu)) : 0u;   /* every 4th reply: the voltage */
  int s = (int)rssi + 120;
  *p++ = (uint8_t)((frame[JETI_HDR] & 0xF0u) | 2u); *p++ = 0x13;
  *p++ = (uint8_t)jr_reply_n; *p++ = (uint8_t)(jr_reply_n >> 8);
  *p++ = (uint8_t)b; *p++ = (uint8_t)(b >> 8);
  *p++ = (uint8_t)(1u | (jr_reply_n < 50u ? 0x80u : 0u));     /* antenna 1; bit 7: the link is new */
  *p++ = (uint8_t)(s < 0 ? 0 : s > 127 ? 127 : s);
  if (pairing) { *p++ = 0x22; *p++ = (uint8_t)jq_pan; *p++ = (uint8_t)(jq_pan >> 8); *p++ = (uint8_t)jq_short; *p++ = (uint8_t)(jq_short >> 8); }
  jr_reply_n++;
  (void)jq_send(jq_rail, f, (uint8_t)(p - f), jr_ch, sfd + JR_REPLY_AT_US, true);
  jr_hold = (sfd + JR_REPLY_HOLD_US) | 1u;
}

/* a transmitter's frame, plain; t: its SFD */
static void jr_frame(const uint8_t *plain, uint8_t len, uint32_t t, int8_t rssi, uint32_t now)
{
  jeti_txframe_t fr;
  if (!(jq_first & 4u)) { jq_first |= 4u; JQ_TRACE("JQ start 7: first 802.15.4 frame received (%u bytes, %d dBm)\r\n", (unsigned)len, (int)rssi); }
  if (!jeti_parse(plain, len, &fr)) { jr_other++; return; }
  bool pairing = false;
  if (jr_state == 0u) {                                       /* bind: a strong transmitter becomes the candidate */
    if (rssi < JR_BIND_RSSI_DBM) return;
    jr_cand = fr.tx_short; jr_cand_pan = fr.tx_pan; jr_hits = 0; jr_binding = true;
  } else if (jr_binding) { if (fr.tx_short != jr_cand) return; if (fr.tx_pan) jr_cand_pan = fr.tx_pan; }
  else if (fr.tx_short != jr_tx) { jr_other++; return; }      /* another Jeti transmitter */
  if (jr_binding && fr.sub == 0x22) pairing = true;
  if (!jr_binding && jr_tx_pan == 0u && fr.tx_pan != 0u) { jr_tx_pan = fr.tx_pan; jr_save(); }   /* its PAN, learned from a 0x22 frame */

  /* the hop clock (as mpm-v2's follower): the frame's time less its place = a step's mean frame time; small errors are
   * averaged out, a big one sets the clock anew */
  jr_ftype = plain[JETI_HDR]; jr_fsub = fr.sub;
  jr_fpan = (uint16_t)(plain[3] | ((uint16_t)plain[4] << 8)); jr_fdst = (uint16_t)(plain[5] | ((uint16_t)plain[6] << 8));
  if (fr.sub == 0x21) jr_n21++; else jr_n22++;
  uint8_t was = jr_state;
  uint32_t m = t - (uint32_t)(int32_t)jr_place_us[fr.place];
  if (was == 2u) {
    uint32_t k = (m - jr_mref + JETI_STEP_US / 2u) / JETI_STEP_US;
    int32_t err = (int32_t)(m - (jr_mref + k * JETI_STEP_US));
    if (err > JR_RESYNC_US || err < -JR_RESYNC_US) { jr_resyncs++; jr_mref = m; }
    else jr_mref += k * JETI_STEP_US + (uint32_t)(err / 4);
  } else jr_mref = m;
  memcpy(jr_l, fr.l, JETI_RING);
  jr_step0 = jr_mref - JR_MEAN_US;
  jr_state = 2u;
  jr_rssi = rssi; jr_last_rx = now | 1u; jr_last_t = now; jr_frames++;
  jr_reply(plain, jr_binding ? jr_cand_pan : jr_tx_pan, fr.tx_short, t, rssi, pairing, now);   /* first: it has a deadline */

  if (jr_binding && ++jr_hits >= JR_BIND_FRAMES) {
    jr_binding = false; jr_tx = jr_cand; jr_tx_pan = jr_cand_pan;
    memset(&jr_rec, 0, sizeof(jr_rec)); jr_fs_valid = false;  /* a new bind: the old failsafe preset goes */
    jr_save();
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("JQ rx bound to transmitter %04X/%04X (%s): ch %u, %d dBm%s\r\n", (unsigned)jr_tx_pan, (unsigned)jr_tx, jr_have ? "stored" : "NOT stored: flash write failed",
            11u + jr_ch, (int)rssi, jr_tx_pan ? "" : "; its PAN is not known yet: no replies until a frame carries it");
#endif
  }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  else if (was != 2u && !jr_binding)
    app_log("JQ rx linked to %04X: ch %u, ring %u %u %u %u %u, %u channels, %d dBm\r\n", (unsigned)jr_tx, 11u + jr_ch,
            11u + jr_l[4], 11u + jr_l[0], 11u + jr_l[1], 11u + jr_l[2], 11u + jr_l[3], (unsigned)fr.n, (int)rssi);
#endif
  if (jr_in_fs) {
    jr_in_fs = false;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("JQ rx link back\r\n");
#endif
  }
  if (!jr_binding && fr.n >= 2u) {                            /* a bind candidate moves no servo; a 1-slot block is the DS-14's masked one: not read */
    for (uint8_t i = 0; i < 8u && i < fr.n; i++) {
      int32_t us = (int32_t)jeti_slot_us(fr.slot[i]) + JR_TRIM_US;
      jr_out_us[i] = (uint16_t)(us < 800 ? 800 : us > 2200 ? 2200 : us);
    }
    if (!jr_out_on) { jr_out_on = true; servo_out_enable(true); }
    (void)sbus_send(jr_out_us, false, false); jr_sbus_us = now;
  }
}

void jeti_rx_process(void)                                    /* both roles: packets held in RAIL */
{
  for (;;) {
    sl_rail_rx_packet_info_t info;
    sl_rail_rx_packet_handle_t h = sl_rail_get_rx_packet_info(jq_rail, SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &info);
    if (h == SL_RAIL_RX_PACKET_HANDLE_INVALID) return;
    static uint8_t b[1u + JETI_PSDU_MAX], plain[JETI_PSDU_MAX];
    uint16_t n = info.packet_bytes;
    bool ok = (info.packet_status == SL_RAIL_RX_PACKET_READY_SUCCESS);
    sl_rail_rx_packet_details_t det; memset(&det, 0, sizeof(det));
    det.time_received.time_position = SL_RAIL_PACKET_TIME_DEFAULT;
    (void)sl_rail_get_rx_packet_details(jq_rail, h, &det);
    det.time_received.total_packet_bytes = info.packet_bytes;
    (void)sl_rail_get_rx_time_sync_word_end(jq_rail, &det);
    if (n > sizeof(b)) info.packet_bytes = sizeof(b);
    sl_rail_copy_rx_packet(jq_rail, b, &info);
    sl_rail_release_rx_packet(jq_rail, h);
    if (!jq_run || !ok || n < 4u || n > sizeof(b)) continue;
    uint8_t psdu = b[0] & 0x7Fu;                              /* the PHR: PSDU bytes with the FCS */
    if (psdu < JETI_HDR + 2u + 2u || (uint16_t)(psdu + 1u) > n) continue;
    uint8_t len = (uint8_t)(psdu - 2u);
    jeti_descramble(b + 1, plain, len);
    if (jq_rx) jr_frame(plain, len, (uint32_t)det.time_received.packet_time, det.rssi_dbm, jq_now());
    else jt_reply(plain, len, det.rssi_dbm, jq_now());
  }
}

void jeti_rx_service(bool bind_held)                          /* receiver: every main-loop pass */
{
  if (!jq_run || !jq_rx) return;
  uint32_t now = jq_now();
  if (!(jq_first & 1u)) { jq_first |= 1u; JQ_TRACE("JQ start 5: main loop runs\r\n"); }
  sbus_service();
  r_vb_service(now);
  if (jq_ovf) { jq_ovf = false; sl_rail_reset_fifo(jq_rail, false, true); jr_listen(jr_ch); }
  if (!bind_held) jr_btn_armed = true;                        /* the hold that bound us is not a preset press */
  if (jr_hold != 0u && (int32_t)(now - jr_hold) >= 0) { jr_hold = 0; jr_ch |= 0x80u; }   /* after the reply: RX on the step's channel again, whatever the radio did */

  if (jr_state == 2u) {
    if ((now - jr_last_t) >= JR_UNLINK_US) {                  /* the link is gone: bind again or search */
      bool to_bind = jr_binding || !jr_have;
      jr_binding = false; jr_hits = 0;
      jr_set_state(to_bind ? 0u : 1u, now);
      jr_listen((uint8_t)(jr_ch + 1u));
    } else if (jr_hold == 0u) {                               /* the transmitter's step by the clock -> its channel */
      uint32_t m = (uint32_t)(now - jr_step0 - JR_SWITCH_US);
      m = ((int32_t)m < 0) ? 0u : m / JETI_STEP_US;
      uint8_t ch = jr_l[(m + 4u) % JETI_RING];
      if (ch != jr_ch) jr_listen(ch);
    }
    if (jr_ch & 0x80u) jr_listen(jr_ch);
    if (jr_state == 2u && !jr_binding && bind_held && jr_btn_armed) {   /* BIND pressed 0.3 s while linked: the failsafe preset */
      if (jr_btn_since == 0u) jr_btn_since = now | 1u;
      else if (!jr_btn_done && (now - jr_btn_since) >= JR_FS_PRESS_US && !jr_in_fs) {
        jr_btn_done = true;
        for (uint8_t i = 0; i < 8u; i++) jr_fs_us[i] = jr_out_us[i];
        jr_fs_valid = true; jr_save();
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("JQ rx failsafe preset %s: %u %u %u %u %u %u %u %u\r\n", jr_have ? "stored" : "NOT stored", jr_fs_us[0], jr_fs_us[1], jr_fs_us[2],
                jr_fs_us[3], jr_fs_us[4], jr_fs_us[5], jr_fs_us[6], jr_fs_us[7]);
#endif
      }
    } else { jr_btn_since = 0u; jr_btn_done = false; }
  } else {                                                    /* bind / search: the next channel after the dwell */
    if ((now - jr_dwell_us) >= JR_DWELL_US) { jr_dwell_us = now; jr_listen((uint8_t)(jr_ch + 1u));
      if (!(jq_first & 2u)) { jq_first |= 2u; JQ_TRACE("JQ start 6: first channel change done (ch %u)\r\n", 11u + jr_ch); } }
    if (jr_state == 0u && jr_have && (now - jr_bind_since) >= JR_BIND_TIMEOUT_US) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("JQ rx no transmitter to bind to in 8 s: back to the stored one\r\n");
#endif
      jr_set_state(1u, now);
    }
  }

  if (jr_out_on) {
    if (!jr_in_fs && (now - jr_last_t) >= JR_FS_US) {         /* 1 s without a frame: failsafe */
      jr_in_fs = true;
      if (jr_fs_valid) for (uint8_t i = 0; i < 8u; i++) jr_out_us[i] = jr_fs_us[i];
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("JQ rx failsafe (1 s without a frame): %s\r\n", jr_fs_valid ? "stored preset" : "hold");
#endif
    }
    if ((now - jr_last_t) >= 40000u && (now - jr_sbus_us) >= 20000u) { jr_sbus_us = now; (void)sbus_send(jr_out_us, true, jr_in_fs); }
  }

  /* LED_B: bind fast blink, search slow blink, link steady; LED_A toggles per 20 frames */
  static uint32_t tb = 0; static uint32_t la = 0;
  if (jr_state == 2u && !jr_in_fs && !jr_binding) sl_gpio_set_pin(&jq_led_b);
  else if ((now - tb) > (jr_state == 0u ? 100000u : 500000u)) { tb = now; sl_gpio_toggle_pin(&jq_led_b); }
  if (jr_frames / 20u != la) { la = jr_frames / 20u; sl_gpio_toggle_pin(&jq_led_a); }

#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if ((now - jq_uart_us) >= JQ_LOG_US) {
    jq_uart_us = now;
    static uint32_t lf = 0, ls = 0;
    uint32_t rs = jr_reply_sent;
    static uint32_t l21 = 0, l22 = 0;
    app_log("JQ rx frames type %02X: 0x21 %lu, 0x22 %lu, sent to %04X/%04X (we are %04X/%04X)\r\n", (unsigned)jr_ftype,
            (unsigned long)(jr_n21 - l21), (unsigned long)(jr_n22 - l22), (unsigned)jr_fpan, (unsigned)jr_fdst, (unsigned)jq_pan, (unsigned)jq_short);
    l21 = jr_n21; l22 = jr_n22;
    app_log("JQ rx %s frames%lu other%lu resync%lu rssi%d replies%lu (busy%lu late%lu) batt %u mV ch%u ring %u %u %u %u %u%s ch %u %u %u %u %u %u %u %u\r\n",
            jr_state == 0u ? "bind" : jr_state == 1u ? "search" : "link", (unsigned long)(jr_frames - lf), (unsigned long)jr_other,
            (unsigned long)jr_resyncs, (int)jr_rssi, (unsigned long)(rs - ls), (unsigned long)jr_reply_busy, (unsigned long)jr_reply_late, (unsigned)r_vb_mv,
            11u + jr_ch, 11u + jr_l[4], 11u + jr_l[0], 11u + jr_l[1], 11u + jr_l[2], 11u + jr_l[3], jr_in_fs ? " FAILSAFE" : "",
            jr_out_us[0], jr_out_us[1], jr_out_us[2], jr_out_us[3], jr_out_us[4], jr_out_us[5], jr_out_us[6], jr_out_us[7]);
    lf = jr_frames; ls = rs;
  }
#endif
}
#endif /* JETI_HOST_TEST */
