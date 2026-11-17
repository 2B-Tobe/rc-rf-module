/* fasst.c — see fasst.h.  #included into app_process.c.   v0.3
 * The framing (fasst_packet, fasst_spread, the hop rule, the whitening stream, the F/S types) is ported from
 * mpm-v2's src/core/fasst_frame.c by Raphael Coeffic (GPL v3). */
#include "fasst.h"
#include <string.h>
#include "sl_core.h"
#include "rx_store.h"
#include "servo_out.h"
#include "sbus_out.h"

/* the PHY "fasst" in config/rail/radio_settings.radioconf. Weak: the build links before Simplicity Studio has
 * generated it into autogen/rail_config.c; fasst_start then refuses to run and says so. */
extern const RAIL_ChannelConfig_t fasst_channelConfig __attribute__((weak));

#define FASST_FRAME_US       7000u
#define FASST_BASE_HZ  2405376000ul      /* 2349 x 1.024 MHz */
#define FASST_SPACING_HZ  2048000ul
#define FASST_AIR_LEN        (FASST_CHIPS_LEN - 4u)   /* the first 32 chips are the radio's sync word */
#define FASST_FS_EVERY       143u        /* frames: the F/S cycle about once a second */
#define FASST_FS_THROTTLE_US 1000u       /* failsafe sent: throttle (channel 3) here, every other channel holds */
#define FASST_START_SLACK_US 600u        /* a frame that cannot start this far ahead is skipped with its hop */
#define FASST_MIN_SLACK_US   100u
#define FASST_LOG_US     5000000u
#define FASST_TX_POWER_DDBM  200         /* flight: module maximum (deci-dBm; RAIL caps it, about +19.5 dBm) */
#define FASST_RANGE_POWER_DDBM (-100)
#define FASST_TX_FIFO_BYTES  512u
#define FASST_RX_FIFO_BYTES 1024u         /* a packet is 348 bytes: the module's usual 256-byte RX FIFO is too small */
#define FASST_SYNC     0x2BB1u           /* bit 0 first */
#define FASST_BARKER   0x712u            /* 11100010010 = a 0 bit, first chip in bit 10 */
#define FASST_ID_DEFAULT 0x12689FE0ul    /* the TM-10's (tests) */

volatile int32_t fasst_diag[FASST_DIAG_N];

/* ---- the pieces a host test can run ---- */
static uint8_t fa_id[4] = { (uint8_t)(FASST_ID_DEFAULT >> 24), (uint8_t)(FASST_ID_DEFAULT >> 16),
                            (uint8_t)(FASST_ID_DEFAULT >> 8), (uint8_t)FASST_ID_DEFAULT };

void fasst_set_id(uint32_t id) { for (uint8_t i = 0; i < 4u; i++) fa_id[i] = (uint8_t)(id >> (24u - 8u * i)); }

/* Whitening of the whole packet: a fixed 16-bit LFSR stream, the same for every transmitter */
static const uint8_t fa_white[FASST_PACKET_LEN] = { 0x00, 0xC0, 0x4C, 0xED, 0xDE, 0x9B, 0xFF, 0xC6, 0x11, 0x77, 0x70,
                                                    0x1B, 0x0F, 0x08, 0x98, 0x65, 0x09, 0x95, 0x4E, 0x26, 0x57, 0xFE };

/* F/S cycle: type bytes 12 / 19 as on air and the channels whose F/S position the packet carries in their slot
 * (-1: none); bit 2 of byte 12 (ch1..4, ch9, ch10) or 19 (ch5..8) on air = that channel holds instead. */
typedef struct { uint8_t b12, b19; int8_t fs[2]; } fa_fs_type_t;
static const fa_fs_type_t fa_fs_types[FASST_FS_TYPES] = {
  { 0x8B, 0x82, { 0, 4 } }, { 0x8A, 0x83, { 1, 5 } }, { 0x81, 0x80, { 2, 6 } },
  { 0x99, 0x90, { 3, 7 } }, { 0x8B, 0x99, { 8, -1 } }, { 0x8A, 0x99, { 9, -1 } },
};

uint8_t fasst_next_hop(uint8_t k, uint8_t step)              /* k + step mod 37, skipping 36 */
{
  k = (uint8_t)((k + step) % 37u);
  return (k == 36u) ? (uint8_t)((k + step) % 37u) : k;
}

uint32_t fasst_freq_hz(uint8_t k) { return FASST_BASE_HZ + (uint32_t)k * FASST_SPACING_HZ; }

uint16_t fasst_value(uint16_t us)                            /* 1000 us -> 224, 1500 -> 1024, 2000 -> 1824 */
{
  int32_t v = (((int32_t)us - 860) * 8) / 5;
  if (v < 0) v = 0;
  if (v > 2047) v = 2047;
  return (uint16_t)v;
}

static uint16_t fa_crc_kermit(const uint8_t *p, uint32_t n)
{
  uint16_t c = 0;
  while (n--) {
    c ^= *p++;
    for (uint8_t i = 0; i < 8u; i++) c = (c & 1u) ? (uint16_t)((c >> 1) ^ 0x8408u) : (uint16_t)(c >> 1);
  }
  return c;
}

/* Plain packet: ID, hop position k + 1 and step, the groups (2 slots of 11 bits + flag each), the type bytes 12 / 19
 * (plain); CRC over bytes 0..19, then the whole packet whitened */
static void fa_pack(uint8_t *p, uint8_t k, uint8_t step, uint16_t g[4][2], uint8_t flags[4][2], uint8_t b12, uint8_t b19)
{
  static const uint8_t pos[4] = { 6, 9, 13, 16 };
  memcpy(p, fa_id, 4);
  p[4] = (uint8_t)((k + 1u) >> 1);
  p[5] = (uint8_t)((((k + 1u) & 1u) << 7) | (step & 0x7Fu));
  for (uint8_t i = 0; i < 4u; i++) {
    uint32_t w = (uint32_t)(g[i][0] & 0x7FFu) | ((uint32_t)flags[i][0] << 11) | ((uint32_t)(g[i][1] & 0x7FFu) << 12)
               | ((uint32_t)flags[i][1] << 23);
    for (uint8_t j = 0; j < 3u; j++) p[pos[i] + j] = (uint8_t)(w >> (8u * j));
  }
  p[12] = b12;
  p[19] = b19;
  uint16_t crc = fa_crc_kermit(p, 20);
  p[20] = (uint8_t)crc;
  p[21] = (uint8_t)(crc >> 8);
  for (uint8_t i = 0; i < FASST_PACKET_LEN; i++) p[i] ^= fa_white[i];
}

void fasst_packet(uint8_t *p, uint8_t k, uint8_t step, const uint16_t *ch, bool alt, uint8_t type,
                  const uint16_t *fs, uint16_t hold)
{
  uint16_t v[FASST_CHANNELS];                                /* slot values: the F/S position for the type's channels that don't hold */
  memcpy(v, ch, sizeof v);
  uint8_t b12 = 0x90, b19 = 0x99;
  bool fs_a = false, fs_b = false;                           /* group 1 slots taken by the type */
  if (type) {
    const fa_fs_type_t *t = &fa_fs_types[type - 1u];
    b12 = t->b12;
    b19 = t->b19;
    for (uint8_t i = 0; i < 2u; i++) {
      int8_t c = t->fs[i];
      if (c < 0) continue;
      if ((hold >> c) & 1u) {
        if (c >= 4 && c < 8) b19 |= 0x04u; else b12 |= 0x04u;
      } else v[c] = fs[c];
      if (c == 0 || c == 8) fs_a = true;
      if (c == 1 || c == 9) fs_b = true;
    }
  }
  bool a9  = fs_a ? (type == 5u) : alt;                      /* group 1: slot a = ch1 or ch9 (flag), slot b = ch2 or ch10 */
  bool b10 = fs_b ? (type == 6u) : alt;
  uint16_t g[4][2] = { { v[a9 ? 8 : 0], v[b10 ? 9 : 1] }, { v[2], v[3] }, { v[4], v[5] }, { v[6], v[7] } };
  uint8_t flags[4][2] = { { (uint8_t)a9, (uint8_t)b10 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };
  fa_pack(p, k, step, g, flags, (uint8_t)(b12 ^ fa_white[12]), (uint8_t)(b19 ^ fa_white[19]));
}

/* The chips: 64 preamble bits (1010..10), the 16 sync bits, the packet bits (bit 0 of each byte first); every bit
 * as the 11 Barker chips (inverted for a 1), first chip first, packed MSB first. */
void fasst_spread(uint8_t *chips, const uint8_t *p)
{
  uint32_t acc = 0; uint8_t nacc = 0; uint16_t o = 0;
  for (uint16_t i = 0; i < 64u + 16u + 8u * FASST_PACKET_LEN; i++) {
    uint8_t d;
    if (i < 64u) d = (uint8_t)(~i & 1u);
    else if (i < 80u) d = (uint8_t)((FASST_SYNC >> (i - 64u)) & 1u);
    else { uint16_t j = (uint16_t)(i - 80u); d = (uint8_t)((p[j >> 3] >> (j & 7u)) & 1u); }
    acc = (acc << 11) | ((d ? ~FASST_BARKER : FASST_BARKER) & 0x7FFu);
    nacc = (uint8_t)(nacc + 11u);
    while (nacc >= 8u) { nacc = (uint8_t)(nacc - 8u); chips[o++] = (uint8_t)(acc >> nacc); }
  }
}

uint32_t fasst_rail_sync(const uint8_t *chips)               /* chip i of the first 32 in bit i: RAIL sends bit 0 first */
{
  uint32_t w = 0;
  for (uint8_t i = 0; i < 32u; i++) if (chips[i >> 3] & (0x80u >> (i & 7u))) w |= (1ul << i);
  return w;
}

uint16_t fasst_us(uint16_t value)                            /* the reverse of fasst_value: 224 -> 1000 us, 1024 -> 1500 */
{
  uint32_t us = 860u + ((uint32_t)value * 5u) / 8u;
  if (us < 800u) us = 800u;
  if (us > 2200u) us = 2200u;
  return (uint16_t)us;
}

/* Receiver: the packet out of the chips after the radio's sync word (FASST_AIR_LEN bytes, first chip in bit 7 of
 * byte 0). The sync word is the first 32 chips of the Barker preamble, a pattern that repeats every 22 chips through
 * that preamble: the radio may have caught a later repeat, so the 16 sync bits (0x2BB1) are looked for in the
 * despread bits. raw: the 22 packet bytes as on air (whitened), CRC good. late_chips: how many chips after the first
 * possible place the radio's sync word was found. False: no sync bits with a packet behind them. */
bool fasst_rx_decode(const uint8_t *pl, uint8_t *raw, uint16_t *late_chips)
{
  uint8_t bits[254];                                         /* bit j = chips 1 + 11 j .. (the sync word ends 10 chips into a bit) */
  for (uint16_t j = 0; j < 253u; j++) {
    int8_t corr = 0;
    for (uint8_t c = 0; c < 11u; c++) {
      uint16_t i = (uint16_t)(1u + 11u * j + c);
      uint8_t chip = (uint8_t)((pl[i >> 3] >> (7u - (i & 7u))) & 1u);
      corr = (int8_t)(corr + ((chip == ((FASST_BARKER >> (10u - c)) & 1u)) ? 1 : -1));
    }
    bits[j] = (corr < 0) ? 1u : 0u;
  }
  for (int16_t j = 0; j <= 61; j++) {                        /* 61: the radio caught the first place; each repeat is 2 bits less.
                                                                From 0 up: before the real sync bits there is only preamble */
    uint8_t k;
    for (k = 0; k < 16u; k++) if (bits[j + k] != ((FASST_SYNC >> k) & 1u)) break;
    if (k < 16u) continue;
    memset(raw, 0, FASST_PACKET_LEN);
    for (uint16_t b = 0; b < 8u * FASST_PACKET_LEN; b++) if (bits[j + 16 + b]) raw[b >> 3] |= (uint8_t)(1u << (b & 7u));
    uint8_t plain[FASST_PACKET_LEN];
    for (uint8_t i = 0; i < FASST_PACKET_LEN; i++) plain[i] = (uint8_t)(raw[i] ^ fa_white[i]);
    if (fa_crc_kermit(plain, 20) != (uint16_t)(plain[20] | ((uint16_t)plain[21] << 8))) continue;   /* sync bits by chance, in the data */
    *late_chips = (uint16_t)(11u * (uint16_t)(61 - j));
    return true;
  }
  return false;
}

/* The packet as on air -> plain; false: the CRC does not fit, or the hop position / step make no sense */
bool fasst_unpack(const uint8_t *raw, uint8_t *plain, uint8_t *k, uint8_t *step)
{
  for (uint8_t i = 0; i < FASST_PACKET_LEN; i++) plain[i] = (uint8_t)(raw[i] ^ fa_white[i]);
  if (fa_crc_kermit(plain, 20) != (uint16_t)(plain[20] | ((uint16_t)plain[21] << 8))) return false;
  uint8_t pos = (uint8_t)((plain[4] << 1) | (plain[5] >> 7));
  *k = (uint8_t)(pos - 1u);
  *step = (uint8_t)(plain[5] & 0x7Fu);
  return pos >= 1u && *k < FASST_HOPS && *step >= 1u && *step < 37u;
}

/* The channels of a plain packet (raw: the same packet as on air, for the type bytes). v[10]: the slot values;
 * live bit i = v[i] is channel i's live position; fsp bit i = v[i] is its failsafe position; fsh bit i = it holds
 * in failsafe. A channel not in this packet (ch1 / ch2 or ch9 / ch10 every other frame) has none of the bits. */
void fasst_slots(const uint8_t *plain, const uint8_t *raw, uint16_t *v, uint16_t *live, uint16_t *fsp, uint16_t *fsh)
{
  static const uint8_t pos[4] = { 6, 9, 13, 16 };
  uint8_t chn[8];
  for (uint8_t i = 0; i < 4u; i++) {
    uint32_t w = (uint32_t)plain[pos[i]] | ((uint32_t)plain[pos[i] + 1u] << 8) | ((uint32_t)plain[pos[i] + 2u] << 16);
    uint8_t c0 = (uint8_t)(2u * i), c1 = (uint8_t)(2u * i + 1u);
    if (i == 0u) { c0 = ((w >> 11) & 1u) ? 8u : 0u; c1 = ((w >> 23) & 1u) ? 9u : 1u; }
    v[c0] = (uint16_t)(w & 0x7FFu); v[c1] = (uint16_t)((w >> 12) & 0x7FFu);
    chn[2u * i] = c0; chn[2u * i + 1u] = c1;
  }
  *live = 0; *fsp = 0; *fsh = 0;
  for (uint8_t i = 0; i < 8u; i++) *live |= (uint16_t)(1u << chn[i]);
  uint8_t b12 = raw[12], b19 = raw[19];
  if (b12 == 0x90u && b19 == 0x99u) return;                  /* a normal packet */
  for (uint8_t t = 0; t < FASST_FS_TYPES; t++) {
    if ((b12 & 0xFBu) != fa_fs_types[t].b12 || (b19 & 0xFBu) != fa_fs_types[t].b19) continue;
    for (uint8_t i = 0; i < 2u; i++) {
      int8_t c = fa_fs_types[t].fs[i];
      if (c < 0) continue;
      bool hold = (c >= 4 && c < 8) ? ((b19 & 0x04u) != 0u) : ((b12 & 0x04u) != 0u);
      if (hold) *fsh |= (uint16_t)(1u << c);                 /* the slot then carries the live value */
      else if (*live & (1u << c)) { *fsp |= (uint16_t)(1u << c); *live &= (uint16_t)~(1u << c); }
    }
    return;
  }
}

/* ---- the radio ---- */
static sl_rail_handle_t fa_rail;
static const volatile uint16_t *fa_in;
static SL_RAIL_DECLARE_FIFO_BUFFER(fa_txfifo, FASST_TX_FIFO_BYTES);
static RAIL_ChannelConfigEntry_t fa_ent;
static RAIL_ChannelConfig_t      fa_cfg;
static uint8_t  fa_hop, fa_step;
static uint8_t  fa_chips[FASST_CHIPS_LEN];
static volatile uint8_t  fa_stage;          /* 0 idle, 1 a packet on its way */
static volatile uint32_t fa_t0;             /* launch time of the packet under way */
static volatile uint32_t fa_next;           /* start of the next frame on the 7 ms grid */
static volatile uint32_t fa_frame, fa_tx_count, fa_skipped;
static uint32_t fa_uart_us, fa_tx_last;
static bool     fa_run, fa_input, fa_range, fa_started;
static bool     fa_rx;                      /* receiver role */
static volatile bool fa_pwr_dirty, fa_pwr_log;
static sl_gpio_t fa_led_a = { .port = SL_GPIO_PORT_C, .pin = 4 };
static sl_gpio_t fa_led_b = { .port = SL_GPIO_PORT_C, .pin = 5 };

static inline uint32_t fa_now(void) { return (uint32_t)sl_rail_get_time(fa_rail); }

static uint16_t fa_ch_us(uint8_t i)                          /* the input, or centre sticks with the throttle low */
{
  if (i >= 8u) return 1500u;                                 /* ch9, ch10: centre */
  if (fa_input) return fa_in[i];
  return (i == 2u) ? 1000u : 1500u;
}

static void fa_make(void)                                    /* this frame's packet, as chips */
{
  uint16_t ch[FASST_CHANNELS], fs[FASST_CHANNELS];
  for (uint8_t i = 0; i < FASST_CHANNELS; i++) { ch[i] = fasst_value(fa_ch_us(i)); fs[i] = ch[i]; }
  fs[2] = fasst_value(FASST_FS_THROTTLE_US);
  uint16_t hold = (uint16_t)(0x3FFu & ~(1u << 2));           /* F/S: throttle to its position, the rest hold */
  uint32_t n = fa_frame % FASST_FS_EVERY;
  uint8_t type = (n < FASST_FS_TYPES) ? (uint8_t)(n + 1u) : 0u;
  uint8_t pkt[FASST_PACKET_LEN];
  fasst_packet(pkt, fa_hop, fa_step, ch, (fa_frame & 1u) != 0u, type, fs, hold);
  fasst_spread(fa_chips, pkt);
}

static void fa_launch(uint32_t when)                         /* main loop (in a critical section) or the RAIL event handler */
{
  (void)sl_rail_write_tx_fifo(fa_rail, fa_chips + 4, FASST_AIR_LEN, true);
  sl_rail_status_t s;
  if ((int32_t)(when - fa_now()) < (int32_t)FASST_MIN_SLACK_US) {
    fasst_diag[FA_LATE]++;
    s = sl_rail_start_tx(fa_rail, fa_hop, SL_RAIL_TX_OPTIONS_DEFAULT, NULL);
  } else {
    sl_rail_scheduled_tx_config_t sc = { .when = when, .mode = SL_RAIL_TIME_ABSOLUTE,
                                         .tx_during_rx = SL_RAIL_SCHEDULED_TX_DURING_RX_POSTPONE_TX };
    s = sl_rail_start_scheduled_tx(fa_rail, fa_hop, SL_RAIL_TX_OPTIONS_DEFAULT, &sc, NULL);
    if (s != SL_RAIL_STATUS_NO_ERROR) { fasst_diag[FA_LATE]++; s = sl_rail_start_tx(fa_rail, fa_hop, SL_RAIL_TX_OPTIONS_DEFAULT, NULL); }
  }
  if (s != SL_RAIL_STATUS_NO_ERROR) { fasst_diag[FA_TXERR]++; fasst_diag[FA_CFG_ERR] |= 64; fa_stage = 0u; }   /* the main loop restarts the chain */
  else fasst_diag[FA_TX]++;
}

/* The next frame, on the 7 ms grid. The receiver hops by the step once per 7 ms whether it hears a packet or not,
 * so a slot that can no longer be met is skipped WITH its hop. */
static void fa_frame_start(void)
{
  uint32_t now = fa_now();
  while ((int32_t)(fa_next - now) < (int32_t)FASST_START_SLACK_US) {
    fa_next += FASST_FRAME_US; fa_hop = fasst_next_hop(fa_hop, fa_step); fa_frame++; fa_skipped++;
  }
  fa_make();
  fa_t0 = fa_next; fa_next += FASST_FRAME_US;
  fa_stage = 1u; fa_tx_count++; fasst_diag[FA_HOP] = fa_hop;
  fa_launch(fa_t0);
}

/* The frames run from the radio's own events, not from the main loop: packet done -> the hop and the next packet.
 * A log line or a flash write in the main loop therefore cannot delay a frame. */
void fasst_on_events(sl_rail_events_t events)
{
  if (!fa_run || fa_rx || !(events & SL_RAIL_EVENTS_TX_COMPLETION)) return;
  if (!(events & SL_RAIL_EVENT_TX_PACKET_SENT)) fasst_diag[FA_TXERR]++;
  if (fa_stage != 1u) return;
  fa_hop = fasst_next_hop(fa_hop, fa_step); fa_frame++;
  if (fa_pwr_dirty) {                                        /* RANGE changed: the new power between two frames */
    fa_pwr_dirty = false;
    sl_rail_idle(fa_rail, SL_RAIL_IDLE, true);
    sl_rail_set_tx_power_dbm(fa_rail, fa_range ? FASST_RANGE_POWER_DDBM : FASST_TX_POWER_DDBM);
    fa_pwr_log = true;
  }
  fa_frame_start();
}

static void fr_start(bool bind_held);

bool fasst_start(sl_rail_handle_t rh, const volatile uint16_t *ch_us, bool range_low, bool rx_role, bool bind_held)
{
  fa_rx = rx_role;
  fa_rail = rh; fa_in = ch_us; fa_range = range_low;
  for (uint8_t i = 0; i < FASST_DIAG_N; i++) fasst_diag[i] = 0;
  fa_stage = 0; fa_run = false; fa_input = false; fa_pwr_dirty = false; fa_pwr_log = false; fa_started = false;
  fa_frame = 0; fa_tx_count = 0; fa_tx_last = 0; fa_skipped = 0; fa_hop = 0;
  sl_gpio_set_pin_mode(&fa_led_a, SL_GPIO_MODE_PUSH_PULL, 0);
  sl_gpio_set_pin_mode(&fa_led_b, SL_GPIO_MODE_PUSH_PULL, 0);

  if (&fasst_channelConfig == 0) {                           /* rail_config.c has no fasst PHY yet */
    fasst_diag[FA_CFG_ERR] |= 128;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("FA the fasst radio setting is not in this build: open radio_settings.radioconf in Simplicity Studio, save, build again. RF off\r\n");
#endif
    return false;
  }

  uint64_t u = ((uint64_t)DEVINFO->EUI64H << 32) | DEVINFO->EUI64L;
  uint32_t seed = (uint32_t)(u ^ (u >> 16) ^ (u >> 32) ^ (u >> 48));
  seed ^= (uint32_t)rc_model * 0x9E5u;                       /* per-model binding (0 = unchanged) */
  uint32_t id = 0x12000000ul | (seed & 0x00FFFFFFul);        /* byte 0 as the TM-10's */
  fasst_set_id(id);
  fa_step = (uint8_t)(1u + (((uint32_t)sl_rail_get_time(rh) ^ (seed >> 9)) % 36u));   /* 1..36, new at every power-up */
  fasst_diag[FA_ID] = (int32_t)id; fasst_diag[FA_STEP] = fa_step;

  sl_rail_idle(rh, SL_RAIL_IDLE_ABORT, true);
  fa_ent = fasst_channelConfig.configs[0];                   /* RAM copy with FASST's exact frequencies */
  fa_ent.baseFrequency = FASST_BASE_HZ;
  fa_ent.channelSpacing = FASST_SPACING_HZ;
  fa_cfg = fasst_channelConfig;
  fa_cfg.configs = &fa_ent; fa_cfg.length = 1u;
  if (sl_rail_config_channels(rh, (const sl_rail_channel_config_t *)&fa_cfg, NULL) != SL_RAIL_STATUS_NO_ERROR) fasst_diag[FA_CFG_ERR] |= 1;
  sl_rail_prepare_channel(rh, 0);
  if (sl_rail_set_fixed_length(rh, FASST_AIR_LEN) != FASST_AIR_LEN) fasst_diag[FA_CFG_ERR] |= 2;
  if (sl_rail_set_tx_fifo(rh, fa_txfifo, FASST_TX_FIFO_BYTES, 0, 0) != SL_RAIL_STATUS_NO_ERROR) fasst_diag[FA_CFG_ERR] |= 4;
  { uint8_t pkt[FASST_PACKET_LEN] = { 0 };                   /* the first 32 chips are preamble chips: the same in every packet */
    fasst_spread(fa_chips, pkt);
    uint32_t w = fasst_rail_sync(fa_chips);
    sl_rail_sync_word_config_t sc = { .sync_word_bits = 32u, .sync_word_0 = w, .sync_word_1 = w };
    if (sl_rail_config_sync_words(rh, &sc) != SL_RAIL_STATUS_NO_ERROR) fasst_diag[FA_CFG_ERR] |= 32; }
  if (fa_rx) { fr_start(bind_held); return true; }
  sl_rail_config_events(rh, SL_RAIL_EVENTS_TX_COMPLETION, SL_RAIL_EVENTS_TX_COMPLETION);
  sl_rail_set_tx_power_dbm(rh, range_low ? FASST_RANGE_POWER_DDBM : FASST_TX_POWER_DDBM);

#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("FA boot fasst v0.1 transmitter (FASST 10ch, experimental): id %08lX, hop step %u, 36 channels from %lu Hz every %lu Hz, frame %u us, %u chip bytes per packet, failsafe cycle every %u frames (throttle %u us, the rest hold), power %s (flight %d, range %d ddBm; RAIL set %d); cfg_err=%ld\r\n",
          (unsigned long)id, (unsigned)fa_step, (unsigned long)FASST_BASE_HZ, (unsigned long)FASST_SPACING_HZ, (unsigned)FASST_FRAME_US,
          (unsigned)FASST_CHIPS_LEN, (unsigned)FASST_FS_EVERY, (unsigned)FASST_FS_THROTTLE_US,
          range_low ? "range" : "normal", (int)FASST_TX_POWER_DDBM, (int)FASST_RANGE_POWER_DDBM,
          (int)sl_rail_get_tx_power_dbm(rh), (long)fasst_diag[FA_CFG_ERR]);
#endif
  fa_uart_us = fa_now();
  fa_next = fa_now() + 2000u;
  fa_run = true;
  return true;
}

void fasst_range(bool range_low)
{
  if (range_low == fa_range) return;
  fa_range = range_low; fa_pwr_dirty = true;
}

void fasst_input(bool input_present)
{
  if (input_present == fa_input) return;
  fa_input = input_present; fasst_diag[FA_INPUT] = input_present ? 1 : 0;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (fa_run) app_log(input_present ? "FA input present\r\n" : "FA no input: sending centre sticks, throttle low\r\n");
#endif
}

void fasst_action(void)
{
  uint32_t now = fa_now();
  if (fa_rx) return;                                           /* the receiver: fasst_rx_service */
  if (fa_run) {
    /* start the chain, or restart it when it stopped (a refused send) or no event came for three frames */
    if (fa_stage == 0u || (int32_t)(now - fa_t0) > (int32_t)(3u * FASST_FRAME_US)) {
      CORE_DECLARE_IRQ_STATE; CORE_ENTER_CRITICAL();
      if (fa_stage == 0u || (int32_t)(fa_now() - fa_t0) > (int32_t)(3u * FASST_FRAME_US)) {
        if (fa_started) { fa_hop = fasst_next_hop(fa_hop, fa_step); fa_frame++; fasst_diag[FA_TXERR]++; }   /* the slot that failed keeps its hop */
        fa_started = true;
        fa_frame_start();
      }
      CORE_EXIT_CRITICAL();
    }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    if (fa_pwr_log) {
      fa_pwr_log = false;
      app_log("FA RANGE %s: power %d ddBm (RAIL set %d)\r\n", fa_range ? "on" : "off",
              (int)(fa_range ? FASST_RANGE_POWER_DDBM : FASST_TX_POWER_DDBM), (int)sl_rail_get_tx_power_dbm(fa_rail));
    }
#endif
  }

  /* LED_B: steady with input, slow blink without (fast: no PHY); LED_A toggles per 20 frames */
  static uint32_t tb = 0; static uint32_t la = 0;
  if (fa_run && fa_input) sl_gpio_set_pin(&fa_led_b);
  else if ((now - tb) > (fa_run ? 500000u : 100000u)) { tb = now; sl_gpio_toggle_pin(&fa_led_b); }
  if (fa_tx_count / 20u != la) { la = fa_tx_count / 20u; sl_gpio_toggle_pin(&fa_led_a); }

#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (fa_run && (now - fa_uart_us) >= FASST_LOG_US) {
    fa_uart_us = now;
    app_log("FA tx frames%lu skipped%lu late%ld err%ld in%d hop%u ch %u %u %u %u %u %u %u %u\r\n", (unsigned long)(fa_tx_count - fa_tx_last),
            (unsigned long)fa_skipped, (long)fasst_diag[FA_LATE], (long)fasst_diag[FA_TXERR], fa_input ? 1 : 0, (unsigned)fa_hop,
            fa_in[0], fa_in[1], fa_in[2], fa_in[3], fa_in[4], fa_in[5], fa_in[6], fa_in[7]);
    fa_tx_last = fa_tx_count;
  }
#endif
}

/* ============================== the receiver role ==============================
 * For another board of this kind (v0.2). The radio's sync word is the first 32 chips of the packet, right after the
 * radio's own 16-bit preamble, which a board of this kind sends and a Futaba transmitter does not: whether this
 * role hears a real FASST transmitter is not known. */
#define FRX_AIR_US          1900u       /* a packet on the air: 2816 chips at 1.5 Mchip/s + the radio's preamble */
#define FRX_SLACK_US         700u       /* waited after a packet should have ended before the slot counts as lost */
#define FRX_DWELL_US      300000u       /* search / bind: this long per channel (a transmitter passes each within 252 ms) */
#define FRX_PARK_FRAMES       30u       /* 0.2 s lost: wait on one channel instead of hopping by the clock */
#define FRX_PARK_MOVE         43u       /* parked: another channel every 0.3 s */
#define FRX_FS_FRAMES        143u       /* 1 s lost: failsafe */
#define FRX_LOST_FRAMES      429u       /* 3 s lost: search */
#define FRX_BIND_RSSI_DBM    (-75)
#define FRX_BIND_FRAMES       10u       /* bind: this many frames of the same strong transmitter, followed hop by hop */
#define FRX_BIND_MISSES       20u       /* ... given up after this many slots in a row without one */
#define FRX_BIND_TIMEOUT_US 8000000u
#define FRX_FS_PRESS_US   300000u

static SL_RAIL_DECLARE_FIFO_BUFFER(fr_rxfifo, FASST_RX_FIFO_BYTES);
static uint8_t  fr_state;                   /* 0 bind, 1 search, 2 link */
static uint32_t fr_id, fr_cand;
static uint8_t  fr_hits;                    /* bind: frames in a row from the candidate */
static bool     fr_binding;                 /* linked to a candidate, not yet stored */
static bool     fr_have, fr_fs_valid, fr_in_fs, fr_out_on, fr_btn_done, fr_btn_armed;
static uint8_t  fr_hop, fr_step;
static uint32_t fr_slot;                    /* start of the packet awaited on fr_hop */
static uint32_t fr_dwell_us, fr_bind_since, fr_last_rx, fr_btn_since, fr_misses, fr_frames, fr_lost, fr_bad, fr_parks, fr_late_max;
static uint16_t fr_out_us[8], fr_fs_us[8], fr_txfs_us[8];   /* fr_txfs_us: the transmitter's failsafe positions, 0 = hold */
static int8_t   fr_rssi;
static rx_record_t fr_rec;

bool fasst_rx_fresh(uint32_t now, uint32_t within_us) { return fr_last_rx != 0u && (now - fr_last_rx) <= within_us; }

static void fr_listen(uint8_t hop)
{
  fr_hop = hop;
  sl_rail_idle(fa_rail, SL_RAIL_IDLE, false);
  (void)sl_rail_start_rx(fa_rail, hop, NULL);
}

static void fr_set_state(uint8_t st, uint32_t now)
{
  fr_state = st; fasst_diag[FA_STEP] = st; fr_dwell_us = now;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (st == 0u) app_log("FA rx bind: waiting for a transmitter close by (%d dBm or stronger)\r\n", (int)FRX_BIND_RSSI_DBM);
  else if (st == 1u) app_log("FA rx searching for transmitter %08lX\r\n", (unsigned long)fr_id);
#endif
}

static void fr_start(bool bind_held)
{
  uint32_t now = fa_now();
  for (uint8_t i = 0; i < 8u; i++) { fr_out_us[i] = 1500u; fr_txfs_us[i] = 0u; }
  fr_out_us[2] = 1000u;                                      /* throttle low until heard */
  fr_misses = 0; fr_frames = 0; fr_lost = 0; fr_bad = 0; fr_parks = 0; fr_last_rx = 0; fr_late_max = 0;
  fr_in_fs = false; fr_out_on = false; fr_binding = false; fr_hits = 0; fr_cand = 0;
  fr_have = rx_store_load(RX_SLOT_FASST, &fr_rec) && fr_rec.bound;
  fr_id = fr_have ? (((uint32_t)fr_rec.hop[0] << 24) | ((uint32_t)fr_rec.hop[1] << 16) | ((uint32_t)fr_rec.hop[2] << 8) | fr_rec.hop[3]) : 0u;
  fr_fs_valid = fr_have && fr_rec.fs_valid;
  if (fr_fs_valid) for (uint8_t i = 0; i < 8u; i++) fr_fs_us[i] = fr_rec.fs_us[i];
  servo_out_init(); servo_out_source(fr_out_us);
  sbus_init();
  { uint16_t sz = FASST_RX_FIFO_BYTES;
    if (sl_rail_set_rx_fifo(fa_rail, fr_rxfifo, &sz) != SL_RAIL_STATUS_NO_ERROR || sz < FASST_RX_FIFO_BYTES) fasst_diag[FA_CFG_ERR] |= 8; }
  sl_rail_reset_fifo(fa_rail, false, true);
  sl_rail_state_transitions_t tr = { .success = SL_RAIL_RF_STATE_RX, .error = SL_RAIL_RF_STATE_RX };
  if (sl_rail_set_rx_transitions(fa_rail, &tr) != SL_RAIL_STATUS_NO_ERROR) fasst_diag[FA_CFG_ERR] |= 16;
  sl_rail_config_events(fa_rail, SL_RAIL_EVENT_RX_PACKET_RECEIVED, SL_RAIL_EVENT_RX_PACKET_RECEIVED);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("FA boot fasst v0.3 receiver (for a transmitter board of this kind; experimental): %s%s, failsafe %s; 36 channels from %lu Hz every %lu Hz; cfg_err=%ld\r\n",
          fr_have ? "bound" : "not bound", bind_held ? ", BIND held: binding" : "", fr_fs_valid ? "preset stored" : "hold",
          (unsigned long)FASST_BASE_HZ, (unsigned long)FASST_SPACING_HZ, (long)fasst_diag[FA_CFG_ERR]);
#endif
  fa_uart_us = now; fr_bind_since = now; fa_run = true;
  fr_set_state((bind_held || !fr_have) ? 0u : 1u, now);
  fr_listen(0);
}

static void fr_unlink(uint32_t now)                          /* the link is gone: bind again or search */
{
  bool to_bind = fr_binding || !fr_have;
  fr_binding = false; fr_hits = 0;
  fr_set_state(to_bind ? 0u : 1u, now);
  fr_listen(fr_hop);
}

static void fr_slot_lost(uint32_t now)                       /* nothing came in this slot */
{
  fr_frames++; fr_misses++; fr_lost++;
  (void)sbus_send(fr_out_us, true, fr_in_fs);
  if (fr_binding && fr_misses >= FRX_BIND_MISSES) { fr_unlink(now); return; }   /* the bind candidate went away */
  if (fr_misses == FRX_FS_FRAMES && !fr_in_fs) {             /* 1 s: failsafe */
    fr_in_fs = true;
    for (uint8_t i = 0; i < 8u; i++) {
      if (fr_fs_valid) fr_out_us[i] = fr_fs_us[i];
      else if (fr_txfs_us[i] != 0u) fr_out_us[i] = fr_txfs_us[i];
    }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("FA rx failsafe (1 s without a frame): %s\r\n", fr_fs_valid ? "stored preset" : "hold / the transmitter's values");
#endif
  }
  if (fr_misses >= FRX_LOST_FRAMES) { fr_unlink(now); return; }
  fr_slot += FASST_FRAME_US;
  if (fr_misses >= FRX_PARK_FRAMES) {                        /* lost for 0.2 s: wait for the transmitter on one channel */
    if (fr_misses == FRX_PARK_FRAMES) fr_parks++;
    if (((fr_misses - FRX_PARK_FRAMES) % FRX_PARK_MOVE) == (FRX_PARK_MOVE - 1u)) fr_listen((uint8_t)((fr_hop + 7u) % FASST_HOPS));
    return;
  }
  fr_listen(fasst_next_hop(fr_hop, fr_step));
}

void fasst_rx_process(void)                                  /* a packet is held in RAIL */
{
  for (;;) {
    sl_rail_rx_packet_info_t info;
    sl_rail_rx_packet_handle_t h = sl_rail_get_rx_packet_info(fa_rail, SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &info);
    if (h == SL_RAIL_RX_PACKET_HANDLE_INVALID) return;
    static uint8_t b[FASST_AIR_LEN];
    uint16_t n = info.packet_bytes;
    sl_rail_rx_packet_details_t det; memset(&det, 0, sizeof(det));
    det.time_received.time_position = SL_RAIL_PACKET_TIME_DEFAULT;
    (void)sl_rail_get_rx_packet_details(fa_rail, h, &det);
    det.time_received.total_packet_bytes = info.packet_bytes;
    (void)sl_rail_get_rx_time_sync_word_end(fa_rail, &det);
    if (n > FASST_AIR_LEN) { info.packet_bytes = FASST_AIR_LEN; }
    memset(b, 0, sizeof(b));
    sl_rail_copy_rx_packet(fa_rail, b, &info);
    sl_rail_release_rx_packet(fa_rail, h);
    if (!fa_run || !fa_rx) continue;
    uint32_t now = fa_now();

    uint8_t raw[FASST_PACKET_LEN], plain[FASST_PACKET_LEN], k, step; uint16_t late;
    if (n < FASST_AIR_LEN || !fasst_rx_decode(b, raw, &late) || !fasst_unpack(raw, plain, &k, &step)) { fr_bad++; continue; }
    uint32_t id = ((uint32_t)plain[0] << 24) | ((uint32_t)plain[1] << 16) | ((uint32_t)plain[2] << 8) | plain[3];
    if (k != fr_hop) { fr_bad++; continue; }                 /* not the channel we listen on */
    /* the packet's first chip: the sync word ended 32 chips (21 us) after it, later if the radio caught a repeat */
    uint32_t start = (uint32_t)det.time_received.packet_time - 21u - ((uint32_t)late * 2u) / 3u;
    if (late > fr_late_max) fr_late_max = late;

    /* ours? bind: a strong transmitter becomes the candidate, followed until FRX_BIND_FRAMES of its frames came */
    bool cand_new = false;
    if (fr_state == 0u) {
      if (det.rssi_dbm < FRX_BIND_RSSI_DBM) continue;
      fr_cand = id; fr_hits = 0; fr_binding = true; cand_new = true;
    } else if (fr_binding) {
      if (id != fr_cand) continue;
    } else if (id != fr_id) continue;                        /* another FASST transmitter */

    /* first of all the radio goes to the next hop: a log line or a flash write below takes longer than a frame */
    uint8_t was = fr_state;
    fr_step = step;
    fr_slot = start + FASST_FRAME_US;                        /* the next packet, one hop on */
    fr_state = 2u; fasst_diag[FA_STEP] = 2;
    fr_listen(fasst_next_hop(k, step));

    fr_rssi = det.rssi_dbm;
    fr_last_rx = now | 1u;
    fr_frames++; fr_misses = 0;
    if (fr_binding && ++fr_hits >= FRX_BIND_FRAMES) {
      fr_binding = false; fr_id = id;
      memset(&fr_rec, 0, sizeof(fr_rec)); fr_rec.bound = 1;
      fr_rec.hop[0] = (uint8_t)(id >> 24); fr_rec.hop[1] = (uint8_t)(id >> 16); fr_rec.hop[2] = (uint8_t)(id >> 8); fr_rec.hop[3] = (uint8_t)id;
      fr_fs_valid = false;                                   /* a new bind: the old failsafe preset goes (hold until a new one is stored) */
      fr_have = rx_store_save(RX_SLOT_FASST, &fr_rec);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("FA rx bound to transmitter %08lX (%s): hop %u, step %u, %d dBm\r\n", (unsigned long)id,
              fr_have ? "stored" : "NOT stored: flash write failed", (unsigned)k, (unsigned)step, (int)fr_rssi);
#endif
    }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    else if (was != 2u && !cand_new && !fr_binding)          /* a bind candidate is followed without a line: it would cost frames */
      app_log("FA rx linked to %08lX: hop %u, step %u, %d dBm\r\n", (unsigned long)fr_id, (unsigned)k, (unsigned)step, (int)fr_rssi);
#endif
    if (fr_in_fs) {
      fr_in_fs = false;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("FA rx link back\r\n");
#endif
    }

    if (!fr_binding) {                                       /* a bind candidate moves no servo */
      uint16_t v[FASST_CHANNELS], live, fsp, fsh;
      fasst_slots(plain, raw, v, &live, &fsp, &fsh);
      for (uint8_t i = 0; i < 8u; i++) {
        if (live & (1u << i)) fr_out_us[i] = fasst_us(v[i]);
        if (fsp & (1u << i)) fr_txfs_us[i] = fasst_us(v[i]);
        if (fsh & (1u << i)) fr_txfs_us[i] = 0u;
      }
      if (!fr_out_on) { fr_out_on = true; servo_out_enable(true); }
      (void)sbus_send(fr_out_us, false, false);
    }
  }
}

void fasst_rx_service(bool bind_held)                        /* every main-loop pass */
{
  if (!fa_run || !fa_rx) return;
  uint32_t now = fa_now();
  sbus_service();
  if (fr_state == 2u) {
    while (fr_state == 2u && (int32_t)(now - (fr_slot + FRX_AIR_US + FRX_SLACK_US)) > 0) fr_slot_lost(now);   /* the packet did not come */
    if (!bind_held) fr_btn_armed = true;                     /* the hold that bound us (BIND at power-up, cfg bind) is not a preset press */
    if (fr_state == 2u && !fr_binding && bind_held && fr_btn_armed) {   /* BIND pressed 0.3 s while linked: store the failsafe preset */
      if (fr_btn_since == 0u) fr_btn_since = now | 1u;
      else if (!fr_btn_done && (now - fr_btn_since) >= FRX_FS_PRESS_US && !fr_in_fs) {
        fr_btn_done = true;
        for (uint8_t i = 0; i < 8u; i++) fr_fs_us[i] = fr_out_us[i];
        fr_fs_valid = true;
        fr_rec.bound = 1;
        fr_rec.hop[0] = (uint8_t)(fr_id >> 24); fr_rec.hop[1] = (uint8_t)(fr_id >> 16); fr_rec.hop[2] = (uint8_t)(fr_id >> 8); fr_rec.hop[3] = (uint8_t)fr_id;
        fr_rec.fs_valid = 1; for (uint8_t i = 0; i < 8u; i++) fr_rec.fs_us[i] = fr_fs_us[i];
        bool ok = rx_store_save(RX_SLOT_FASST, &fr_rec);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("FA rx failsafe preset %s: %u %u %u %u %u %u %u %u\r\n", ok ? "stored" : "NOT stored", fr_fs_us[0], fr_fs_us[1], fr_fs_us[2], fr_fs_us[3],
                fr_fs_us[4], fr_fs_us[5], fr_fs_us[6], fr_fs_us[7]);
#else
        (void)ok;
#endif
      }
    } else { fr_btn_since = 0u; fr_btn_done = false; }
  } else {                                                   /* bind / search: the next channel after the dwell */
    if (!bind_held) fr_btn_armed = true;
    if ((now - fr_dwell_us) >= FRX_DWELL_US) { fr_dwell_us = now; fr_listen((uint8_t)((fr_hop + 7u) % FASST_HOPS)); }
    if (fr_state == 0u && fr_have && (now - fr_bind_since) >= FRX_BIND_TIMEOUT_US) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("FA rx no transmitter to bind to in 8 s: back to the stored one\r\n");
#endif
      fr_set_state(1u, now);
    }
  }

  /* LED_B: bind fast blink, search slow blink, link steady; LED_A toggles per 20 frames */
  static uint32_t tb = 0; static uint32_t la = 0;
  if (fr_state == 2u && !fr_in_fs && !fr_binding) sl_gpio_set_pin(&fa_led_b);
  else if ((now - tb) > (fr_state == 0u ? 100000u : 500000u)) { tb = now; sl_gpio_toggle_pin(&fa_led_b); }
  if (fr_frames / 20u != la) { la = fr_frames / 20u; sl_gpio_toggle_pin(&fa_led_a); }

#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if ((now - fa_uart_us) >= FASST_LOG_US) {                  /* every 5 s: the line itself costs a few frames */
    fa_uart_us = now;
    static uint32_t lf = 0, ll = 0;
    app_log("FA rx %s frames%lu lost%lu bad%lu parks%lu late%lu rssi%d hop%u step%u%s ch %u %u %u %u %u %u %u %u\r\n",
            fr_state == 0u ? "bind" : fr_state == 1u ? "search" : "link", (unsigned long)(fr_frames - lf), (unsigned long)(fr_lost - ll),
            (unsigned long)fr_bad, (unsigned long)fr_parks, (unsigned long)fr_late_max, (int)fr_rssi, (unsigned)fr_hop, (unsigned)fr_step,
            fr_in_fs ? " FAILSAFE" : "",
            fr_out_us[0], fr_out_us[1], fr_out_us[2], fr_out_us[3], fr_out_us[4], fr_out_us[5], fr_out_us[6], fr_out_us[7]);
    lf = fr_frames; ll = fr_lost;
  }
#endif
}
