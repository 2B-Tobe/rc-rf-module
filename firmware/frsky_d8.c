/*
 * frsky_d8.c — FrSky D8 transmitter / receiver with PPM out, version 2 (see frsky_d8.h).
 * Pulled into app_process.c with #include "frsky_d8.c" so the generated cmake
 * source list stays untouched.
 */
#include "frsky_d8.h"
#include "sl_core.h"
#include "servo_pins.h"
#include "rail_config.h"
#include <string.h>
#include "rx_store.h"

extern const RAIL_ChannelConfig_t frsky_d8_channelConfig;   /* autogen/rail_config.c */

/* ---- knobs -------------------------------------------------------------- */
#ifndef FRSKY_BIND_PIN
#define FRSKY_BIND_PIN         0       /* 1 once PB00 reads 3.3 V idle / 0 V shorted */
#endif
#ifndef FRSKY_BIND_SLOTS
#define FRSKY_BIND_SLOTS       167u    /* 167 x 9 ms = 1.5 s bind window (RX powered first) */
#endif
#ifndef FRSKY_BIND_POWER_DDBM
#define FRSKY_BIND_POWER_DDBM  0       /* 0 dBm: bind from a metre or two          */
#endif
#ifndef FRSKY_DATA_POWER_DDBM
#define FRSKY_DATA_POWER_DDBM  100     /* +10 dBm flight power (raise once proven) */
#endif
#ifndef FRSKY_RANGE_POWER_DDBM
#define FRSKY_RANGE_POWER_DDBM -100    /* -10 dBm range check (RANGE input low)    */
#endif
#ifndef FRSKY_PPM_CHANNELS
#define FRSKY_PPM_CHANNELS     8u      /* RX role: channels in the PPM frame      */
#endif
#define FRSKY_PPM_FRAME_US     22500u
#define FRSKY_PPM_PULSE_US     300u    /* low pulse; line idles high               */
#define FRSKY_FAILSAFE_MISSES  111u    /* RX: ~1 s without packets -> PPM stops    */
#define FRSKY_LOST_MISSES      333u    /* RX: ~3 s -> back to searching on hop[0]  */
#define FRSKY_REPLY_AFTER_US   1000u   /* RX: telemetry reply this long after the slot-2 packet end (31 kbit/s: a packet
                                         * takes 7.2 ms, so the reply must start early to be off the air in time) */
#define FRSKY_RX_MISS_US       600u    /* RX: a packet not ended this long after its expected end is lost; the next one
                                         * starts 1.8 ms after that end, so the receiver must be on its hop before */
#ifndef FRSKY_FS_DEFAULT
#define FRSKY_FS_DEFAULT       0       /* RX failsafe with no stored preset: 0 = hold, 1 = no pulses */
#endif
#define FRSKY_FS_PRESS_US      300000u /* BIND held this long while linked -> store failsafe preset */
#define FRSKY_BIND_TIMEOUT_US  8000000u /* BIND at boot: give up & revert to stored bind after this */
#define FRSKY_COMMIT_PKTS      20u      /* good data packets before a fresh bind is written to flash */
#ifndef FRSKY_RX_BIND_BUTTON
#define FRSKY_RX_BIND_BUTTON   1       /* 1 once PB00 is verified: BIND at boot = forget TX, BIND while linked = store preset */
#endif
#define FRSKY_SEARCH_ALT_US    300000u /* RX search: alternate hop[0] / bind channel this often */

/* PPM channel -> FrSky channel. FrSky receivers expect CH1..CH4 = A E T R.
 * Identity by default: PPM 1..8 -> CH1..CH8. */
static const uint8_t frsky_ch_map[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };

/* ---- protocol constants ------------------------------------------------- */
#define FRSKY_SLOT_US          9000u
#define FRSKY_TX_LEAD_US       1000u   /* build + schedule each packet this early */
#define FRSKY_TELEM_RX_OPEN_US 7500u   /* open RX this long after the slot-2 packet, and only once it is sent */
#define FRSKY_PKT_LEN          18u
#define FRSKY_HOP_COUNT        47u
#define FRSKY_BIND_CHANNEL     0u
#define FRSKY_COUNTER_MOD      188u
#define FRSKY_LINK_HOLD_US     500000u

/* ---- the CC2500 air format (v2, 25 Oct 2026) ------------------------------
 * A FrSky D8 transmitter or receiver is a CC2500 with its default sync word 0xD3 0x91 (15/16 bits checked), and
 * its CRC-16 (poly 0x8005, seed 0xFFFF, MSB first, over the length byte and the data, high byte first) after the
 * 18 bytes. v1 left both to the radio configurator: the generated sync register does not hold 0xD391 (two EFR32
 * boards heard each other, no FrSky receiver ever could). Now both are set here, independent of the PHY file:
 *   sync  sl_rail_config_sync_words() sends its value least-significant bit first, so D3 91 (each MSB first) is
 *         the 16-bit bit reversal of 0xD391 = 0x89CB
 *   CRC   TX: the radio's CRC-16 (poly 0x8005, seed 0xFFFF from the PHY file). RX keeps the two received CRC bytes
 *         (STORE_CRC, IGNORE_CRC_ERRORS) and checks them with the CC2500 CRC in software, and counts how often the
 *         radio's own check agreed (hwcrc in the FS line): an EFR32 receiver shows that way whether the radio's CRC
 *         is the CC2500's. (A software CRC on TX with SL_RAIL_TX_OPTION_REMOVE_CRC tripped RAIL assert 62, "TX CRC
 *         configuration is corrupt", after a few thousand packets: not used.) */
#define FRSKY_SYNC_ON_AIR      0x89CBu
/* Frequency tune (like Multiprotocol's FrSky "option"): bench 25 Oct 2026, a FrSky D8R-II Plus works from -20 kHz,
 * a RadioMaster R88 from -60 to +60 kHz. Transmitter only; the receiver's 250 kHz bandwidth needs none. */
#define FRSKY_D8_TUNE_HZ       (-20000)
#define FRSKY_CC2500_SPACING   299927u  /* CC2500 CHANSPC 0x7A at 26 MHz; RAIL's 300000 is 17 kHz off at ch 235 */
#define FRSKY_RX_END_LEAD_US   500u     /* the telemetry window closes this long before the next packet */
static uint32_t s_chg_n; static uint16_t s_chg_max; static uint16_t s_prev_v[8];   /* value changes between packets */
static RAIL_ChannelConfigEntry_t s_tune_ent;
static RAIL_ChannelConfig_t      s_tune_cfg;
#define FRSKY_AIR_LEN          (FRSKY_PKT_LEN + 2u)
static uint32_t s_crc_bad, s_hwcrc_agree;   /* diagnostics: CC2500-CRC mismatches; packets where radio and CC2500 CRC agree */
static uint16_t s_last_n, s_last_rxcrc, s_last_swcrc; static uint8_t s_last_hw;   /* the last packet, for the FS line */
static uint8_t  s_last_pkt[FRSKY_AIR_LEN];                                         /* its bytes */

/* Transmitter identity: id[1] must be 1 (Multi: "keep compatibility with
 * already bound RXs"); id[2], id[3] are ours, per module (chip EUI64) and per model
 * (per-model binding, rc_model in app_process.c). Receivers learn them at bind. */
static uint8_t frsky_id[4] = { 0x00, 0x01, 0x00, 0x00 };

static void frsky_make_id(void)
{
  uint32_t e = DEVINFO->EUI64L ^ DEVINFO->EUI64H;
  frsky_id[2] = (uint8_t)((uint8_t)e ^ (uint8_t)(e >> 16) ^ rc_model);
  frsky_id[3] = (uint8_t)((uint8_t)(e >> 8) ^ (uint8_t)(e >> 24) ^ (uint8_t)(rc_model * 37u));
  if ((frsky_id[2] | frsky_id[3]) == 0u) frsky_id[3] = 1u;
}

/* ---- state -------------------------------------------------------------- */
volatile int32_t frsky_diag[16];

static sl_rail_handle_t s_rail;
static const volatile uint16_t *s_ch_us;
static uint8_t  s_hop[50];
static uint8_t  s_pkt[FRSKY_PKT_LEN];
static uint8_t  s_rx_buf[FRSKY_PKT_LEN + 8u];
static frsky_phase_t s_phase;
static uint32_t s_next_us;
/* transmitter slot chain state */
static volatile bool s_chain_run, s_rx_window;
static volatile uint32_t s_chain_seen;               /* RAIL time of the chain's last step (watchdog) */
static uint32_t s_chain_t;                           /* the slot time of the packet now on the air */
static uint32_t s_resync_n, s_sched_fail;                       /* chain restarts */
static uint32_t s_slot;          /* slots elapsed in the current phase    */
static bool     s_bind_hold;     /* TX: BIND on in the serial frames (frsky_bind) */
static uint8_t  s_counter;       /* data-phase hop counter 0..187         */
static bool     s_rx_pending;
static uint32_t s_last_telem_us;
static int32_t  s_power_ddbm = FRSKY_DATA_POWER_DDBM;

/* ---- receiver state (rx_role) ---- */
typedef enum { D8RX_BIND = 0, D8RX_SEARCH, D8RX_TRACK } d8rx_t;
static bool     s_rx_role;
static d8rx_t   s_rxs;
static uint16_t s_bind_mask;             /* bits 0..9: hop-table chunks received     */
static uint8_t  s_rx_id2, s_rx_id3;
static uint8_t  s_bhop[50], s_bid3, s_bid2;   /* hop table + id being collected during bind */
static uint32_t s_bind_last_us;
static uint32_t s_expect_us;             /* frame-end time the next packet should have */
static bool     s_win_open;
static uint32_t s_misses;
static uint16_t s_out_us[8];             /* RX role: decoded channels for PPM */
static bool     s_reply_pending;
static volatile bool s_reply_txing;      /* RX role: telemetry reply on air (cleared by the TX-complete event) */
static volatile bool s_reply_done;       /* RX role: reply finished -> retune to the next hop from the main loop */
static uint32_t s_reply_at_us;
static uint8_t  s_reply_ch;
static volatile bool    s_isr_done;      /* the RAIL event already retuned / scheduled the reply for the newest packet */
static volatile uint8_t s_after_reply_ch; /* where to listen once the reply is off the air */
static int8_t   s_last_rssi;
static rx_record_t s_rec;               /* stored bind + failsafe preset (USERDATA flash) */
static uint32_t s_btn_since;
static rx_record_t s_saved;
static bool     s_have_saved, s_binding, s_provisional;
static uint16_t s_good;
static uint32_t s_bind_deadline;            /* BIND press start, 0 = not pressed             */
static bool     s_btn_done;
static bool     s_in_failsafe;
static uint32_t s_last_rx;                 /* time of the last valid data packet (dual receiver) */

/* Multi's default D8 hop table: (i*30) % 235, +1 at i = 3, 23, 47. */
static void frsky_init_hop(void)
{
  for (uint8_t i = 0; i < 50; i++) {
    uint8_t f = (uint8_t)((i * 0x1Eu) % 0xEBu);
    if (i == 3 || i == 23 || i == 47) f++;
    if (i > 47) f = 0;
    s_hop[i] = f;
  }
}

static uint16_t frsky_crc(const uint8_t *p, uint8_t n)          /* CC2500 CRC-16 */
{
  uint16_t c = 0xFFFFu;
  while (n--) {
    c ^= (uint16_t)(*p++) << 8;
    for (uint8_t k = 0; k < 8u; k++) c = (c & 0x8000u) ? (uint16_t)((c << 1) ^ 0x8005u) : (uint16_t)(c << 1);
  }
  return c;
}

/* send the 18 bytes + the CC2500 CRC as a 20-byte payload. The radio's own CRC (which is not the CC2500's,
 * bench 25 Oct: every packet hw-pass, never equal to the CC2500 CRC) follows as two extra bytes; a CC2500 reads
 * length byte 0x11 + 17 + its CRC and never looks at them. No SL_RAIL_TX_OPTION_REMOVE_CRC (assert 62). */
static uint32_t s_tx_when;          /* absolute RAIL time for the next packet (0 = now) */
static bool frsky_air_tx(uint16_t channel, const uint8_t *p)
{
  uint8_t b[FRSKY_AIR_LEN];
  memcpy(b, p, FRSKY_PKT_LEN);
  uint16_t c = frsky_crc(b, FRSKY_PKT_LEN);
  b[FRSKY_PKT_LEN] = (uint8_t)(c >> 8); b[FRSKY_PKT_LEN + 1u] = (uint8_t)c;
  sl_rail_idle(s_rail, SL_RAIL_IDLE, false);
  sl_rail_set_fixed_length(s_rail, FRSKY_AIR_LEN);
  sl_rail_write_tx_fifo(s_rail, b, FRSKY_AIR_LEN, true);
  uint32_t when = s_tx_when; s_tx_when = 0u;
  if (when != 0u && (int32_t)(when - (uint32_t)sl_rail_get_time(s_rail)) > 50) {   /* exactly on the 9 ms grid */
    sl_rail_scheduled_tx_config_t sc = { .when = when, .mode = SL_RAIL_TIME_ABSOLUTE,
                                         .tx_during_rx = SL_RAIL_SCHEDULED_TX_DURING_RX_ABORT_TX };
    if (sl_rail_start_scheduled_tx(s_rail, channel, SL_RAIL_TX_OPTIONS_DEFAULT, &sc, NULL) == SL_RAIL_STATUS_NO_ERROR) return true;
    s_sched_fail++;            /* refused (radio still leaving RX/TX): send now instead, so the chain gets its event */
  }
  return sl_rail_start_tx(s_rail, channel, SL_RAIL_TX_OPTIONS_DEFAULT, NULL) == SL_RAIL_STATUS_NO_ERROR;
}

/* Listen on a channel: 18 bytes + the two CRC bytes (STORE_CRC), checked against the CC2500 CRC in software. */
static sl_rail_status_t frsky_air_rx(uint16_t channel)
{
  sl_rail_idle(s_rail, SL_RAIL_IDLE, false);
  sl_rail_set_fixed_length(s_rail, FRSKY_PKT_LEN);
  return sl_rail_start_rx(s_rail, channel, NULL);
}

static void frsky_build_bind(uint32_t n)
{
  uint8_t idx = (uint8_t)((n % 10u) * 5u);
  memset(s_pkt, 0, FRSKY_PKT_LEN);
  s_pkt[0] = 0x11;
  s_pkt[1] = 0x03;
  s_pkt[2] = 0x01;
  s_pkt[3] = frsky_id[3];
  s_pkt[4] = frsky_id[2];
  s_pkt[5] = idx;
  for (uint8_t k = 0; k < 5; k++) s_pkt[6 + k] = s_hop[idx + k];
  s_pkt[17] = frsky_id[1];
}

/* 12-bit FrSky value = microseconds * 1.5 (1500 us -> 2250 = 0x8CA). */
static inline uint16_t frsky_val(uint16_t us)
{
  if (us < 900u)  us = 900u;
  if (us > 2100u) us = 2100u;
  return (uint16_t)((us * 3u) / 2u);
}

static void frsky_build_data(void)
{
  memset(s_pkt, 0, FRSKY_PKT_LEN);
  s_pkt[0] = 0x11;
  s_pkt[1] = frsky_id[3];
  s_pkt[2] = frsky_id[2];
  s_pkt[3] = s_counter;
  s_pkt[4] = 0x00;            /* telemetry user-frame counter, unused */
  s_pkt[5] = frsky_id[1];
  for (uint8_t i = 0; i < 8; i++) {
    uint16_t v = frsky_val(s_ch_us[frsky_ch_map[i]]);
    { uint16_t d = (v > s_prev_v[i]) ? (uint16_t)(v - s_prev_v[i]) : (uint16_t)(s_prev_v[i] - v);
      if (d) { s_chg_n++; if (d > s_chg_max) s_chg_max = d; } s_prev_v[i] = v; }
    if (i < 4) {
      s_pkt[6 + i] = (uint8_t)(v & 0xFF);
      s_pkt[10 + (i >> 1)] |= (uint8_t)(((v >> 8) & 0x0F) << (4 * (i & 1)));
    } else {
      s_pkt[8 + i] = (uint8_t)(v & 0xFF);
      s_pkt[16 + ((i - 4) >> 1)] |= (uint8_t)(((v >> 8) & 0x0F) << (4 * ((i - 4) & 1)));
    }
  }
}

static volatile bool s_tx_busy;     /* a packet is on the air (31 kbit/s: 7.2 ms) */
static void frsky_tx(uint16_t channel)
{
  s_tx_busy = true;
  if (!frsky_air_tx(channel, s_pkt)) { frsky_diag[F_TX_ERR]++; s_tx_busy = false; }   /* leaves the telemetry RX window first */
}

static void frsky_set_phase(frsky_phase_t p)
{
  s_phase = p;
  frsky_diag[F_PHASE] = p;
  s_slot = 0;
  s_counter = 0;
  s_rx_pending = false;
  sl_rail_set_tx_power_dbm(s_rail, (p == FRSKY_PHASE_BIND) ? FRSKY_BIND_POWER_DDBM : s_power_ddbm);
}

/* ---- PPM + 8 servo pulses: servo_out.c (TIMER2) reads s_out_us ---- */
static void frsky_ppm_enable(bool on) { servo_out_enable(on); }
bool frsky_rx_fresh(uint32_t now, uint32_t within_us) { return s_last_rx != 0u && (now - s_last_rx) <= within_us; }

void frsky_start(sl_rail_handle_t rh, const volatile uint16_t *ch_us, bool bind_held, bool rx_role)
{
  frsky_make_id();
  s_rail = rh;
  s_ch_us = ch_us;
  s_rx_role = rx_role;
  for (int i = 0; i < 16; i++) frsky_diag[i] = 0;
  frsky_diag[F_ROLE] = rx_role ? 1 : 0;
  frsky_init_hop();

  int32_t tune = rx_role ? 0 : FRSKY_D8_TUNE_HZ;
  sl_rail_idle(s_rail, SL_RAIL_IDLE_ABORT, true);
  s_tune_ent = frsky_d8_channelConfig.configs[0];   /* RAM copy: the CC2500's channel spacing, + the tune on a TX */
  s_tune_ent.baseFrequency = (uint32_t)((int32_t)frsky_d8_channelConfig.configs[0].baseFrequency + tune);
  s_tune_ent.channelSpacing = FRSKY_CC2500_SPACING;
  s_tune_cfg = frsky_d8_channelConfig;
  s_tune_cfg.configs = &s_tune_ent; s_tune_cfg.length = 1u;
  if (sl_rail_config_channels(s_rail, (const sl_rail_channel_config_t *)&s_tune_cfg, NULL)
      != SL_RAIL_STATUS_NO_ERROR) { frsky_diag[F_CFG_ERR] |= 1; }
  if (sl_rail_set_fixed_length(s_rail, FRSKY_PKT_LEN) != FRSKY_PKT_LEN) { frsky_diag[F_CFG_ERR] |= 2; }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("FS D8 %s: channel 0 = %lu Hz, spacing %lu Hz\r\n", rx_role ? "receiver" : "transmitter",
          (unsigned long)s_tune_ent.baseFrequency, (unsigned long)FRSKY_CC2500_SPACING);
#endif
  if (sl_rail_config_rx_options(s_rail, SL_RAIL_RX_OPTION_STORE_CRC | SL_RAIL_RX_OPTION_IGNORE_CRC_ERRORS,
                                SL_RAIL_RX_OPTION_STORE_CRC | SL_RAIL_RX_OPTION_IGNORE_CRC_ERRORS) != SL_RAIL_STATUS_NO_ERROR)
    frsky_diag[F_CFG_ERR] |= 32;
  s_crc_bad = 0; s_hwcrc_agree = 0;
  sl_rail_config_events(s_rail, SL_RAIL_EVENT_RX_SYNC_0_DETECT | SL_RAIL_EVENT_RX_PACKET_RECEIVED,
                                SL_RAIL_EVENT_RX_SYNC_0_DETECT | SL_RAIL_EVENT_RX_PACKET_RECEIVED);

  s_next_us = (uint32_t)sl_rail_get_time(s_rail);
  s_last_telem_us = s_next_us - FRSKY_LINK_HOLD_US;
  s_chain_run = false; s_rx_window = false;
  if (!rx_role)
    sl_rail_config_events(s_rail, SL_RAIL_EVENT_RX_SCHEDULED_RX_END | SL_RAIL_EVENTS_TX_COMPLETION,
                                  SL_RAIL_EVENT_RX_SCHEDULED_RX_END | SL_RAIL_EVENTS_TX_COMPLETION);
  if (rx_role) {                                  /* ---- D8 receiver with PPM out ---- */
    frsky_set_phase(FRSKY_PHASE_RX);
    s_rxs = D8RX_BIND; s_bind_mask = 0; s_misses = 0; s_win_open = false;
    s_reply_pending = false; s_in_failsafe = false; s_btn_since = 0; s_btn_done = false; s_last_rx = 0;
    s_reply_txing = false; s_reply_done = false;
    sbus_init();
    s_provisional = false; s_good = 0; s_binding = false;
    for (uint8_t i = 0; i < 8; i++) s_out_us[i] = 1500u;
    if (bind_held) frsky_diag[F_CFG_ERR] |= 128;                     /* [6] bit 128: BIND read low at boot */
    s_have_saved = (rx_store_load(RX_SLOT_D8, &s_saved) && s_saved.bound);
    if (bind_held) {                        /* BIND low at boot: try to (re)bind, keep stored as fallback */
      s_rxs = D8RX_BIND; s_bind_mask = 0; s_binding = true;
      s_bind_deadline = s_next_us + FRSKY_BIND_TIMEOUT_US;
      frsky_diag[F_PHASE] = 0; frsky_diag[F_BIND_SENT] = 0;
      memset(&s_rec, 0, sizeof(s_rec));
    } else if (s_have_saved) {
      s_rec = s_saved;
      memcpy(s_hop, s_rec.hop, 50); s_rx_id2 = s_rec.id2; s_rx_id3 = s_rec.id3;
      s_rxs = D8RX_SEARCH; frsky_diag[F_PHASE] = 10; frsky_diag[F_BIND_SENT] = 7;   /* 7 = loaded from flash */
    } else memset(&s_rec, 0, sizeof(s_rec));
    servo_out_init(); servo_out_source(s_out_us);                 /* pulses: servo_out.c */
    if (frsky_air_rx((s_rxs == D8RX_BIND) ? FRSKY_BIND_CHANNEL : s_hop[0]) != SL_RAIL_STATUS_NO_ERROR) { frsky_diag[F_CFG_ERR] |= 8; }
    return;
  }
#if FRSKY_BIND_PIN
  frsky_set_phase(bind_held ? FRSKY_PHASE_BIND : FRSKY_PHASE_DATA);
#else
  (void)bind_held;
  frsky_set_phase(FRSKY_PHASE_BIND);
#endif
}


/* ---- receiver: window scheduling, misses, telemetry reply ---- */
static volatile uint32_t s_reply_t0;    /* RAIL time a telemetry reply was started (stuck-reply watchdog) */
/* Main-loop retune. Done with interrupts held off, so the RAIL event handler (frsky_on_rx_held, which also
 * retunes and schedules replies) can never run in the middle of it; a reply still pending is dropped. */
static void frsky_rx_listen(uint16_t ch)
{
  CORE_DECLARE_IRQ_STATE; CORE_ENTER_CRITICAL();
  if (s_reply_txing) { s_reply_txing = false; sl_rail_idle(s_rail, SL_RAIL_IDLE_ABORT, false); }
  (void)frsky_air_rx(ch);
  CORE_EXIT_CRITICAL();
}

/* Step to the next slot the transmitter will use (slot 3 of every four is silent). */
static void frsky_rx_advance(void)
{
  s_counter = (uint8_t)((s_counter + 1u) % FRSKY_COUNTER_MOD);
  s_expect_us += FRSKY_SLOT_US;
  if ((s_counter & 3u) == 3u) {                   /* telemetry slot: TX sends nothing */
    s_counter = (uint8_t)((s_counter + 1u) % FRSKY_COUNTER_MOD);
    s_expect_us += FRSKY_SLOT_US;
  }
  s_win_open = false;
}

/* Link lost for FRSKY_FAILSAFE_MISSES slots: apply the failsafe. */
static void frsky_rx_failsafe(void)
{
  s_in_failsafe = true;
  if (s_rec.fs_valid) { memcpy(s_out_us, s_rec.fs_us, sizeof(s_out_us)); }        /* preset */
  else if (FRSKY_FS_DEFAULT == 1) { frsky_ppm_enable(false); }                     /* no pulses */
  /* else hold: outputs keep the last values */
}

/* BIND held >300 ms while the link is up: store the current positions as the failsafe preset. */
static void frsky_rx_button(bool bind_held, uint32_t now)
{
  if (!FRSKY_RX_BIND_BUTTON || !bind_held || s_binding) { s_btn_since = 0; s_btn_done = false; return; }
  if (s_btn_since == 0) { s_btn_since = now ? now : 1u; return; }
  if (s_btn_done || (now - s_btn_since) < FRSKY_FS_PRESS_US) return;
  s_btn_done = true;
  if (s_rxs != D8RX_TRACK || s_in_failsafe) return;
  memcpy(s_rec.fs_us, s_out_us, sizeof(s_rec.fs_us)); s_rec.fs_valid = 1;
  if (rx_store_save(RX_SLOT_D8, &s_rec)) frsky_diag[F_BIND_SENT] += 100;          /* [2] += 100 per stored preset */
}

static void frsky_rx_service(bool bind_held)
{
  sbus_service();
  uint32_t now = (uint32_t)sl_rail_get_time(s_rail);
  if (s_binding && (int32_t)(now - s_bind_deadline) >= 0) {   /* bind gave up: revert to the stored bind */
    s_binding = false; s_provisional = false;
    if (s_have_saved) {
      s_rec = s_saved; memcpy(s_hop, s_rec.hop, 50); s_rx_id2 = s_rec.id2; s_rx_id3 = s_rec.id3;
      s_rxs = D8RX_SEARCH; s_win_open = false; frsky_diag[F_PHASE] = 10; frsky_diag[F_BIND_SENT] = 7;
    }
  }
  frsky_rx_button(bind_held, now);

  if (s_reply_pending && (int32_t)(now - s_reply_at_us) >= 0) {
    s_reply_pending = false;
    uint8_t r[FRSKY_PKT_LEN];
    memset(r, 0, FRSKY_PKT_LEN);
    r[0] = 0x11; r[1] = s_rx_id3; r[2] = s_rx_id2;
    r[3] = 0; r[4] = 0;                                            /* A1, A2: no ADC yet */
    int32_t q = (int32_t)s_last_rssi + 110; if (q < 0) q = 0; if (q > 255) q = 255;
    r[5] = (uint8_t)q;                                             /* RSSI, dBm + 110 */
    CORE_DECLARE_IRQ_STATE; CORE_ENTER_CRITICAL();
    sl_rail_idle(s_rail, SL_RAIL_IDLE, false);
    sl_rail_set_tx_power_dbm(s_rail, FRSKY_DATA_POWER_DDBM);
    if (!frsky_air_tx(s_reply_ch, r))
      frsky_diag[F_TX_ERR]++;
    else { frsky_diag[F_DATA_SENT]++; s_reply_txing = true; s_reply_t0 = now; }
    CORE_EXIT_CRITICAL();
    s_win_open = false;
    return;
  }
  if (s_reply_txing && (now - s_reply_t0) > 15000u) {   /* a reply never reported back (cancelled): listen again */
    frsky_diag[F_TX_ERR]++;
    if (s_rxs == D8RX_TRACK) { frsky_rx_listen(s_after_reply_ch); s_win_open = true; }
    else s_reply_txing = false;
  }
  if (s_reply_done) {                     /* reply is off the air: sit on the next hop right away */
    s_reply_done = false;
    if (s_rxs == D8RX_TRACK && !s_win_open) { frsky_rx_listen(s_hop[s_counter % FRSKY_HOP_COUNT]); s_win_open = true; }
  }

  if (s_rxs == D8RX_BIND && s_rec.bound && (now - s_bind_last_us) > 2000000u) {   /* rebind stalled: keep the stored TX */
    s_rxs = D8RX_SEARCH; frsky_diag[F_PHASE] = 10; s_bind_last_us = now; s_win_open = false;
  }
  if (s_rxs == D8RX_SEARCH) {                                      /* bound, TX not yet reacquired: keep RX on hop[0] */
    static uint32_t s_arm_us;
    if (!s_win_open || (now - s_arm_us) > 100000u) {
      sl_rail_idle(s_rail, SL_RAIL_IDLE_ABORT, true);           /* whatever state the radio was left in */
      frsky_rx_listen(s_hop[0]); s_win_open = true; s_arm_us = now;
    }
    return;
  }
  if (s_rxs != D8RX_TRACK) return;
  if (!s_win_open && (int32_t)(now - (s_expect_us - 2000u)) >= 0) {   /* fallback only: normally already listening */
    frsky_rx_listen(s_hop[s_counter % FRSKY_HOP_COUNT]);
    s_win_open = true;
  }
  if ((int32_t)(now - (s_expect_us + FRSKY_RX_MISS_US)) >= 0) {   /* missed this slot */
    s_misses++; frsky_diag[F_EV_SYNC] = (int32_t)s_misses;
    frsky_rx_advance();
    if (s_misses == FRSKY_FAILSAFE_MISSES) frsky_rx_failsafe();
    (void)sbus_send(s_out_us, true, s_in_failsafe);
    if (s_misses >= FRSKY_LOST_MISSES) { s_rxs = D8RX_SEARCH; s_win_open = false; frsky_rx_listen(s_hop[0]); }
    else { frsky_rx_listen(s_hop[s_counter % FRSKY_HOP_COUNT]); s_win_open = true; }   /* next hop right away */
  }
}

/* ---- the transmitter's slot chain, driven by radio events (interrupt context) ----------------------
 * Every packet is scheduled at an absolute RAIL time on the 9 ms grid. When it has gone (TX completion event) the
 * next one is built and scheduled. After the third data packet the radio listens for the receiver's telemetry on
 * the next hop in a scheduled RX window that ends 1 ms before the following packet; that window's end event
 * schedules the packet. Same order and timing as Multiprotocol's FRSKYD_callback. */

static void frsky_slot_tx(uint32_t t)                /* build and schedule the packet of the slot at time t */
{
  s_chain_t = t; s_chain_seen = (uint32_t)sl_rail_get_time(s_rail);
  if (s_phase == FRSKY_PHASE_DATA && s_bind_hold && s_slot >= FRSKY_BIND_SLOTS && (s_slot & 3u) == 0u)
    frsky_set_phase(FRSKY_PHASE_BIND);               /* BIND still on: the next bind turn */
  if (s_phase == FRSKY_PHASE_BIND) {
    frsky_build_bind(s_slot);
    s_tx_when = t | 1u; frsky_tx(FRSKY_BIND_CHANNEL);
    frsky_diag[F_BIND_SENT]++;
    s_slot++;
    if (s_slot >= FRSKY_BIND_SLOTS) frsky_set_phase(FRSKY_PHASE_DATA);
    return;
  }
  frsky_build_data();
  s_tx_when = t | 1u; frsky_tx(s_hop[s_counter % FRSKY_HOP_COUNT]);
  frsky_diag[F_DATA_SENT]++;
  s_counter = (uint8_t)((s_counter + 1u) % FRSKY_COUNTER_MOD);
  s_slot++;
}

static void frsky_chain_tx_done(void)                /* the packet of slot s_chain_t has gone */
{
  uint32_t t = s_chain_t + FRSKY_SLOT_US;
  if (s_phase == FRSKY_PHASE_DATA && (s_slot & 3u) == 3u) {   /* next slot is the receiver's: listen there */
    uint16_t ch = s_hop[s_counter % FRSKY_HOP_COUNT];
    s_counter = (uint8_t)((s_counter + 1u) % FRSKY_COUNTER_MOD);
    s_slot++;
    s_chain_t = t;                                   /* the packet after it goes at t + 9 ms */
    sl_rail_idle(s_rail, SL_RAIL_IDLE, false);
    sl_rail_set_fixed_length(s_rail, FRSKY_PKT_LEN);
    sl_rail_scheduled_rx_config_t rc = { .start = 0, .start_mode = SL_RAIL_TIME_DELAY,
                                         .end = t + FRSKY_SLOT_US - FRSKY_RX_END_LEAD_US, .end_mode = SL_RAIL_TIME_ABSOLUTE,
                                         .rx_transition_end_schedule = 0, .hard_window_end = 1 };
    s_rx_window = true;
    if (sl_rail_start_scheduled_rx(s_rail, ch, &rc, NULL) != SL_RAIL_STATUS_NO_ERROR) {
      s_rx_window = false; frsky_slot_tx(t + FRSKY_SLOT_US);
    }
    s_chain_seen = (uint32_t)sl_rail_get_time(s_rail);
    return;
  }
  frsky_slot_tx(t);
}

/* BIND from the serial frames on the transmitter. While it is selected, FRSKY_BIND_SLOTS of bind (1.5 s) and as many
 * slots of flying data take turns: a receiver can be put into bind at any time, and Tobe's own EFR32 FrSky
 * receiver, which keeps a new bind only after hearing flying data within 8 s of it, stores it in a data turn.
 * A FrSky receiver just takes the hop table in a bind turn. Leaving BIND ends the turns after the current one. */
void frsky_bind(bool held) { s_bind_hold = held; }

void frsky_service(bool bind_held, bool range_low)
{
  if (s_phase == FRSKY_PHASE_RX) { frsky_rx_service(bind_held); return; }

  uint32_t now = (uint32_t)sl_rail_get_time(s_rail);

  /* flight power follows the RANGE input */
  int32_t want = range_low ? FRSKY_RANGE_POWER_DDBM : FRSKY_DATA_POWER_DDBM;
  if (want != s_power_ddbm) {
    s_power_ddbm = want;
    if (s_phase == FRSKY_PHASE_DATA) sl_rail_set_tx_power_dbm(s_rail, s_power_ddbm);
  }

  /* the slots run from the radio's own events (frsky_chain_*), so nothing in the main loop - log
   * printing, serial frames - can delay a packet. The main loop only (re)starts the chain if it ever stops. */
  if (!s_chain_run || (int32_t)(now - s_chain_seen) > (int32_t)(4u * FRSKY_SLOT_US)) {
    if (s_chain_run) s_resync_n++;
    s_chain_run = true; s_chain_seen = now;
    CORE_DECLARE_IRQ_STATE; CORE_ENTER_CRITICAL();
    frsky_slot_tx(now + 2000u);
    CORE_EXIT_CRITICAL();
  }
  (void)bind_held;
}

frsky_phase_t frsky_phase(void) { return s_phase; }
uint32_t frsky_crc_bad(void) { return s_crc_bad; }
void frsky_tx_stats(uint32_t *n, uint32_t *worst, uint32_t *resync, uint32_t *sched_fail) { *n = s_chg_n; *worst = (uint32_t)s_chg_max * 2u / 3u; *resync = s_resync_n; *sched_fail = s_sched_fail; s_chg_n = 0; s_chg_max = 0; }   /* channel value changes, biggest step in us, slot chain restarts */
uint32_t frsky_hwcrc_agree(void) { return s_hwcrc_agree; }
void frsky_crc_last(uint16_t *n, uint8_t *hw, uint16_t *rx, uint16_t *sw) { *n = s_last_n; *hw = s_last_hw; *rx = s_last_rxcrc; *sw = s_last_swcrc; }
const uint8_t *frsky_last_pkt(void) { return s_last_pkt; }

int8_t frsky_tele_rssi_dbm(void) { return s_last_rssi; }

bool frsky_link_up(void)
{
  if (s_rx_role) return (s_rxs == D8RX_TRACK) && (s_misses < FRSKY_FAILSAFE_MISSES);
  uint32_t now = (uint32_t)sl_rail_get_time(s_rail);
  return (now - s_last_telem_us) < FRSKY_LINK_HOLD_US;
}

void frsky_on_events(sl_rail_events_t events)
{
  if (events & SL_RAIL_EVENTS_TX_COMPLETION)  s_tx_busy = false;
  if (!s_rx_role && s_chain_run && s_phase != FRSKY_PHASE_RX) {                          /* the slot chain */
    if (s_rx_window && (events & SL_RAIL_EVENT_RX_SCHEDULED_RX_END)) { s_rx_window = false; frsky_slot_tx(s_chain_t + FRSKY_SLOT_US); }
    else if (!s_rx_window && (events & SL_RAIL_EVENTS_TX_COMPLETION)) frsky_chain_tx_done();
  }
  if (events & SL_RAIL_EVENT_TX_PACKET_SENT)   frsky_diag[F_TX_OK]++;
  else if (events & SL_RAIL_EVENTS_TX_COMPLETION) frsky_diag[F_TX_ERR]++;
  if (s_reply_txing && (events & SL_RAIL_EVENTS_TX_COMPLETION)) {
    s_reply_txing = false; s_reply_done = true;
    if (s_rxs == D8RX_TRACK) { (void)frsky_air_rx(s_after_reply_ch); s_win_open = true; }   /* next hop at once */
  }
  if (!s_rx_role && (events & SL_RAIL_EVENT_RX_SYNC_0_DETECT)) frsky_diag[F_EV_SYNC]++;   /* RX role: [15] = misses */
}

/* RX role, called from the RAIL event handler right after a received packet is held. At 31 kbit/s the next packet
 * starts 1.8 ms after this one ends, which the main loop cannot be trusted to meet: a good data packet from our
 * transmitter is checked here (peek, CC2500 CRC), the radio goes straight to the next hop, and after the third
 * packet of four the telemetry reply is scheduled 1 ms on. The main loop still decodes the packet as before. */
void frsky_on_rx_held(void)
{
  if (!s_rx_role || s_rxs != D8RX_TRACK || s_reply_txing) return;
  sl_rail_rx_packet_info_t info;
  sl_rail_rx_packet_handle_t h = sl_rail_get_rx_packet_info(s_rail, SL_RAIL_RX_PACKET_HANDLE_NEWEST, &info);
  if (h == SL_RAIL_RX_PACKET_HANDLE_INVALID) return;
  uint8_t b[FRSKY_AIR_LEN];
  if (sl_rail_peek_rx_packet(s_rail, h, b, FRSKY_AIR_LEN, 0) < FRSKY_AIR_LEN) return;
  if (b[0] != 0x11u || b[1] != s_rx_id3 || b[2] != s_rx_id2) return;
  if (frsky_crc(b, FRSKY_PKT_LEN) != (uint16_t)((b[FRSKY_PKT_LEN] << 8) | b[FRSKY_PKT_LEN + 1u])) return;
  uint8_t c = (uint8_t)(b[3] % FRSKY_COUNTER_MOD);
  uint8_t n = (uint8_t)((c + 1u) % FRSKY_COUNTER_MOD);
  if ((n & 3u) == 3u) n = (uint8_t)((n + 1u) % FRSKY_COUNTER_MOD);   /* the TX skips slot 3 */
  uint32_t now = (uint32_t)sl_rail_get_time(s_rail);
  if ((c & 3u) == 2u) {                                            /* slot 3 is ours: the telemetry reply */
    uint8_t r[FRSKY_PKT_LEN];
    memset(r, 0, FRSKY_PKT_LEN);
    r[0] = 0x11; r[1] = s_rx_id3; r[2] = s_rx_id2;
    int32_t q = (int32_t)s_last_rssi + 110; if (q < 0) q = 0; if (q > 255) q = 255;
    r[5] = (uint8_t)q;
    s_after_reply_ch = s_hop[n % FRSKY_HOP_COUNT];
    s_tx_when = (now + FRSKY_REPLY_AFTER_US) | 1u;
    if (frsky_air_tx(s_hop[(c + 1u) % FRSKY_HOP_COUNT], r)) { s_reply_txing = true; s_reply_t0 = now; frsky_diag[F_DATA_SENT]++; }
    else { frsky_diag[F_TX_ERR]++; (void)frsky_air_rx(s_after_reply_ch); }
  } else {
    (void)frsky_air_rx(s_hop[n % FRSKY_HOP_COUNT]);
  }
  s_isr_done = true;
}

/* Decode one 12-bit channel value back to microseconds (loopback diagnostics). */
static uint16_t frsky_decode_ch(const uint8_t *p, uint8_t i)
{
  uint16_t v;
  if (i < 4) v = (uint16_t)(p[6 + i] | ((p[10 + (i >> 1)] >> (4 * (i & 1))) & 0x0F) << 8);
  else       v = (uint16_t)(p[8 + i] | ((p[16 + ((i - 4) >> 1)] >> (4 * ((i - 4) & 1))) & 0x0F) << 8);
  return (uint16_t)((v * 2u) / 3u);
}

static void frsky_rx_one(void)
{
  sl_rail_rx_packet_info_t info;
  sl_rail_rx_packet_handle_t h =
    sl_rail_get_rx_packet_info(s_rail, SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &info);
  if (h == SL_RAIL_RX_PACKET_HANDLE_INVALID) return;

  uint16_t n = info.packet_bytes;
  if (n > sizeof(s_rx_buf)) n = sizeof(s_rx_buf);
  sl_rail_rx_packet_details_t det; memset(&det, 0, sizeof(det));
  (void)sl_rail_get_rx_packet_details(s_rail, h, &det);
  (void)sl_rail_get_rx_time_frame_end(s_rail, &det);              /* timestamp = end of packet */
  sl_rail_copy_rx_packet(s_rail, s_rx_buf, &info);
  sl_rail_release_rx_packet(s_rail, h);
  { /* compare the radio's CRC verdict with the CC2500 CRC over bytes 0..17; accept on the radio's verdict */
    uint16_t sw = frsky_crc(s_rx_buf, FRSKY_PKT_LEN);
    uint16_t rc = (n >= FRSKY_AIR_LEN) ? (uint16_t)((s_rx_buf[FRSKY_PKT_LEN] << 8) | s_rx_buf[FRSKY_PKT_LEN + 1u]) : 0xFFFFu;
    s_last_n = n; s_last_hw = det.crc_passed ? 1u : 0u; s_last_rxcrc = rc; s_last_swcrc = sw;
    memset(s_last_pkt, 0, sizeof s_last_pkt); memcpy(s_last_pkt, s_rx_buf, n < FRSKY_AIR_LEN ? n : FRSKY_AIR_LEN);
    bool cc_ok = (n >= FRSKY_AIR_LEN) && (sw == rc);
    if (cc_ok) { if (det.crc_passed) s_hwcrc_agree++; } else s_crc_bad++;
    if (!cc_ok && !(n >= FRSKY_PKT_LEN && det.crc_passed)) {      /* CC2500 CRC (FrSky) or the radio's (older TX) */
      if (s_phase == FRSKY_PHASE_RX && s_rxs != D8RX_TRACK) frsky_rx_listen(s_rxs == D8RX_BIND ? FRSKY_BIND_CHANNEL : s_hop[0]);
      return;
    }
  }
  n = FRSKY_PKT_LEN;
  frsky_diag[F_RX_PACKETS]++;
  frsky_diag[F_RX_RSSI] = sl_rail_get_rssi(s_rail, 0);

  if (s_phase == FRSKY_PHASE_RX) {
    if (n != FRSKY_PKT_LEN || s_rx_buf[0] != 0x11) {
      frsky_diag[F_RX_ODD]++;
      if (s_rxs != D8RX_TRACK) frsky_rx_listen(s_rxs == D8RX_BIND ? FRSKY_BIND_CHANNEL : s_hop[0]);
      return;
    }
    s_last_rssi = det.rssi_dbm;

    if (s_rx_buf[1] == 0x03 && s_rx_buf[2] == 0x01) {              /* bind packet */
      frsky_diag[F_RX_BIND]++;
      if (s_rxs == D8RX_BIND) {
        uint8_t idx = s_rx_buf[5];
        s_bind_last_us = (uint32_t)sl_rail_get_time(s_rail);
        if (s_bind_mask == 0 || s_rx_buf[3] != s_bid3 || s_rx_buf[4] != s_bid2) {   /* new table: remember whose */
          s_bind_mask = 0; s_bid3 = s_rx_buf[3]; s_bid2 = s_rx_buf[4];
        }
        if (idx <= 45u && (idx % 5u) == 0u) {
          for (uint8_t k = 0; k < 5; k++) s_bhop[idx + k] = s_rx_buf[6 + k];
          s_bind_mask |= (uint16_t)(1u << (idx / 5u));
          if (s_bind_mask == 0x3FFu) {                             /* whole table: adopt provisionally */
            memcpy(s_hop, s_bhop, 50);
            s_rx_id3 = s_bid3; s_rx_id2 = s_bid2;
            memset(&s_rec, 0, sizeof(s_rec));
            memcpy(s_rec.hop, s_hop, 50); s_rec.id2 = s_rx_id2; s_rec.id3 = s_rx_id3; s_rec.bound = 1;
            s_provisional = true; s_good = 0;
            s_bind_deadline = (uint32_t)sl_rail_get_time(s_rail) + FRSKY_BIND_TIMEOUT_US;
            s_rxs = D8RX_SEARCH; s_win_open = false; frsky_diag[F_PHASE] = 10;
            frsky_rx_listen(s_hop[0]); return;
          }
        }
        frsky_rx_listen(FRSKY_BIND_CHANNEL);
      }
      return;
    }
    if (s_rxs == D8RX_BIND || s_rx_buf[1] != s_rx_id3 || s_rx_buf[2] != s_rx_id2) {
      frsky_diag[F_RX_ODD]++;
      if (s_rxs != D8RX_TRACK) frsky_rx_listen(s_rxs == D8RX_BIND ? FRSKY_BIND_CHANNEL : s_hop[0]);
      return;
    }
    /* data packet from our transmitter: the counter tells us where it is in the hop cycle */
    frsky_diag[F_RX_DATA]++;
    uint8_t c = (uint8_t)(s_rx_buf[3] % FRSKY_COUNTER_MOD);
    frsky_diag[F_RX_COUNTER] = c;
    uint32_t t = (uint32_t)det.time_received.packet_time;         /* end of this packet */
    s_misses = 0; frsky_diag[F_EV_SYNC] = 0;
    s_rxs = D8RX_TRACK; frsky_diag[F_PHASE] = 11; s_in_failsafe = false; s_win_open = false;
    if (s_provisional) {
      if (++s_good >= FRSKY_COMMIT_PKTS) {
        s_provisional = false; s_binding = false;
        if (rx_store_save(RX_SLOT_D8, &s_rec)) { s_saved = s_rec; s_have_saved = true; frsky_diag[F_BIND_SENT] = 1; }
      }
    } else s_binding = false;
    for (uint8_t i = 0; i < 8; i++) s_out_us[i] = frsky_decode_ch(s_rx_buf, i);
    s_last_rx = (uint32_t)sl_rail_get_time(s_rail);
    frsky_diag[F_RX_CH0_US] = s_out_us[0]; frsky_diag[F_RX_CH1_US] = s_out_us[1];
    frsky_ppm_enable(true);
    (void)sbus_send(s_out_us, false, s_in_failsafe);
    s_counter = c;
    s_expect_us = t;
    bool isr = s_isr_done; s_isr_done = false;                     /* the event handler already acted on it */
    if ((c & 3u) == 2u && !isr) {                                  /* slot 3 is ours: reply on the next hop */
      s_reply_pending = true;
      s_reply_at_us = t + FRSKY_REPLY_AFTER_US;
      s_reply_ch = s_hop[(c + 1u) % FRSKY_HOP_COUNT];
    }
    frsky_rx_advance();                                            /* -> next TX slot (skips slot 3) */
    /* Listen on the next hop immediately (as Multi's FrSky RX / FrSky receivers do) instead of a
     * narrow window just before the expected time: a packet that arrives early or late is still
     * caught, so one late TX packet can no longer drag the receiver out of phase. When a telemetry
     * reply is due, the retune happens once the reply is off the air (s_reply_done). */
    if (isr) { s_win_open = true; return; }
    if (!s_reply_pending) { frsky_rx_listen(s_hop[s_counter % FRSKY_HOP_COUNT]); s_win_open = true; }
    else s_after_reply_ch = s_hop[s_counter % FRSKY_HOP_COUNT];
    return;
  }

  /* transmitter: telemetry reply  11 id3 id2 A1 A2 RSSI hub... */
  if (n == FRSKY_PKT_LEN && s_rx_buf[0] == 0x11
      && s_rx_buf[1] == frsky_id[3] && s_rx_buf[2] == frsky_id[2]) {
    frsky_diag[F_RX_BIND]++;                 /* telemetry packets with our id */
    frsky_diag[F_RX_CH0_US]  = s_rx_buf[3];  /* A1 */
    frsky_diag[F_RX_CH1_US]  = s_rx_buf[4];  /* A2 */
    frsky_diag[F_RX_COUNTER] = s_rx_buf[5];  /* RSSI as seen by the receiver */
    s_last_rssi = det.rssi_dbm;              /* how strong the receiver is heard here */
    s_last_telem_us = (uint32_t)sl_rail_get_time(s_rail);
  } else {
    frsky_diag[F_RX_ODD]++;
  }
}

/* Every packet waiting, not just one: the event handler holds each received packet, and at the full packet rate two
 * can arrive between main-loop passes. Processing only one per pass left the other held for good; held packets
 * filled the receive buffer and the radio stopped receiving (bench 25 Oct: link gone after 30-60 s, reboot only). */
void frsky_rx_process(void)
{
  for (uint8_t k = 0; k < 8u; k++) {
    sl_rail_rx_packet_info_t info;
    if (sl_rail_get_rx_packet_info(s_rail, SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &info) == SL_RAIL_RX_PACKET_HANDLE_INVALID) return;
    frsky_rx_one();
  }
}
