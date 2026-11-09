/* frsky_d16.c — FrSky D16 (ACCST v1/v2, EU-LBT and FCC) transmitter and receiver (see .h).
 * Pulled into app_process.c with #include "frsky_d16.c". */
#include "frsky_d16.h"
#include "servo_pins.h"
#include "rail_config.h"
#include <string.h>
#include "rx_store.h"
#include "sl_core.h"

extern const RAIL_ChannelConfig_t frsky_d16eu_channelConfig;   /* autogen/rail_config.c */
/* FCC radio settings (radio_settings.radioconf, 25 Oct 2026). Weak: a rail_config.c generated before they existed
 * still links, and the FCC variants then fall back to the EU settings (log says so). */
extern const RAIL_ChannelConfig_t frsky_d16fcc_channelConfig  __attribute__((weak));   /* 70,022 bit/s GFSK, ACCST v1 */
extern const RAIL_ChannelConfig_t frsky_d16fcc2_channelConfig __attribute__((weak));   /* 76,965 bit/s GFSK, ACCST v2 */

#ifndef FRSKY16_PPM_CHANNELS
#define FRSKY16_PPM_CHANNELS     8u      /* channels in the PPM frame (max 8)        */
#endif
#define FRSKY16_PPM_FRAME_US     22500u
#define FRSKY16_PPM_PULSE_US     300u    /* low pulse; line idles high               */
static void frsky16_ppm_enable(bool on);   /* servo_out.c does the pulses (below) */
#define FRSKY16_FAILSAFE_MISSES  111u    /* ~1 s without packets -> PPM stops        */
#define FRSKY16_LOST_MISSES      333u    /* ~3 s -> back to searching on hop[0]      */
#ifndef FRSKY16_BIND_PIN
#define FRSKY16_BIND_PIN         0
#endif
#ifndef FRSKY16_BIND_SLOTS
#define FRSKY16_BIND_SLOTS       167u    /* 167 x 9 ms = 1.5 s (RX powered first) */
#endif
#ifndef FRSKY16_BIND_POWER_DDBM
#define FRSKY16_BIND_POWER_DDBM  0
#endif
#ifndef FRSKY16_DATA_POWER_DDBM
#define FRSKY16_DATA_POWER_DDBM  100
#endif
#ifndef FRSKY16_RANGE_POWER_DDBM
#define FRSKY16_RANGE_POWER_DDBM -100
#endif
#ifndef FRSKY16_LBT_POWER_DDBM
#define FRSKY16_LBT_POWER_DDBM   -100    /* power used when the channel is busy */
#endif
#define FRSKY16_LBT_BUSY_QDBM    (-67 * 4)
#ifndef FRSKY16_FS_DEFAULT
#define FRSKY16_FS_DEFAULT       0       /* RX failsafe with no stored preset: 0 = hold, 1 = no pulses */
#endif
#define FRSKY16_FS_PRESS_US      300000u /* BIND held this long while linked -> store failsafe preset */
#define FRSKY16_BIND_TIMEOUT_US  30000000u /* BIND at boot: give up & revert to stored bind after this (30 s: time to start a TX's bind) */
#define FRSKY16_COMMIT_PKTS      20u      /* good data packets before a fresh bind is written to flash      */
#ifndef FRSKY16_RX_BIND_BUTTON
#define FRSKY16_RX_BIND_BUTTON   1       /* 1 once PB00 is verified: BIND at boot = forget TX, BIND while linked = store preset */
#endif
#define FRSKY16_SEARCH_ALT_US    300000u /* RX search: alternate hop[0] / bind channel 0 this often */

static const uint8_t frsky16_ch_map[16] = { 0,1,2,3,4,5,6,7, 0,1,2,3,4,5,6,7 };  /* PPM has 8; 9-16 mirror */

#define SLOT_US        9000u
#define LBT_US         400u
#define RX_OPEN_US     4400u
#define RX_CLOSE_US    8600u
#define TX_LEN         33u
#define RX_LEN         15u
#define HOP_COUNT      47u
#define LINK_HOLD_US   500000u
/* Protocol variant from the encoder's Sub (or the stored setup), as Multiprotocol's FrSkyX / FrSkyX2:
 *   0 = ACCST D16 v1 EU-LBT  (100 kbit/s 2-FSK, 2405 MHz, 33-byte packets, hop table sent at bind, LBT)
 *   1 = ACCST D16 v2 EU-LBT  (same radio; 30-byte packets + CC2500 CRC, hop table from the ID, XOR-coded bind)
 *   2 = ACCST D16 v1 FCC     (70 kbit/s GFSK, 2404 MHz, 30-byte packets, no LBT)
 *   3 = ACCST D16 v2 FCC     (77 kbit/s GFSK, 2404 MHz, 30-byte packets + CC2500 CRC, no LBT)
 * The EFR32 receiver follows the same Sub (bench 26 Oct: binds to a Taranis X7 in D16 = v1 FCC, Sub 2). */
uint8_t frsky16_variant = 0;
/* Sub 5 = v1 FCC with 8 channels only (Multiprotocol's "CH_8": every packet carries channels 1-8). */
static bool    g_ch8;
static bool    g_v2, g_fcc;
static uint8_t g_tx_len = TX_LEN;      /* bytes incl. the length byte: 33 (v1) / 30 (v2) */
static uint8_t g_air_len = TX_LEN;     /* on air: 33 (v1) / 32 (v2: + CC2500 CRC)       */
#define D16_TUNE_HZ      (-20000)   /* transmitter tune, as D8 (bench 25 Oct: our crystal) */
#define D16_SPACING_HZ   299927u    /* CC2500 CHANSPC 0x7A at 26 MHz (RAIL's 300000 is 17 kHz off at ch 235) */
#define D16_LBT_AVG_US   300u       /* RSSI averaged over this long before each data packet (ETSI LBT) */
#define D16_RX_MISS_US   2000u      /* RX: a packet not ended this long after its expected end is lost */

/* id1 = 0x02 for D16; id2, id3 per module (chip EUI64) and per model (per-model binding, rc_model in
 * app_process.c). RX_NUM, FrSky's own receiver-number field, carries the model number too. */
static uint8_t frsky16_id[4] = { 0x00, 0x02, 0x00, 0x00 };
#define RX_NUM rc_model

static void frsky16_make_id(void)
{
  uint32_t e = DEVINFO->EUI64L ^ DEVINFO->EUI64H;
  frsky16_id[2] = (uint8_t)((uint8_t)e ^ (uint8_t)(e >> 16) ^ rc_model);
  frsky16_id[3] = (uint8_t)((uint8_t)(e >> 8) ^ (uint8_t)(e >> 24) ^ (uint8_t)(rc_model * 37u));
  if ((frsky16_id[2] | frsky16_id[3]) == 0u) frsky16_id[3] = 1u;
}

volatile int32_t frsky16_diag[16];

static sl_rail_handle_t g_rail;
static const volatile uint16_t *g_ch;
static uint8_t  g_hop[50];
static uint8_t  g_pkt[TX_LEN + 2u];
static uint8_t  g_rx_buf[40];
static uint8_t  g_last_raw[40];             /* the last packet heard, raw (FS D16 pkt log line) */
static uint16_t g_last_n;
static frsky16_phase_t g_phase;
static uint32_t g_slot_us, g_slot, g_last_telem_us;
static bool     g_bind_hold;         /* TX: BIND on in the serial frames (frsky16_bind) */
static uint8_t  g_step;              /* 0 listen, 1 sent, 2 rx open, 3 rx closed */
static uint8_t  g_hop_no, g_chanskip, g_tx_seq = 8, g_rx_seq = 0, g_chan_start;
static int32_t  g_power_ddbm = FRSKY16_DATA_POWER_DDBM;
static RAIL_ChannelConfigEntry_t g_cc_ent;     /* RAM copy of the channel table: CC2500 spacing (+ tune on a TX) */
static RAIL_ChannelConfig_t      g_cc;
/* ---- transmitter slot chain (radio events; see frsky16_on_events) ---- */
static volatile bool     g_chain_run;
static volatile uint32_t g_chain_seen;  /* RAIL time of the chain's last step (watchdog) */
static volatile uint8_t  g_cstate;      /* 1 LBT RSSI, 2 packet on air, 3 telemetry window */
static uint32_t g_slot_t;               /* start time of the current slot */
static uint16_t g_cur_ch;               /* channel of the current slot */
static uint32_t g_resync, g_refused, g_busy;

/* ---- receiver state (rx_role) ---- */
typedef enum { RXS_BIND = 0, RXS_SEARCH, RXS_TRACK } rxs_t;
static rxs_t    g_rxs;
static bool     g_rx_role;
static uint16_t g_bind_mask;              /* bits 0..9: hop-table chunks received  */
static uint8_t  g_rx_id2, g_rx_id3;
static uint8_t  g_bhop[50], g_bid3, g_bid2;   /* hop table + id being collected during bind */
static uint32_t g_bind_last_us;           /* last bind packet seen (bind-stall timeout) */
static uint32_t g_expect_us;              /* timestamp the next data packet should carry */
static bool     g_win_open;
static uint32_t g_misses;
static uint16_t g_ch_us[16];
static bool     g_reply_pending;
static volatile bool g_reply_txing;      /* RX role: telemetry reply on air (cleared by the TX-complete event) */
static volatile bool g_reply_done;       /* RX role: reply finished -> retune to the next hop from the main loop */
static uint32_t g_reply_at_us;
static volatile bool     g_isr_done;       /* RX: the event handler already replied / retuned for the newest packet */
static volatile uint16_t g_after_reply_ch; /* RX: where to listen once the reply is off the air */
static volatile uint32_t g_reply_t0;       /* RX: when the reply was started (stuck-reply watchdog) */
static uint8_t  g_rx_seq_out;
static int8_t   g_last_rssi;
static rx_record_t g_rec;               /* stored bind + failsafe preset (USERDATA flash) */
static uint32_t g_btn_since;
static rx_record_t g_saved;      /* bind loaded from flash at boot = fallback                 */
static bool     g_have_saved;    /* g_saved is valid                                          */
static bool     g_binding;       /* a boot-initiated bind attempt is in progress              */
static bool     g_provisional;   /* current hop/id came from a fresh bind, not yet committed   */
static uint16_t g_good;          /* good data packets since a provisional bind                 */
static uint32_t g_bind_deadline; /* revert-to-stored time for a boot bind attempt             */
static bool     g_btn_done;
static bool     g_in_failsafe;
static uint32_t g_last_rx;                 /* time of the last valid data packet (dual receiver) */

/* Multi's FrSkyX CRC (table for the low nibble, 0x1081 multiples for the high). */
static const uint16_t crc_short[16] = {
  0x0000,0x1189,0x2312,0x329B,0x4624,0x57AD,0x6536,0x74BF,
  0x8C48,0x9DC1,0xAF5A,0xBED3,0xCA6C,0xDBE5,0xE97E,0xF8F7 };
static uint16_t crc_step(uint8_t v) { return (uint16_t)(crc_short[v & 0x0F] ^ (0x1081u * (v >> 4))); }
static uint16_t frsky16_crc(const uint8_t *d, uint8_t len)
{
  uint16_t crc = 0;
  for (uint8_t i = 0; i < len; i++) crc = (uint16_t)((crc << 8) ^ crc_step((uint8_t)((crc >> 8) ^ d[i])));
  return crc;
}

/* Multiprotocol's Frsky_init_hop (FrSkyX v1): start 0..7, a per-ID step, channels 0x00 / 0x5A / 0xDC avoided,
 * entries 47..49 = 0 (bind channel). The receiver learns the table at bind. */
static void frsky16_init_hop_v2(void)          /* Multiprotocol FrSkyX2_init_hop, LBT exceptions */
{
  uint16_t id = (uint16_t)((frsky16_id[2] << 8) + frsky16_id[3]);
  uint8_t inc = (uint8_t)((id % 46u) + 1u);
  if (inc == 12u || inc == 35u) inc++;
  uint8_t offset = (uint8_t)(id % 5u);
  for (uint8_t i = 0; i < 47u; i++) {
    uint8_t ch = (uint8_t)(5u * ((uint16_t)(inc * i) % 47u) + offset);
    if (!g_fcc) {
      if (ch <= 1u || ch == 43u || ch == 44u || ch == 87u || ch == 88u || ch == 129u || ch == 130u || ch == 173u || ch == 174u) ch = (uint8_t)(ch + 2u);
      else if (ch == 216u || ch == 217u || ch == 218u) ch = (uint8_t)(ch + 3u);
    } else if (ch == 3u || ch == 4u || ch == 46u || ch == 47u || ch == 90u || ch == 91u || ch == 133u || ch == 134u ||
               ch == 176u || ch == 177u || ch == 220u || ch == 221u) ch = (uint8_t)(ch + 2u);
    g_hop[i] = ch;
  }
  g_hop[47] = 0; g_hop[48] = 0; g_hop[49] = 0;
}

static void frsky16_init_hop(void)
{
  uint32_t e = DEVINFO->EUI64L ^ DEVINFO->EUI64H;
  uint8_t channel = (uint8_t)(((uint8_t)(e >> 4) ^ rc_model) & 0x07u);
  uint8_t sp = (uint8_t)((uint8_t)(e >> 12) ^ (uint8_t)(rc_model * 13u));
  if (sp < 0x02u) sp = (uint8_t)(sp + 0x02u);
  if (sp > 0xE9u) sp = (uint8_t)(sp - 0xE7u);
  if ((sp % 0x2Fu) == 0u) sp++;
  g_hop[0] = channel;
  for (uint8_t i = 1; i < 50; i++) {
    channel = (uint8_t)((channel + sp) % 0xEBu);
    uint8_t v = channel;
    if (v == 0x00u || v == 0x5Au || v == 0xDCu) v++;
    g_hop[i] = (i > 46u) ? 0u : v;
  }
}

static void frsky16_seal(void)   /* FrSkyX CRC over bytes 3..len-3 -> the last two; v2: + the CC2500 CRC after them */
{
  uint16_t c = frsky16_crc(&g_pkt[3], (uint8_t)(g_tx_len - 5u));
  g_pkt[g_tx_len - 2u] = (uint8_t)(c >> 8);
  g_pkt[g_tx_len - 1u] = (uint8_t)c;
  if (g_v2) {                                    /* CC2500 hardware CRC (PKTCTRL0 0x05), as in frsky_d8.c */
    uint16_t h = frsky_crc(g_pkt, g_tx_len);
    g_pkt[g_tx_len] = (uint8_t)(h >> 8); g_pkt[g_tx_len + 1u] = (uint8_t)h;
  }
}

static void frsky16_build_bind(uint32_t n)
{
  if (g_v2) {                                    /* Multiprotocol FrSkyX_build_bind_packet, FRSKYX2 branch */
    memset(g_pkt, 0, sizeof g_pkt);
    g_pkt[0] = (uint8_t)(g_tx_len - 1u);         /* 0x1D */
    g_pkt[1] = 0x03; g_pkt[2] = 0x01;
    g_pkt[3] = frsky16_id[3]; g_pkt[4] = frsky16_id[2];
    if (n & 1u) memcpy(&g_pkt[7], "\x00\xCC\x00\x00\x00\x70\x14\x15\x00\xD3\x08\x00\x00\xCE\xE2\x85\xC7\x00\x00\x00\x00", 21);
    else        memcpy(&g_pkt[7], "\x27\xFB\x00\x00\x00\xBC\xEF\x19\x00\x26\x07\x00\x00\xB7\xED\x85\xC7\xA7\xA7\xA7\xA7", 21);
    g_pkt[5] = frsky16_id[1];
    g_pkt[6] = RX_NUM;
    g_pkt[20] ^= frsky16_id[3];
    g_pkt[21] ^= frsky16_id[2];
    g_pkt[22] ^= (uint8_t)(frsky16_id[1] & 0x3Fu);
    for (uint8_t i = 3; i < g_tx_len - 2u; i++) g_pkt[i] ^= 0xA7;
    frsky16_seal();
    return;
  }
  uint8_t idx = (uint8_t)((n % 10u) * 5u);
  memset(g_pkt, 0, sizeof g_pkt);
  g_pkt[0] = (uint8_t)(g_tx_len - 1u);      /* 0x20 v1 EU, 0x1D FCC */
  g_pkt[1] = 0x03; g_pkt[2] = 0x01;
  g_pkt[3] = frsky16_id[3]; g_pkt[4] = frsky16_id[2];
  g_pkt[5] = idx;
  for (uint8_t k = 0; k < 5; k++) g_pkt[6 + k] = g_hop[idx + k];
  g_pkt[11] = frsky16_id[1];
  g_pkt[12] = RX_NUM;
  frsky16_seal();
}

static inline uint16_t frsky16_val(uint16_t us)
{
  if (us < 860u) us = 860u;
  if (us > 2140u) us = 2140u;
  return (uint16_t)(((us - 860u) * 3u) / 2u + 64u);
}

static void frsky16_build_data(void)
{
  memset(g_pkt, 0, sizeof g_pkt);
  g_pkt[0] = (uint8_t)(g_tx_len - 1u);
  g_pkt[1] = frsky16_id[3]; g_pkt[2] = frsky16_id[2]; g_pkt[3] = frsky16_id[1];
  g_pkt[4] = (uint8_t)((g_chanskip << 6) | g_hop_no);
  g_pkt[5] = (uint8_t)(g_chanskip >> 2);
  g_pkt[6] = RX_NUM;
  g_pkt[7] = 0; g_pkt[8] = 0;                     /* flags, reserved */
  uint8_t ci = g_chan_start;
  for (uint8_t i = 9; i < 21; i += 3) {
    uint16_t a = frsky16_val(g_ch[frsky16_ch_map[ci]]); if (ci > 7) a |= 0x800; ci++;
    uint16_t b = frsky16_val(g_ch[frsky16_ch_map[ci]]); if (ci > 7) b |= 0x800; ci++;
    g_pkt[i]     = (uint8_t)a;
    g_pkt[i + 1] = (uint8_t)(((a >> 8) & 0x0F) | (b << 4));
    g_pkt[i + 2] = (uint8_t)(b >> 4);
  }
  if (!g_ch8) g_chan_start ^= 8;                  /* 16 channels: halves alternate; 8-channel mode: always 1-8 */
  g_pkt[21] = (uint8_t)((g_rx_seq << 4) | g_tx_seq);
  g_pkt[22] = 0;                                  /* no S.Port data */
  frsky16_seal();
}

static void frsky16_set_phase(frsky16_phase_t p)
{
  g_phase = p; frsky16_diag[G_PHASE] = p;
  g_slot = 0; g_step = 0; g_hop_no = 0; g_chan_start = 0; g_tx_seq = 8; g_rx_seq = 0;
  sl_rail_set_tx_power_dbm(g_rail, (p == FRSKY16_PHASE_BIND) ? FRSKY16_BIND_POWER_DDBM : g_power_ddbm);
}

/* ---- PPM + 8 servo pulses: servo_out.c (TIMER2) reads g_ch_us ---- */
static void frsky16_ppm_enable(bool on) { servo_out_enable(on); }
bool frsky16_rx_fresh(uint32_t now, uint32_t within_us) { return g_last_rx != 0u && (now - g_last_rx) <= within_us; }


void frsky16_start(sl_rail_handle_t rh, const volatile uint16_t *ch_us, bool bind_held, bool rx_role)
{
  g_rail = rh; g_ch = ch_us; g_rx_role = rx_role;
  for (int i = 0; i < 16; i++) frsky16_diag[i] = 0;
  frsky16_diag[G_ROLE] = rx_role ? 1 : 0;
  uint8_t sub = frsky16_variant;
  g_ch8 = (!rx_role && sub == 5u);               /* Sub 5: v1 FCC with 8 channels only */
  if (sub == 5u) sub = 2u;
  uint8_t var = (uint8_t)(sub & 3u);             /* the receiver follows the same Sub: EU or FCC, v1 or v2 */
  g_v2  = (var == 1u || var == 3u);
  g_fcc = (var >= 2u);
  const RAIL_ChannelConfig_t *base = &frsky_d16eu_channelConfig;
  bool fcc_missing = false;
  if (g_fcc) {
    const RAIL_ChannelConfig_t *f = g_v2 ? &frsky_d16fcc2_channelConfig : &frsky_d16fcc_channelConfig;
    if (f != NULL) base = f; else fcc_missing = true;
  }
  g_tx_len  = (var == 0u) ? TX_LEN : 30u;
  g_air_len = (uint8_t)(g_tx_len + (g_v2 ? 2u : 0u));
  frsky16_make_id();
  if (g_v2) frsky16_init_hop_v2(); else frsky16_init_hop();
  g_chanskip = (uint8_t)((DEVINFO->EUI64L % 46u) + 1u);

  sl_rail_idle(g_rail, SL_RAIL_IDLE_ABORT, true);
  g_cc_ent = base->configs[0];
  g_cc_ent.baseFrequency = (uint32_t)((int32_t)base->configs[0].baseFrequency + (rx_role ? 0 : D16_TUNE_HZ));
  g_cc_ent.channelSpacing = D16_SPACING_HZ;
  g_cc = *base; g_cc.configs = &g_cc_ent; g_cc.length = 1u;
  if (sl_rail_config_channels(g_rail, (const sl_rail_channel_config_t *)&g_cc, NULL)
      != SL_RAIL_STATUS_NO_ERROR) frsky16_diag[G_CFG_ERR] |= 1;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("FS D16 %s %s: channel 0 = %lu Hz, spacing %lu Hz%s\r\n", rx_role ? "receiver" : "transmitter",
          var == 0u ? "ACCST v1 EU-LBT (Sub 0)" : var == 1u ? "ACCST v2 EU-LBT (Sub 1)" :
          g_ch8 ? "ACCST v1 FCC, 8 channels (Sub 5)" : var == 2u ? "ACCST v1 FCC (Sub 2)" : "ACCST v2 FCC (Sub 3)",
          (unsigned long)g_cc_ent.baseFrequency, (unsigned long)D16_SPACING_HZ,
          (rx_role && g_v2) ? " (receiver binding is v1 only)" : "");
  if (fcc_missing) app_log("FS D16 FCC radio settings missing from rail_config.c (regenerate): using the EU ones\r\n");
#else
  (void)fcc_missing;
#endif
  if (sl_rail_set_fixed_length(g_rail, g_air_len) != g_air_len) frsky16_diag[G_CFG_ERR] |= 2;
  /* v1 (25 Oct 2026): the CC2500 of an X-series receiver runs SYNC_MODE 30/32, i.e. it sends and expects its sync
   * word 0xD3 0x91 twice (32 bits). The RAIL API sends its value least-significant bit first, so D3 91 D3 91 (each
   * byte MSB first) is the 32-bit bit reversal 0x89CB89CB. v0 left a 16-bit sync from the PHY file: two EFR32
   * boards heard each other, a FrSky receiver could not. */
  /* The doubled sync word D3 91 D3 91 (CC2500 SYNC_MODE 30/32) comes from the radio settings themselves
   * (syncword_0 0xD391D391, 32 bits). A runtime override sent it in a form FrSky receivers did not accept. */
  sl_rail_config_events(g_rail, SL_RAIL_EVENT_RX_PACKET_RECEIVED, SL_RAIL_EVENT_RX_PACKET_RECEIVED);
  if (!rx_role)
    sl_rail_config_events(g_rail, SL_RAIL_EVENT_RSSI_AVERAGE_DONE | SL_RAIL_EVENT_RX_SCHEDULED_RX_END | SL_RAIL_EVENTS_TX_COMPLETION,
                                  SL_RAIL_EVENT_RSSI_AVERAGE_DONE | SL_RAIL_EVENT_RX_SCHEDULED_RX_END | SL_RAIL_EVENTS_TX_COMPLETION);
  g_chain_run = false; g_cstate = 0;

  g_slot_us = (uint32_t)sl_rail_get_time(g_rail);
  g_last_telem_us = g_slot_us - LINK_HOLD_US;
  if (rx_role) {                       /* ---- D16 EU receiver with PPM out ---- */
    frsky16_set_phase(FRSKY16_PHASE_RX);
    g_rxs = RXS_BIND; g_bind_mask = 0; g_misses = 0; g_win_open = false; g_reply_pending = false; g_last_rx = 0;
    g_in_failsafe = false; g_btn_since = 0; g_btn_done = false;
    g_reply_txing = false; g_reply_done = false;
    sbus_init();
    g_provisional = false; g_good = 0; g_binding = false;
    for (uint8_t i = 0; i < 16; i++) g_ch_us[i] = 1500u;
    if (bind_held) frsky16_diag[G_CFG_ERR] |= 128;                   /* [6] bit 128: BIND read low at boot */
    g_have_saved = (rx_store_load(RX_SLOT_D16, &g_saved) && g_saved.bound);
    if (bind_held) {                        /* BIND low at boot: try to (re)bind, keep the stored one as fallback */
      g_rxs = RXS_BIND; g_bind_mask = 0; g_binding = true;
      g_bind_deadline = g_slot_us + FRSKY16_BIND_TIMEOUT_US;
      frsky16_diag[G_PHASE] = 0; frsky16_diag[G_BIND_SENT] = 0;
      memset(&g_rec, 0, sizeof(g_rec));
    } else if (g_have_saved) {              /* normal boot: use the stored bind */
      g_rec = g_saved;
      memcpy(g_hop, g_rec.hop, 50); g_rx_id2 = g_rec.id2; g_rx_id3 = g_rec.id3;
      g_rxs = RXS_SEARCH; frsky16_diag[G_PHASE] = 10; frsky16_diag[G_BIND_SENT] = 7;   /* 7 = loaded from flash */
    } else {                                /* nothing stored: must bind (no fallback) */
      g_rxs = RXS_BIND; g_bind_mask = 0; memset(&g_rec, 0, sizeof(g_rec));
    }
    servo_out_init(); servo_out_source(g_ch_us);                  /* pulses: servo_out.c */
    sl_rail_set_fixed_length(g_rail, g_air_len);
    if (sl_rail_start_rx(g_rail, (g_rxs == RXS_BIND) ? 0 : g_hop[0], NULL) != SL_RAIL_STATUS_NO_ERROR) frsky16_diag[G_CFG_ERR] |= 8;
    return;
  }
#if FRSKY16_BIND_PIN
  frsky16_set_phase(bind_held ? FRSKY16_PHASE_BIND : FRSKY16_PHASE_DATA);
#else
  (void)bind_held;
  frsky16_set_phase(FRSKY16_PHASE_BIND);
#endif
}

/* Send g_pkt on ch at RAIL time 'when' (or now if that is too close). */
static void frsky16_tx_at(uint16_t ch, int32_t ddbm, uint32_t when)
{
  sl_rail_idle(g_rail, SL_RAIL_IDLE, false);
  sl_rail_set_tx_power_dbm(g_rail, ddbm);
  sl_rail_set_fixed_length(g_rail, g_air_len);
  sl_rail_write_tx_fifo(g_rail, g_pkt, g_air_len, true);
  g_cstate = 2;
  if ((int32_t)(when - (uint32_t)sl_rail_get_time(g_rail)) > 50) {
    sl_rail_scheduled_tx_config_t sc = { .when = when, .mode = SL_RAIL_TIME_ABSOLUTE,
                                         .tx_during_rx = SL_RAIL_SCHEDULED_TX_DURING_RX_ABORT_TX };
    if (sl_rail_start_scheduled_tx(g_rail, ch, SL_RAIL_TX_OPTIONS_DEFAULT, &sc, NULL) == SL_RAIL_STATUS_NO_ERROR) return;
    g_refused++;
  }
  if (sl_rail_start_tx(g_rail, ch, SL_RAIL_TX_OPTIONS_DEFAULT, NULL) != SL_RAIL_STATUS_NO_ERROR) {
    frsky16_diag[G_TX_ERR]++; g_cstate = 0;          /* no event will come: the watchdog restarts the chain */
  }
}

/* ---- the transmitter's slot chain, driven by radio events (interrupt context) ------------------------------
 * Same order and timing as Multiprotocol's FRSKYX_callback (LBT): at the slot start the receiver averages RSSI on
 * the slot's channel for 300 us (ETSI listen-before-talk; busy -> low power, as Multi), the packet goes out 400 us
 * into the slot, then a telemetry window from 4.4 to 8.6 ms on the same channel, then the next slot. Bind: one
 * bind packet per 9 ms slot on channel 0. Nothing in the main loop (log printing, serial frames) can move a packet. */
static void frsky16_slot_begin(uint32_t S)
{
  g_slot_t = S; g_chain_seen = (uint32_t)sl_rail_get_time(g_rail);
  if (g_phase == FRSKY16_PHASE_BIND) {
    frsky16_build_bind(g_slot); frsky16_diag[G_BIND_SENT]++;
    g_cur_ch = 0u;
    frsky16_tx_at(0u, FRSKY16_BIND_POWER_DDBM, S + (g_fcc ? 0u : LBT_US));
    return;
  }
  frsky16_build_data(); frsky16_diag[G_DATA_SENT]++;
  g_cur_ch = g_hop[g_hop_no];
  g_hop_no = (uint8_t)((g_hop_no + g_chanskip) % HOP_COUNT);     /* the packet carries the number it is sent on */
  if (g_fcc) { frsky16_tx_at(g_cur_ch, g_power_ddbm, S); return; }   /* FCC: no listen-before-talk */
  sl_rail_idle(g_rail, SL_RAIL_IDLE, false);
  g_cstate = 1;
  if (sl_rail_start_average_rssi(g_rail, g_cur_ch, D16_LBT_AVG_US, NULL) != SL_RAIL_STATUS_NO_ERROR)
    frsky16_tx_at(g_cur_ch, g_power_ddbm, S + LBT_US);          /* no RSSI: send at the normal power */
}

static void frsky16_slot_next(void)
{
  g_slot++;
  if (g_phase == FRSKY16_PHASE_BIND) {
    if (g_slot >= FRSKY16_BIND_SLOTS) frsky16_set_phase(FRSKY16_PHASE_DATA);
  } else if (g_bind_hold && g_slot >= FRSKY16_BIND_SLOTS) {
    frsky16_set_phase(FRSKY16_PHASE_BIND);                       /* BIND still on: the next bind turn */
  }
  frsky16_slot_begin(g_slot_t + SLOT_US);
}



/* ---- receiver: window scheduling, misses, telemetry reply ---- */
static void frsky16_rx_arm(uint16_t ch)              /* ISR-safe: listen on ch for a data packet */
{
  sl_rail_idle(g_rail, SL_RAIL_IDLE, false);
  sl_rail_set_fixed_length(g_rail, g_air_len);
  (void)sl_rail_start_rx(g_rail, ch, NULL);
}
/* Main-loop retune, with interrupts held off so the event handler cannot run in the middle; a reply still
 * pending is dropped. */
static void frsky16_rx_listen(uint16_t ch)
{
  CORE_DECLARE_IRQ_STATE; CORE_ENTER_CRITICAL();
  if (g_reply_txing) { g_reply_txing = false; sl_rail_idle(g_rail, SL_RAIL_IDLE_ABORT, false); }
  frsky16_rx_arm(ch);
  CORE_EXIT_CRITICAL();
}

static void frsky16_reply_build(uint8_t *r)
{
  memset(r, 0, RX_LEN);
  r[0] = RX_LEN - 1u; r[1] = g_rx_id3; r[2] = g_rx_id2;
  r[3] = 0; r[4] = 0;                                            /* A1, A2: no ADC yet */
  r[5] = (uint8_t)((g_last_rssi + 72) * 2);
  r[6] = g_rx_seq_out;
  uint16_t c = frsky16_crc(&r[3], RX_LEN - 5u);
  r[13] = (uint8_t)(c >> 8); r[14] = (uint8_t)c;
}

/* Send a reply r on ch at RAIL time 'when'. */
static bool frsky16_reply_at(const uint8_t *r, uint16_t ch, uint32_t when)
{
  sl_rail_idle(g_rail, SL_RAIL_IDLE, false);
  sl_rail_set_tx_power_dbm(g_rail, FRSKY16_DATA_POWER_DDBM);
  sl_rail_set_fixed_length(g_rail, RX_LEN);
  sl_rail_write_tx_fifo(g_rail, r, RX_LEN, true);
  if ((int32_t)(when - (uint32_t)sl_rail_get_time(g_rail)) > 50) {
    sl_rail_scheduled_tx_config_t sc = { .when = when, .mode = SL_RAIL_TIME_ABSOLUTE,
                                         .tx_during_rx = SL_RAIL_SCHEDULED_TX_DURING_RX_ABORT_TX };
    if (sl_rail_start_scheduled_tx(g_rail, ch, SL_RAIL_TX_OPTIONS_DEFAULT, &sc, NULL) == SL_RAIL_STATUS_NO_ERROR) return true;
  }
  return sl_rail_start_tx(g_rail, ch, SL_RAIL_TX_OPTIONS_DEFAULT, NULL) == SL_RAIL_STATUS_NO_ERROR;
}

/* Link lost for FRSKY16_FAILSAFE_MISSES slots: apply the failsafe. */
static void frsky16_rx_failsafe(void)
{
  g_in_failsafe = true;
  if (g_rec.fs_valid) { for (uint8_t i = 0; i < 8; i++) g_ch_us[i] = g_rec.fs_us[i]; }   /* preset */
  else if (FRSKY16_FS_DEFAULT == 1) { frsky16_ppm_enable(false); }                    /* no pulses */
  /* else hold: outputs keep the last values */
}

/* BIND held >300 ms while the link is up: store the current positions as the failsafe preset. */
static void frsky16_rx_button(bool bind_held, uint32_t now)
{
  if (!FRSKY16_RX_BIND_BUTTON || !bind_held || g_binding) { g_btn_since = 0; g_btn_done = false; return; }
  if (g_btn_since == 0) { g_btn_since = now ? now : 1u; return; }
  if (g_btn_done || (now - g_btn_since) < FRSKY16_FS_PRESS_US) return;
  g_btn_done = true;
  if (g_rxs != RXS_TRACK || g_in_failsafe) return;
  for (uint8_t i = 0; i < 8; i++) g_rec.fs_us[i] = g_ch_us[i];
  g_rec.fs_valid = 1;
  if (rx_store_save(RX_SLOT_D16, &g_rec)) frsky16_diag[G_BIND_SENT] += 100;         /* [2] += 100 per stored preset */
}

static void frsky16_rx_service(bool bind_held)
{
  sbus_service();
  uint32_t now = (uint32_t)sl_rail_get_time(g_rail);
  if (g_binding && (int32_t)(now - g_bind_deadline) >= 0) {   /* bind attempt gave up: revert to the stored bind */
    g_binding = false; g_provisional = false;
    if (g_have_saved) {
      g_rec = g_saved; memcpy(g_hop, g_rec.hop, 50); g_rx_id2 = g_rec.id2; g_rx_id3 = g_rec.id3;
      g_rxs = RXS_SEARCH; g_win_open = false; frsky16_diag[G_PHASE] = 10; frsky16_diag[G_BIND_SENT] = 7;
    }                                              /* no fallback: stay in bind, keep listening */
  }
  frsky16_rx_button(bind_held, now);

  if (g_reply_pending && (int32_t)(now - g_reply_at_us) >= 0) {
    g_reply_pending = false;
    uint8_t r[RX_LEN];
    frsky16_reply_build(r);
    uint16_t ch = g_hop[g_hop_no];
    g_hop_no = (uint8_t)((g_hop_no + g_chanskip) % HOP_COUNT);   /* after the reply: the next hop */
    g_after_reply_ch = g_hop[g_hop_no];
    CORE_DECLARE_IRQ_STATE; CORE_ENTER_CRITICAL();
    if (!frsky16_reply_at(r, ch, 0u)) { frsky16_diag[G_TX_ERR]++; frsky16_rx_arm(g_after_reply_ch); }
    else { frsky16_diag[G_DATA_SENT]++; g_reply_txing = true; g_reply_t0 = now; }
    CORE_EXIT_CRITICAL();
    g_win_open = false;
    return;
  }
  if (g_reply_txing && (now - g_reply_t0) > 6000u) {   /* a reply never reported back (cancelled): listen again */
    frsky16_diag[G_TX_ERR]++;
    if (g_rxs == RXS_TRACK) { frsky16_rx_listen(g_after_reply_ch); g_win_open = true; }
    else g_reply_txing = false;
  }
  if (g_reply_done) {                     /* reply is off the air: sit on the next hop right away */
    g_reply_done = false;
    if (g_rxs == RXS_TRACK && !g_win_open) { frsky16_rx_listen(g_hop[g_hop_no]); g_win_open = true; }
  }

  if (g_rxs == RXS_BIND && g_rec.bound && (now - g_bind_last_us) > 2000000u) {   /* rebind stalled: keep the stored TX */
    g_rxs = RXS_SEARCH; frsky16_diag[G_PHASE] = 10; g_bind_last_us = now; g_win_open = false;
  }
  if (g_rxs == RXS_SEARCH) {                                       /* bound, TX not yet reacquired: keep RX on hop[0] */
    static uint32_t g_arm_us;
    if (!g_win_open || (now - g_arm_us) > 100000u) {
      sl_rail_idle(g_rail, SL_RAIL_IDLE_ABORT, true);            /* whatever state the radio was left in */
      frsky16_rx_listen(g_hop[0]); g_win_open = true; g_arm_us = now;
    }
    return;
  }
  if (g_rxs != RXS_TRACK) return;
  if (!g_win_open && (int32_t)(now - (g_expect_us - 2000u)) >= 0) {   /* fallback only: normally already listening */
    frsky16_rx_listen(g_hop[g_hop_no]);
    g_win_open = true;
  }
  if ((int32_t)(now - (g_expect_us + D16_RX_MISS_US)) >= 0) {     /* missed this slot */
    g_misses++; frsky16_diag[G_LBT_BUSY] = (int32_t)g_misses;
    g_expect_us += SLOT_US;
    g_hop_no = (uint8_t)((g_hop_no + g_chanskip) % HOP_COUNT);
    g_win_open = false;
    if (g_misses == FRSKY16_FAILSAFE_MISSES) frsky16_rx_failsafe();
    (void)sbus_send(g_ch_us, true, g_in_failsafe);
    if (g_misses >= FRSKY16_LOST_MISSES) { g_rxs = RXS_SEARCH; g_win_open = false; frsky16_rx_listen(g_hop[0]); }
    else { frsky16_rx_listen(g_hop[g_hop_no]); g_win_open = true; }   /* next hop right away */
  }
}

/* BIND from the serial frames on the transmitter: as frsky_bind() in frsky_d8.c -- 1.5 s of bind and 1.5 s of flying
 * data take turns while it is selected; leaving BIND ends the turns after the current one. */
void frsky16_bind(bool held) { g_bind_hold = held; }

void frsky16_service(bool bind_held, bool range_low)
{
  if (g_phase == FRSKY16_PHASE_RX) { frsky16_rx_service(bind_held); return; }
  uint32_t now = (uint32_t)sl_rail_get_time(g_rail);
  int32_t want = range_low ? FRSKY16_RANGE_POWER_DDBM : FRSKY16_DATA_POWER_DDBM;
  if (want != g_power_ddbm) g_power_ddbm = want;                 /* used from the next packet on */
  (void)bind_held;
  if (!g_chain_run || (int32_t)(now - g_chain_seen) > (int32_t)(4u * SLOT_US)) {   /* start, or restart a stalled chain */
    if (g_chain_run) g_resync++;
    g_chain_run = true; g_chain_seen = now;
    CORE_DECLARE_IRQ_STATE; CORE_ENTER_CRITICAL();
    frsky16_slot_begin(now + 2000u);
    CORE_EXIT_CRITICAL();
  }
}

void frsky16_tx_stats(uint32_t *busy, uint32_t *resync, uint32_t *refused) { *busy = g_busy; *resync = g_resync; *refused = g_refused; }

frsky16_phase_t frsky16_phase(void) { return g_phase; }
const uint8_t *frsky16_last_pkt(uint16_t *n) { *n = g_last_n; return g_last_raw; }
int8_t frsky16_tele_rssi_dbm(void) { return g_last_rssi; }

bool frsky16_link_up(void)
{
  if (g_rx_role) return (g_rxs == RXS_TRACK) && (g_misses < FRSKY16_FAILSAFE_MISSES);
  return ((uint32_t)sl_rail_get_time(g_rail) - g_last_telem_us) < LINK_HOLD_US;
}

void frsky16_on_events(sl_rail_events_t events)
{
  if (events & SL_RAIL_EVENT_TX_PACKET_SENT) frsky16_diag[G_TX_OK]++;
  else if (events & SL_RAIL_EVENTS_TX_COMPLETION) frsky16_diag[G_TX_ERR]++;
  if (g_rx_role) {
    if (g_reply_txing && (events & SL_RAIL_EVENTS_TX_COMPLETION)) {
      g_reply_txing = false; g_reply_done = true;
      if (g_rxs == RXS_TRACK) { frsky16_rx_arm(g_after_reply_ch); g_win_open = true; }   /* next hop at once */
    }
    return;
  }
  if (!g_chain_run || g_phase == FRSKY16_PHASE_RX) return;
  if (g_cstate == 1 && (events & SL_RAIL_EVENT_RSSI_AVERAGE_DONE)) {         /* LBT result: send 400 us into the slot */
    int16_t r = sl_rail_get_average_rssi(g_rail);
    int32_t p = g_power_ddbm;
    if (r != SL_RAIL_RSSI_INVALID && r > FRSKY16_LBT_BUSY_QDBM) { p = FRSKY16_LBT_POWER_DDBM; g_busy++; frsky16_diag[G_LBT_BUSY]++; }
    frsky16_tx_at(g_cur_ch, p, g_slot_t + LBT_US);
  } else if (g_cstate == 2 && (events & SL_RAIL_EVENTS_TX_COMPLETION)) {      /* packet gone */
    g_chain_seen = (uint32_t)sl_rail_get_time(g_rail);
    if (g_phase == FRSKY16_PHASE_BIND) { frsky16_slot_next(); return; }
    sl_rail_idle(g_rail, SL_RAIL_IDLE, false);
    sl_rail_set_fixed_length(g_rail, (uint16_t)(g_v2 ? RX_LEN + 2u : RX_LEN));
    sl_rail_scheduled_rx_config_t rc = { .start = g_slot_t + (g_fcc ? 5200u : RX_OPEN_US), .start_mode = SL_RAIL_TIME_ABSOLUTE,
                                         .end = g_slot_t + (g_fcc ? 8200u : RX_CLOSE_US), .end_mode = SL_RAIL_TIME_ABSOLUTE,
                                         .rx_transition_end_schedule = 0, .hard_window_end = 1 };
    g_cstate = 3;
    if (sl_rail_start_scheduled_rx(g_rail, g_cur_ch, &rc, NULL) != SL_RAIL_STATUS_NO_ERROR) frsky16_slot_next();
  } else if (g_cstate == 3 && (events & SL_RAIL_EVENT_RX_SCHEDULED_RX_END)) { /* telemetry window over */
    frsky16_slot_next();
  }
}

static void frsky16_rx_one(void)
{
  sl_rail_rx_packet_info_t info;
  sl_rail_rx_packet_handle_t h = sl_rail_get_rx_packet_info(g_rail, SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &info);
  if (h == SL_RAIL_RX_PACKET_HANDLE_INVALID) return;
  uint16_t n = info.packet_bytes; if (n > sizeof(g_rx_buf)) n = sizeof(g_rx_buf);
  sl_rail_rx_packet_details_t det; memset(&det, 0, sizeof(det));
  (void)sl_rail_get_rx_packet_details(g_rail, h, &det);
  (void)sl_rail_get_rx_time_frame_end(g_rail, &det);              /* timestamp = end of packet */
  sl_rail_copy_rx_packet(g_rail, g_rx_buf, &info);
  sl_rail_release_rx_packet(g_rail, h);
  frsky16_diag[G_RX_RSSI] = sl_rail_get_rssi(g_rail, 0);
  g_last_n = n; memcpy(g_last_raw, g_rx_buf, n);                    /* every packet heard, valid or not (log) */
  if (g_phase == FRSKY16_PHASE_RX) {
    bool ok = (n >= g_tx_len) && (g_rx_buf[0] == g_tx_len - 1u)
              && frsky16_crc(&g_rx_buf[3], (uint8_t)(g_tx_len - 5u)) == (uint16_t)((g_rx_buf[g_tx_len - 2u] << 8) | g_rx_buf[g_tx_len - 1u]);
    if (!ok) { frsky16_diag[G_RX_ODD]++; if (g_rxs != RXS_TRACK) frsky16_rx_listen(g_rxs == RXS_BIND ? 0 : g_hop[0]); return; }
    frsky16_diag[G_RX_PACKETS]++;
    g_last_rssi = det.rssi_dbm;

    if (g_rx_buf[1] == 0x03 && g_rx_buf[2] == 0x01) {              /* bind packet */
      frsky16_diag[G_RX_BIND]++;
      if (g_rxs == RXS_BIND) {
        uint8_t idx = g_rx_buf[5];
        g_bind_last_us = (uint32_t)sl_rail_get_time(g_rail);
        if (g_bind_mask == 0 || g_rx_buf[3] != g_bid3 || g_rx_buf[4] != g_bid2) {   /* new table: remember whose */
          g_bind_mask = 0; g_bid3 = g_rx_buf[3]; g_bid2 = g_rx_buf[4];
        }
        if (idx <= 45u && (idx % 5u) == 0u) {
          for (uint8_t k = 0; k < 5; k++) g_bhop[idx + k] = g_rx_buf[6 + k];
          g_bind_mask |= (uint16_t)(1u << (idx / 5u));
          if (g_bind_mask == 0x3FFu) {                             /* whole table: adopt provisionally */
            memcpy(g_hop, g_bhop, 50);
            g_rx_id3 = g_bid3; g_rx_id2 = g_bid2;
            memset(&g_rec, 0, sizeof(g_rec));
            memcpy(g_rec.hop, g_hop, 50); g_rec.id2 = g_rx_id2; g_rec.id3 = g_rx_id3; g_rec.bound = 1;
            g_provisional = true; g_good = 0;                    /* prove it with good packets before storing */
            g_bind_deadline = (uint32_t)sl_rail_get_time(g_rail) + FRSKY16_BIND_TIMEOUT_US;
            g_rxs = RXS_SEARCH; g_win_open = false; frsky16_diag[G_PHASE] = 10;
            frsky16_rx_listen(g_hop[0]); return;
          }
        }
        frsky16_rx_listen(0);
      }
      return;
    }
    if (g_rxs == RXS_BIND || g_rx_buf[1] != g_rx_id3 || g_rx_buf[2] != g_rx_id2) {
      frsky16_diag[G_RX_ODD]++;
      if (g_rxs != RXS_TRACK) frsky16_rx_listen(g_rxs == RXS_BIND ? 0 : g_hop[0]);
      return;
    }
    /* data packet from our transmitter */
    frsky16_diag[G_RX_DATA]++;
    g_hop_no   = g_rx_buf[4] & 0x3F;
    g_chanskip = (uint8_t)(((g_rx_buf[5] << 2) | (g_rx_buf[4] >> 6)) % HOP_COUNT);
    frsky16_diag[G_RX_HOPNO] = g_hop_no;
    uint32_t t = (uint32_t)det.time_received.packet_time;
    g_expect_us = t + SLOT_US;
    g_misses = 0; frsky16_diag[G_LBT_BUSY] = 0;
    g_rxs = RXS_TRACK; frsky16_diag[G_PHASE] = 11; g_in_failsafe = false; g_win_open = false;
    if (g_provisional) {                              /* fresh bind proving itself: commit after enough good packets */
      if (++g_good >= FRSKY16_COMMIT_PKTS) {
        g_provisional = false; g_binding = false;
        if (rx_store_save(RX_SLOT_D16, &g_rec)) { g_saved = g_rec; g_have_saved = true; frsky16_diag[G_BIND_SENT] = 1; }
      }
    } else g_binding = false;                         /* reconnected to the stored TX: no longer a bind attempt */
    for (uint8_t k = 0; k < 4; k++) {
      const uint8_t *p = &g_rx_buf[9 + 3 * k];
      uint16_t a = (uint16_t)(p[0] | ((p[1] & 0x0F) << 8));
      uint16_t b = (uint16_t)((p[1] >> 4) | (p[2] << 4));
      uint8_t base = (a & 0x800) ? 8u : 0u;
      g_ch_us[base + 2 * k]     = (uint16_t)(((a & 0x7FF) - 64) * 2 / 3 + 860);
      g_ch_us[base + 2 * k + 1] = (uint16_t)(((b & 0x7FF) - 64) * 2 / 3 + 860);
    }
    frsky16_diag[G_RX_CH0_US] = g_ch_us[0]; frsky16_diag[G_RX_CH1_US] = g_ch_us[1];
    g_last_rx = (uint32_t)sl_rail_get_time(g_rail);
    frsky16_ppm_enable(true);
    (void)sbus_send(g_ch_us, false, g_in_failsafe);
    if (g_isr_done) {                                 /* the event handler already replied and retuned */
      g_isr_done = false;
      g_hop_no = (uint8_t)((g_hop_no + g_chanskip) % HOP_COUNT);
      g_win_open = true;
      return;
    }
    g_rx_seq_out = (uint8_t)((g_rx_seq_out + 1u) & 3u);
    g_reply_pending = true; g_reply_at_us = t + 1500u;              /* inside the TX's 4.4-8.6 ms window */
    return;
  }
  /* transmitter: telemetry reply  0E id3 id2 echo X seq sport... crc. An X-series receiver puts, in turn, the RSSI it
     sees (bit 7 set, the usual 0..100-odd figure) and its battery (bit 7 clear, about 0.1 V a step: on the A1 scale
     of 13.2 V / 255 that is (v << 1) + 1) in byte 4; byte 3 echoes our packet's byte 3 (Betaflight cc2500_frsky_x.c:
     buildTelemetryFrame). Diag: [11] = battery on the A1 scale, [12] = the echo, [13] = the receiver's RSSI figure. */
  bool ok = (n >= RX_LEN) && (g_rx_buf[0] == RX_LEN - 1u) && g_rx_buf[1] == frsky16_id[3] && g_rx_buf[2] == frsky16_id[2]
            && frsky16_crc(&g_rx_buf[3], RX_LEN - 5u) == (uint16_t)((g_rx_buf[13] << 8) | g_rx_buf[14]);
  if (!ok) { frsky16_diag[G_RX_ODD]++; return; }
  frsky16_diag[G_RX_PACKETS]++; frsky16_diag[G_RX_BIND]++;
  if (g_rx_buf[4] & 0x80) frsky16_diag[G_RX_HOPNO]  = g_rx_buf[4] & 0x7F;
  else                    frsky16_diag[G_RX_CH0_US] = (int32_t)(((uint16_t)g_rx_buf[4] << 1) + 1u);
  frsky16_diag[G_RX_CH1_US] = g_rx_buf[3];
  g_last_rssi = det.rssi_dbm;                                       /* how strong the receiver is heard here */
  uint8_t in_seq = g_rx_buf[6];
  g_rx_seq = (uint8_t)(((in_seq & 0x0F) + 1u) & 3u);
  if (!(in_seq & 0xF8)) g_tx_seq = (uint8_t)((g_tx_seq + 1u) & 3u);
  g_last_telem_us = (uint32_t)sl_rail_get_time(g_rail);
}

/* Every packet waiting, not just one (D8 bench 25 Oct: a forgotten held packet per pass filled the buffer and the
 * radio went deaf until a reboot). */
void frsky16_rx_process(void)
{
  for (uint8_t k = 0; k < 8u; k++) {
    sl_rail_rx_packet_info_t info;
    if (sl_rail_get_rx_packet_info(g_rail, SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &info) == SL_RAIL_RX_PACKET_HANDLE_INVALID) return;
    frsky16_rx_one();
  }
}

/* RX role, from the RAIL event handler right after a packet is held: a good data packet from our transmitter gets
 * its telemetry reply scheduled 1.5 ms on (inside the TX's 4.4..8.6 ms window), then the radio goes to the next
 * hop. The main loop still decodes the packet. */
void frsky16_on_rx_held(void)
{
  if (!g_rx_role || g_rxs != RXS_TRACK || g_reply_txing) return;
  sl_rail_rx_packet_info_t info;
  sl_rail_rx_packet_handle_t h = sl_rail_get_rx_packet_info(g_rail, SL_RAIL_RX_PACKET_HANDLE_NEWEST, &info);
  if (h == SL_RAIL_RX_PACKET_HANDLE_INVALID) return;
  uint8_t b[TX_LEN + 2u];
  if (sl_rail_peek_rx_packet(g_rail, h, b, g_tx_len, 0) < g_tx_len) return;
  if (b[0] != g_tx_len - 1u || b[1] != g_rx_id3 || b[2] != g_rx_id2) return;
  if (frsky16_crc(&b[3], (uint8_t)(g_tx_len - 5u)) != (uint16_t)((b[g_tx_len - 2u] << 8) | b[g_tx_len - 1u])) return;
  uint8_t hop_no = (uint8_t)(b[4] & 0x3Fu);
  uint8_t skip   = (uint8_t)(((b[5] << 2) | (b[4] >> 6)) % HOP_COUNT);
  g_after_reply_ch = g_hop[(hop_no + skip) % HOP_COUNT];
  uint8_t r[RX_LEN];
  g_rx_seq_out = (uint8_t)((g_rx_seq_out + 1u) & 3u);
  frsky16_reply_build(r);
  if (frsky16_reply_at(r, g_hop[hop_no], (uint32_t)sl_rail_get_time(g_rail) + 1500u)) {
    g_reply_txing = true; g_reply_t0 = (uint32_t)sl_rail_get_time(g_rail); frsky16_diag[G_DATA_SENT]++;
  } else { frsky16_diag[G_TX_ERR]++; frsky16_rx_arm(g_after_reply_ch); }
  g_isr_done = true;
}
