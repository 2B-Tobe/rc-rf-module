/* sfhss.c — see sfhss.h.  #included into app_process.c.   v0.4 */
#include "sfhss.h"
#include <string.h>
#include "sl_core.h"
#include "rx_store.h"
#include "servo_out.h"
#include "sbus_out.h"

/* the PHY "sfhss" in config/rail/radio_settings.radioconf. Weak: the build links before Simplicity Studio has
 * generated it into autogen/rail_config.c; sfhss_start then refuses to run and says so. */
extern const RAIL_ChannelConfig_t sfhss_channelConfig __attribute__((weak));

#define SFHSS_PKT_LEN         13u
#define SFHSS_AIR_LEN         (SFHSS_PKT_LEN + 2u)     /* + the CRC, built here */
#define SFHSS_CHANNELS        30u
#define SFHSS_FRAME_US      6800u
#define SFHSS_PAIR_US       1625u      /* packet B this long after packet A (Multi: 1600..1650 works) */
#define SFHSS_SYNC_ON_AIR 0x89CB89CBul /* D3 91 D3 91, each byte MSB first = the bit reversal (RAIL sends LSB first) */
#define SFHSS_BASE_HZ   2403998656ul   /* CC2500: 2399999633 Hz + channel 16 x 249938.965 Hz */
#define SFHSS_SPACING_HZ   1499634ul   /* 6 CC2500 channels */
#define SFHSS_MIN_SLACK_US   100u      /* a launch nearer than this goes out at once */
#define SFHSS_START_SLACK_US 400u      /* a frame that cannot start this far ahead is skipped (the grid and the hop go on) */
#define SFHSS_LOG_US     5000000u      /* the status line: a line stalls the main loop for about 10 ms */
#define SFHSS_TX_POWER_DDBM  200       /* flight: module maximum (deci-dBm; RAIL caps it, about +19.5 dBm) */
#define SFHSS_FS_THROTTLE_US 1000u     /* failsafe sent to the receiver: throttle (channel 3) here, every other channel holds */
#define SFHSS_RANGE_POWER_DDBM (-100)
#define SFHSS_TX_FIFO_BYTES   64u

#ifndef LOG_TXT
#define LOG_TXT(s) s
#endif

volatile int32_t sfhss_diag[SFHSS_DIAG_N];
static sl_rail_handle_t f_rail;
static const volatile uint16_t *f_in;
static SL_RAIL_DECLARE_FIFO_BUFFER(f_txfifo, SFHSS_TX_FIFO_BYTES);
static RAIL_ChannelConfigEntry_t f_ent;
static RAIL_ChannelConfig_t      f_cfg;
static uint16_t f_id;
static uint8_t  f_code, f_ch;
static uint16_t f_counter;                 /* Multi's counter: its bit 0 swaps the channel halves frame by frame */
static uint8_t  f_a[SFHSS_AIR_LEN], f_b[SFHSS_AIR_LEN];
static volatile uint8_t f_stage;           /* 0 idle, 1 packet A on its way, 2 packet B on its way */
static volatile uint32_t f_t0;             /* launch time of packet A */
static volatile uint32_t f_next;          /* start of the next frame on the 6.8 ms grid */
static volatile uint32_t f_tx_count, f_skipped;
static uint32_t f_uart_us, f_tx_last;
static bool     f_run, f_input, f_range, f_started;
static volatile bool f_pwr_dirty, f_pwr_log;
static bool     f_rx;                      /* receiver role */
static sl_gpio_t f_led_a = { .port = SL_GPIO_PORT_C, .pin = 4 };
static sl_gpio_t f_led_b = { .port = SL_GPIO_PORT_C, .pin = 5 };

static inline uint32_t f_now(void) { return (uint32_t)sl_rail_get_time(f_rail); }

/* ---- the pieces a host test can run ---- */
uint16_t sfhss_crc(const uint8_t *d, uint8_t n)              /* the CC2500's CRC in its CC2400 mode (PKTCTRL0 CC2400_EN) */
{
  /* CRC-16 0x8005 from 0 with the data bit entering at x0: measured on a CC2500 by Raphael Coeffic (mpm-v2,
     docs/interop-results.md: "from 0 over all bytes but the last 2, which are XORed into it unprocessed" - the same
     thing), and a Futaba R2008SB links with it. */
  uint16_t c = 0x0000u;
  while (n--) {
    uint8_t b = *d++;
    for (uint8_t k = 0; k < 8u; k++, b = (uint8_t)(b << 1)) {
      bool fb = (c & 0x8000u) != 0u;
      c = (uint16_t)((c << 1) | (uint16_t)(b >> 7));
      if (fb) c ^= 0x8005u;
    }
  }
  return c;
}

uint8_t sfhss_next_chan(uint8_t ch, uint8_t code)
{
  ch = (uint8_t)(ch + code + 2u);
  if (ch > 29u) {
    if (ch < 31u) ch = (uint8_t)(ch + code + 2u);
    ch = (uint8_t)(ch - 31u);
  }
  return ch;
}

uint16_t sfhss_value(uint16_t us)                            /* 988 us -> 2020, 1500 -> 1520, 2012 -> 1020 */
{
  int32_t v = 1520 - (((int32_t)us - 1500) * 1000) / 1024;
  if (v < 0) v = 0;
  if (v > 4095) v = 4095;
  return (uint16_t)v;
}

uint16_t sfhss_us(uint16_t value)                            /* the reverse of sfhss_value */
{
  int32_t us = 1500 + ((1520 - (int32_t)value) * 1024) / 1000;
  if (us < 800) us = 800;
  if (us > 2200) us = 2200;
  return (uint16_t)us;
}

/* Failsafe words (cmd bit 2), as a Futaba T14SG sends them (mpm-v2 / Multi's sfhss-fs-scale): 0..1023 no pulses,
 * 1024..2047 hold, 2048..4095 a position: 0x800 | a signed 10-bit offset from the centre value in 1.25 steps (bit 10
 * ignored): 3072 = value 1520, 2672 = 1020, 3472 = 2020. */
uint16_t sfhss_fs_word(uint16_t value)                       /* the F/S word for a position given as a live value */
{
  int32_t off = (((int32_t)value - 1520) * 4) / 5;
  if (off < -511) off = -511;
  if (off > 511) off = 511;
  return (uint16_t)(0x800u | ((uint32_t)off & 0x3FFu));
}

uint16_t sfhss_fs_value(uint16_t word)                       /* the live value an F/S position word stands for */
{
  int32_t off = (int32_t)(word & 0x3FFu);
  if (off & 0x200) off -= 0x400;
  return (uint16_t)(1520 + (off * 5) / 4);
}

void sfhss_build(uint8_t *p, uint16_t id, uint8_t rf_ch, uint8_t code, uint8_t cmd, const uint16_t v[4])
{
  p[0]  = 0x81u;
  p[1]  = (uint8_t)(id >> 8);
  p[2]  = (uint8_t)id;
  p[3]  = 0x00u;
  p[4]  = 0x00u;
  p[5]  = (uint8_t)((rf_ch << 3) | ((v[0] >> 9) & 0x07u));
  p[6]  = (uint8_t)(v[0] >> 1);
  p[7]  = (uint8_t)((v[0] << 7) | ((v[1] >> 5) & 0x7Fu));
  p[8]  = (uint8_t)((v[1] << 3) | ((v[2] >> 9) & 0x07u));
  p[9]  = (uint8_t)(v[2] >> 1);
  p[10] = (uint8_t)((v[2] << 7) | ((v[3] >> 5) & 0x7Fu));
  p[11] = (uint8_t)((v[3] << 3) | ((code >> 2) & 0x07u));
  p[12] = (uint8_t)((code << 6) | cmd);
}

uint16_t sfhss_fix_id(uint32_t seed)                         /* no run of more than six equal bits */
{
  uint32_t id = 1u ^ (seed & 1u);
  uint8_t run = 0;
  for (uint8_t i = 0; i < 16u; i++) {
    id = (id << 1) | (seed & 1u);
    seed >>= 1;
    if ((id & 3u) == 0u || (id & 3u) == 3u) {
      if (++run > 6u) { id ^= 1u; run = 0; }
    } else run = 0;
  }
  return (uint16_t)id;
}

/* ---- the radio ---- */
static uint16_t f_ch_us(uint8_t i)                           /* the input, or centre sticks with the throttle low */
{
  if (f_input) return f_in[i];
  return (i == 2u) ? 1000u : 1500u;
}

static void f_make_pair(void)                                /* packets A and B of this frame (the T8J's order) */
{
  for (uint8_t k = 0; k < 2u; k++) {
    uint8_t cmd = k;                                         /* bit 0: packet B */
    f_counter = (uint16_t)(f_counter + k);
    if ((f_counter & 0x3FCu) == 0x3FCu) {                    /* every 1024 frames (7 s): two failsafe packets, as Multi */
      if (((f_counter & 1u) ^ (cmd & 1u)) == 0u) cmd |= 0x04u;
    } else cmd |= 0x02u;
    f_counter &= 0x3FFu;
    if (f_counter & 1u) cmd |= 0x08u;                        /* channels 5-8 */
    uint8_t off = (cmd & 0x08u) ? 4u : 0u;
    uint16_t v[4];
    for (uint8_t i = 0; i < 4u; i++) {
      if (!(cmd & 0x04u)) v[i] = sfhss_value(f_ch_us((uint8_t)(off + i)));
      else v[i] = (off + i == 2u) ? sfhss_fs_word(sfhss_value(SFHSS_FS_THROTTLE_US)) : 1024u;   /* F/S: throttle low, the rest hold */
    }
    uint8_t *p = k ? f_b : f_a;
    sfhss_build(p, f_id, f_ch, f_code, cmd, v);
    uint16_t c = sfhss_crc(p, SFHSS_PKT_LEN);
    p[SFHSS_PKT_LEN] = (uint8_t)(c >> 8); p[SFHSS_PKT_LEN + 1u] = (uint8_t)c;
    if (cmd & 0x04u) sfhss_diag[SF_FS]++;
  }
}

static void f_launch(const uint8_t *p, uint32_t when)        /* main loop or the RAIL event handler */
{
  (void)sl_rail_write_tx_fifo(f_rail, p, SFHSS_AIR_LEN, true);
  sl_rail_status_t s;
  if ((int32_t)(when - f_now()) < (int32_t)SFHSS_MIN_SLACK_US) {
    sfhss_diag[SF_LATE]++;
    s = sl_rail_start_tx(f_rail, f_ch, SL_RAIL_TX_OPTIONS_DEFAULT, NULL);
  } else {
    sl_rail_scheduled_tx_config_t sc = { .when = when, .mode = SL_RAIL_TIME_ABSOLUTE,
                                         .tx_during_rx = SL_RAIL_SCHEDULED_TX_DURING_RX_POSTPONE_TX };
    s = sl_rail_start_scheduled_tx(f_rail, f_ch, SL_RAIL_TX_OPTIONS_DEFAULT, &sc, NULL);
    if (s != SL_RAIL_STATUS_NO_ERROR) { sfhss_diag[SF_LATE]++; s = sl_rail_start_tx(f_rail, f_ch, SL_RAIL_TX_OPTIONS_DEFAULT, NULL); }
  }
  if (s != SL_RAIL_STATUS_NO_ERROR) { sfhss_diag[SF_TXERR]++; sfhss_diag[SF_CFG_ERR] |= 64; f_stage = 0u; }   /* the main loop restarts the chain */
  else sfhss_diag[SF_TX]++;
}

/* The next frame, on the 6.8 ms grid. A receiver follows that grid and hops once per slot whether it hears a
 * frame or not, so a slot that can no longer be met is skipped WITH its hop: the transmitter is then still where
 * the receiver expects it. (v0.2 sent late and one hop behind: a receiver lost it for up to 2 s.) */
static void f_frame_start(void)
{
  uint32_t now = f_now();
  while ((int32_t)(f_next - now) < (int32_t)SFHSS_START_SLACK_US) {
    f_next += SFHSS_FRAME_US; f_ch = sfhss_next_chan(f_ch, f_code); f_skipped++;
  }
  f_make_pair();
  f_t0 = f_next; f_next += SFHSS_FRAME_US;
  f_stage = 1u; f_tx_count++; sfhss_diag[SF_CH] = f_ch;
  f_launch(f_a, f_t0);
}

/* The frames run from the radio's own events, not from the main loop: packet A done -> packet B (it must start
 * 1.625 ms after A started, 0.19 ms after A ends); packet B done -> the hop and packet A of the next frame.
 * A log line or a flash write in the main loop therefore cannot delay a frame. */
void sfhss_on_events(sl_rail_events_t events)
{
  if (!f_run || f_rx || !(events & SL_RAIL_EVENTS_TX_COMPLETION)) return;
  if (!(events & SL_RAIL_EVENT_TX_PACKET_SENT)) sfhss_diag[SF_TXERR]++;
  if (f_stage == 1u) { f_stage = 2u; f_launch(f_b, f_t0 + SFHSS_PAIR_US); }
  else if (f_stage == 2u) {
    f_ch = sfhss_next_chan(f_ch, f_code);                    /* both packets out: hop */
    if (f_pwr_dirty) {                                       /* RANGE changed: the new power between two frames */
      f_pwr_dirty = false;
      sl_rail_idle(f_rail, SL_RAIL_IDLE, true);
      sl_rail_set_tx_power_dbm(f_rail, f_range ? SFHSS_RANGE_POWER_DDBM : SFHSS_TX_POWER_DDBM);
      f_pwr_log = true;
    }
    f_frame_start();
  }
}

static void x_start(bool bind_held);

bool sfhss_start(sl_rail_handle_t rh, const volatile uint16_t *ch_us, bool range_low, bool rx_role, bool bind_held)
{
  f_rx = rx_role;
  f_rail = rh; f_in = ch_us; f_range = range_low;
  for (uint8_t i = 0; i < SFHSS_DIAG_N; i++) sfhss_diag[i] = 0;
  f_stage = 0; f_run = false; f_input = false; f_pwr_dirty = false; f_pwr_log = false; f_started = false; f_counter = 0; f_ch = 0;
  f_tx_count = 0; f_tx_last = 0; f_skipped = 0;
  sl_gpio_set_pin_mode(&f_led_a, SL_GPIO_MODE_PUSH_PULL, 0);
  sl_gpio_set_pin_mode(&f_led_b, SL_GPIO_MODE_PUSH_PULL, 0);

  if (&sfhss_channelConfig == 0) {                           /* rail_config.c has no sfhss PHY yet */
    sfhss_diag[SF_CFG_ERR] |= 128;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("SF the sfhss radio setting is not in this build: open radio_settings.radioconf in Simplicity Studio, save, build again. RF off\r\n");
#endif
    return false;
  }

  uint64_t u = ((uint64_t)DEVINFO->EUI64H << 32) | DEVINFO->EUI64L;
  uint32_t seed = (uint32_t)(u ^ (u >> 16) ^ (u >> 32) ^ (u >> 48));
  seed ^= (uint32_t)rc_model * 0x9E5u;                       /* per-model binding (0 = unchanged) */
  f_id = sfhss_fix_id(seed);
  f_code = (uint8_t)(((uint32_t)sl_rail_get_time(rh) ^ (seed >> 7)) % 28u);   /* new at every power-up, as a Futaba transmitter */
  sfhss_diag[SF_ID] = f_id; sfhss_diag[SF_CODE] = f_code;

  sl_rail_idle(rh, SL_RAIL_IDLE_ABORT, true);
  f_ent = sfhss_channelConfig.configs[0];                    /* RAM copy with the CC2500's exact frequencies */
  f_ent.baseFrequency = SFHSS_BASE_HZ;
  f_ent.channelSpacing = SFHSS_SPACING_HZ;
  f_cfg = sfhss_channelConfig;
  f_cfg.configs = &f_ent; f_cfg.length = 1u;
  if (sl_rail_config_channels(rh, (const sl_rail_channel_config_t *)&f_cfg, NULL) != SL_RAIL_STATUS_NO_ERROR) sfhss_diag[SF_CFG_ERR] |= 1;
  sl_rail_prepare_channel(rh, 0);
  if (sl_rail_set_fixed_length(rh, SFHSS_AIR_LEN) != SFHSS_AIR_LEN) sfhss_diag[SF_CFG_ERR] |= 2;
  if (sl_rail_set_tx_fifo(rh, f_txfifo, SFHSS_TX_FIFO_BYTES, 0, 0) != SL_RAIL_STATUS_NO_ERROR) sfhss_diag[SF_CFG_ERR] |= 4;
  sl_rail_sync_word_config_t sc = { .sync_word_bits = 32u, .sync_word_0 = SFHSS_SYNC_ON_AIR, .sync_word_1 = SFHSS_SYNC_ON_AIR };
  if (sl_rail_config_sync_words(rh, &sc) != SL_RAIL_STATUS_NO_ERROR) sfhss_diag[SF_CFG_ERR] |= 32;
  if (f_rx) { x_start(bind_held); return true; }
  sl_rail_config_events(rh, SL_RAIL_EVENTS_TX_COMPLETION, SL_RAIL_EVENTS_TX_COMPLETION);
  sl_rail_set_tx_power_dbm(rh, range_low ? SFHSS_RANGE_POWER_DDBM : SFHSS_TX_POWER_DDBM);

#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("SF boot sfhss v0.4 transmitter: id %04X, hop code %u, failsafe sent every 7 s (throttle %u us, the rest hold), 30 channels from %lu Hz every %lu Hz, frame %u us, pair %u us, power %s (flight %d, range %d ddBm; RAIL set %d); cfg_err=%ld\r\n",
          (unsigned)f_id, (unsigned)f_code, (unsigned)SFHSS_FS_THROTTLE_US,
          (unsigned long)SFHSS_BASE_HZ, (unsigned long)SFHSS_SPACING_HZ, (unsigned)SFHSS_FRAME_US, (unsigned)SFHSS_PAIR_US,
          range_low ? "range" : "normal", (int)SFHSS_TX_POWER_DDBM, (int)SFHSS_RANGE_POWER_DDBM,
          (int)sl_rail_get_tx_power_dbm(rh), (long)sfhss_diag[SF_CFG_ERR]);
#endif
  f_uart_us = f_now();
  f_next = f_uart_us + 3000u;
  f_run = true;
  return true;
}

void sfhss_range(bool range_low)
{
  if (range_low == f_range) return;
  f_range = range_low; f_pwr_dirty = true;
}

void sfhss_input(bool input_present)
{
  if (input_present == f_input) return;
  f_input = input_present; sfhss_diag[SF_INPUT] = input_present ? 1 : 0;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (f_run) app_log(input_present ? "SF input present\r\n" : "SF no input: sending centre sticks, throttle low\r\n");
#endif
}

void sfhss_action(void)
{
  uint32_t now = f_now();
  if (f_rx) return;                                            /* the receiver: sfhss_rx_service */
  if (f_run) {
    /* start the chain, or restart it when it stopped (a refused send) or no event came for three frames */
    if (f_stage == 0u || (int32_t)(now - f_t0) > (int32_t)(3u * SFHSS_FRAME_US)) {
      CORE_DECLARE_IRQ_STATE; CORE_ENTER_CRITICAL();
      if (f_stage == 0u || (int32_t)(f_now() - f_t0) > (int32_t)(3u * SFHSS_FRAME_US)) {
        if (f_started) { f_ch = sfhss_next_chan(f_ch, f_code); sfhss_diag[SF_TXERR]++; }   /* the slot that failed keeps its hop */
        f_started = true;
        f_frame_start();
      }
      CORE_EXIT_CRITICAL();
    }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    if (f_pwr_log) {
      f_pwr_log = false;
      app_log("SF RANGE %s: power %d ddBm (RAIL set %d)\r\n", f_range ? "on" : "off",
              (int)(f_range ? SFHSS_RANGE_POWER_DDBM : SFHSS_TX_POWER_DDBM), (int)sl_rail_get_tx_power_dbm(f_rail));
    }
#endif
  }

  /* LED_B: steady with input, slow blink without (fast: no PHY); LED_A toggles per 20 frames */
  static uint32_t tb = 0; static uint32_t la = 0;
  if (f_run && f_input) sl_gpio_set_pin(&f_led_b);
  else if ((now - tb) > (f_run ? 500000u : 100000u)) { tb = now; sl_gpio_toggle_pin(&f_led_b); }
  if (f_tx_count / 20u != la) { la = f_tx_count / 20u; sl_gpio_toggle_pin(&f_led_a); }

#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (f_run && (now - f_uart_us) >= SFHSS_LOG_US) {
    f_uart_us = now;
    app_log("SF tx frames%lu skipped%lu late%ld err%ld in%d h%u ch %u %u %u %u %u %u %u %u\r\n", (unsigned long)(f_tx_count - f_tx_last),
            (unsigned long)f_skipped, (long)sfhss_diag[SF_LATE], (long)sfhss_diag[SF_TXERR], f_input ? 1 : 0, (unsigned)f_ch,
            f_in[0], f_in[1], f_in[2], f_in[3], f_in[4], f_in[5], f_in[6], f_in[7]);
    f_tx_last = f_tx_count;
  }
#endif
}

/* ============================== the receiver role ============================== */
#define SFX_AIR_US          936u       /* sync word end -> packet end: 15 bytes at 128143 bit/s */
#define SFX_B_SLACK_US      500u       /* packet B not in by its end + this: the frame is over */
#define SFX_DWELL_US     300000u       /* search / bind: this long on a channel */
#define SFX_FS_FRAMES       147u       /* 1 s of lost frames: failsafe */
#define SFX_LOST_FRAMES     441u       /* 3 s: search again */
#define SFX_PARK_FRAMES       30u       /* 0.2 s lost: stop hopping by the clock and wait on one channel (the transmitter
                                          passes every channel within 204 ms), a new channel every SFX_PARK_MOVE frames */
#define SFX_PARK_MOVE        44u       /* 0.3 s */
#define SFX_BIND_HITS        10u
#define SFX_BIND_RSSI_DBM  (-75)
#define SFX_BIND_TIMEOUT_US 8000000u   /* BIND at power-up with a transmitter stored: back to it after this */
#define SFX_FS_PRESS_US   300000u

static uint8_t  x_state;                   /* 0 bind, 1 search, 2 link */
static uint16_t x_id, x_cand; static uint8_t x_hits;
static bool     x_have, x_fs_valid, x_in_fs, x_got, x_out_on, x_btn_done, x_btn_armed;
static uint8_t  x_ch, x_code;
static uint32_t x_ts;                      /* sync-word end of packet A of the frame being listened for */
static uint32_t x_dwell_us, x_bind_since, x_last_rx, x_btn_since, x_misses, x_frames, x_lost, x_bad, x_parks;
static uint16_t x_out_us[8], x_fs_us[8], x_txfs_us[8];   /* x_txfs_us: the transmitter's failsafe values, 0 = hold */
static int8_t   x_rssi;
static rx_record_t x_rec;

bool sfhss_rx_fresh(uint32_t now, uint32_t within_us) { return x_last_rx != 0u && (now - x_last_rx) <= within_us; }

static void x_listen(uint8_t ch)
{
  x_ch = ch;
  sl_rail_idle(f_rail, SL_RAIL_IDLE, false);
  (void)sl_rail_start_rx(f_rail, ch, NULL);
}

static void x_set_state(uint8_t st, uint32_t now)
{
  x_state = st; sfhss_diag[SF_STATE] = st; x_dwell_us = now; x_hits = 0;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (st == 0u) app_log("SF rx bind: waiting for a transmitter close by (%d dBm or stronger)\r\n", (int)SFX_BIND_RSSI_DBM);
  else if (st == 1u) app_log("SF rx searching for transmitter %04X\r\n", (unsigned)x_id);
  else app_log("SF rx linked to %04X: channel %u, hop code %u, %d dBm\r\n", (unsigned)x_id, (unsigned)x_ch, (unsigned)x_code, (int)x_rssi);
#endif
}

static void x_start(bool bind_held)
{
  uint32_t now = f_now();
  for (uint8_t i = 0; i < 8u; i++) { x_out_us[i] = 1500u; x_txfs_us[i] = 0u; }
  x_out_us[2] = 1000u;                                       /* throttle low until heard */
  x_misses = 0; x_frames = 0; x_lost = 0; x_bad = 0; x_parks = 0; x_last_rx = 0; x_in_fs = false; x_got = false; x_out_on = false;
  x_have = rx_store_load(RX_SLOT_SFHSS, &x_rec) && x_rec.bound;
  x_id = x_have ? (uint16_t)(((uint16_t)x_rec.hop[0] << 8) | x_rec.hop[1]) : 0u;
  x_fs_valid = x_have && x_rec.fs_valid;
  if (x_fs_valid) for (uint8_t i = 0; i < 8u; i++) x_fs_us[i] = x_rec.fs_us[i];
  servo_out_init(); servo_out_source(x_out_us);
  sbus_init();
  sl_rail_state_transitions_t tr = { .success = SL_RAIL_RF_STATE_RX, .error = SL_RAIL_RF_STATE_RX };   /* packet B follows 0.19 ms after A */
  if (sl_rail_set_rx_transitions(f_rail, &tr) != SL_RAIL_STATUS_NO_ERROR) sfhss_diag[SF_CFG_ERR] |= 8;
  sl_rail_config_events(f_rail, SL_RAIL_EVENT_RX_PACKET_RECEIVED, SL_RAIL_EVENT_RX_PACKET_RECEIVED);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("SF boot sfhss v0.4 receiver: %s%s, failsafe %s; 30 channels from %lu Hz every %lu Hz; cfg_err=%ld\r\n",
          x_have ? "bound" : "not bound", bind_held ? ", BIND held: binding" : "", x_fs_valid ? "preset stored" : "hold",
          (unsigned long)SFHSS_BASE_HZ, (unsigned long)SFHSS_SPACING_HZ, (long)sfhss_diag[SF_CFG_ERR]);
#endif
  f_uart_us = now; x_bind_since = now; f_run = true;
  x_set_state((bind_held || !x_have) ? 0u : 1u, now);
  x_listen(0);
}

static void x_frame_done(uint32_t now)                       /* the frame is over: count it, hop, expect the next */
{
  x_frames++;
  if (x_got) { x_misses = 0; }
  else {
    x_misses++; x_lost++; sfhss_diag[SF_LOST] = (int32_t)x_lost;
    (void)sbus_send(x_out_us, true, x_in_fs);
    if (x_misses == SFX_FS_FRAMES && !x_in_fs) {             /* 1 s: failsafe */
      x_in_fs = true;
      for (uint8_t i = 0; i < 8u; i++) {
        if (x_fs_valid) x_out_us[i] = x_fs_us[i];
        else if (x_txfs_us[i] != 0u) x_out_us[i] = x_txfs_us[i];
      }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("SF rx failsafe (1 s without a frame): %s\r\n", x_fs_valid ? "stored preset" : "hold / the transmitter's values");
#endif
    }
    if (x_misses >= SFX_LOST_FRAMES) { x_set_state(1u, now); x_listen(x_ch); return; }
  }
  x_got = false;
  x_ts += SFHSS_FRAME_US;
  if (x_misses >= SFX_PARK_FRAMES) {                         /* lost for 0.2 s: the clock may no longer fit (the transmitter
                                                                restarted or skipped): wait for it on one channel */
    if (x_misses == SFX_PARK_FRAMES) x_parks++;
    if (((x_misses - SFX_PARK_FRAMES) % SFX_PARK_MOVE) == (SFX_PARK_MOVE - 1u)) x_listen((uint8_t)((x_ch + 7u) % SFHSS_CHANNELS));
    return;
  }
  x_listen(sfhss_next_chan(x_ch, x_code));
}

void sfhss_rx_process(void)
{
  for (;;) {
    sl_rail_rx_packet_info_t info;
    sl_rail_rx_packet_handle_t h = sl_rail_get_rx_packet_info(f_rail, SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &info);
    if (h == SL_RAIL_RX_PACKET_HANDLE_INVALID) return;
    uint8_t b[SFHSS_AIR_LEN];
    uint16_t n = info.packet_bytes;
    sl_rail_rx_packet_details_t det; memset(&det, 0, sizeof(det));
    det.time_received.time_position = SL_RAIL_PACKET_TIME_DEFAULT;
    (void)sl_rail_get_rx_packet_details(f_rail, h, &det);
    det.time_received.total_packet_bytes = info.packet_bytes;
    (void)sl_rail_get_rx_time_sync_word_end(f_rail, &det);
    if (n > SFHSS_AIR_LEN) { info.packet_bytes = SFHSS_AIR_LEN; }
    memset(b, 0, sizeof(b));
    sl_rail_copy_rx_packet(f_rail, b, &info);
    sl_rail_release_rx_packet(f_rail, h);
    if (!f_run || !f_rx) continue;
    uint32_t ts = (uint32_t)det.time_received.packet_time;   /* the end of the sync word */
    uint32_t now = f_now();

    uint16_t rc = (uint16_t)(((uint16_t)b[SFHSS_PKT_LEN] << 8) | b[SFHSS_PKT_LEN + 1u]);
    if (n < SFHSS_AIR_LEN || sfhss_crc(b, SFHSS_PKT_LEN) != rc || (b[0] & 0xFCu) != 0x80u) { x_bad++; continue; }   /* noise, or not S-FHSS */
    uint16_t id  = (uint16_t)(((uint16_t)b[1] << 8) | b[2]);
    uint8_t  rf  = (uint8_t)(b[5] >> 3);
    uint8_t  code = (uint8_t)(((b[11] & 0x07u) << 2) | (b[12] >> 6));
    uint8_t  cmd = (uint8_t)(b[12] & 0x0Fu);
    if (rf != x_ch || rf >= SFHSS_CHANNELS || code > 27u) { x_bad++; continue; }   /* not the channel we listen on */

    if (x_state == 0u) {                                     /* bind: the same strong transmitter, 10 times */
      if (det.rssi_dbm < SFX_BIND_RSSI_DBM) continue;
      if (id == x_cand) x_hits++; else { x_cand = id; x_hits = 1; }
      if (x_hits < SFX_BIND_HITS) continue;
      x_id = id;
      memset(&x_rec, 0, sizeof(x_rec)); x_rec.bound = 1; x_rec.hop[0] = (uint8_t)(id >> 8); x_rec.hop[1] = (uint8_t)id;
      x_fs_valid = false;                                    /* a new bind: the old failsafe preset goes (hold until a new one is stored) */
      x_have = rx_store_save(RX_SLOT_SFHSS, &x_rec);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("SF rx bound to transmitter %04X (%s)\r\n", (unsigned)id, x_have ? "stored" : "NOT stored: flash write failed");
#endif
    } else if (id != x_id) continue;                         /* another S-FHSS transmitter */

    x_rssi = det.rssi_dbm; sfhss_diag[SF_RSSI] = x_rssi;
    x_code = code;
    x_ts = (cmd & 0x01u) ? (ts - SFHSS_PAIR_US) : ts;        /* packet B is 1.625 ms after A */
    x_last_rx = now | 1u;
    if (x_state != 2u) { x_misses = 0; x_set_state(2u, now); }
    x_got = true;
    if (x_in_fs) {
      x_in_fs = false;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("SF rx link back\r\n");
#endif
    }

    uint16_t v[4];                                           /* the four 12-bit values */
    v[0] = (uint16_t)(((uint16_t)(b[5] & 0x07u) << 9) | ((uint16_t)b[6] << 1) | (b[7] >> 7));
    v[1] = (uint16_t)(((uint16_t)(b[7] & 0x7Fu) << 5) | (b[8] >> 3));
    v[2] = (uint16_t)(((uint16_t)(b[8] & 0x07u) << 9) | ((uint16_t)b[9] << 1) | (b[10] >> 7));
    v[3] = (uint16_t)(((uint16_t)(b[10] & 0x7Fu) << 5) | (b[11] >> 3));
    uint8_t off = (cmd & 0x08u) ? 4u : 0u;
    if (cmd & 0x04u) {                                       /* the transmitter's failsafe values, not positions */
      for (uint8_t i = 0; i < 4u; i++) {
        uint16_t us = 0u;                                    /* below 2048: hold (1024..) or no pulses: both held here */
        if (v[i] >= 2048u) us = sfhss_us(sfhss_fs_value(v[i]));   /* a position, on the scale of the live values */
        x_txfs_us[off + i] = us;
      }
    } else {
      for (uint8_t i = 0; i < 4u; i++) x_out_us[off + i] = sfhss_us(v[i]);
      if (!x_out_on) { x_out_on = true; servo_out_enable(true); }
      (void)sbus_send(x_out_us, false, false);
    }
    if (cmd & 0x01u) x_frame_done(now);                      /* packet B: the frame is complete, hop now */
  }
}

void sfhss_rx_service(bool bind_held)
{
  if (!f_run || !f_rx) return;
  if (!bind_held) x_btn_armed = true;                        /* the hold that bound us (BIND at power-up, cfg bind) is not a preset press */
  uint32_t now = f_now();
  sbus_service();
  if (x_state == 2u) {
    if ((int32_t)(now - (x_ts + SFHSS_PAIR_US + SFX_AIR_US + SFX_B_SLACK_US)) > 0) x_frame_done(now);   /* packet B did not come */
    if (bind_held && x_btn_armed) {                          /* BIND pressed 0.3 s while linked: store the failsafe preset */
      if (x_btn_since == 0u) x_btn_since = now | 1u;
      else if (!x_btn_done && (now - x_btn_since) >= SFX_FS_PRESS_US && !x_in_fs) {
        x_btn_done = true;
        for (uint8_t i = 0; i < 8u; i++) x_fs_us[i] = x_out_us[i];
        x_fs_valid = true;
        x_rec.bound = 1; x_rec.hop[0] = (uint8_t)(x_id >> 8); x_rec.hop[1] = (uint8_t)x_id;
        x_rec.fs_valid = 1; for (uint8_t i = 0; i < 8u; i++) x_rec.fs_us[i] = x_fs_us[i];
        bool ok = rx_store_save(RX_SLOT_SFHSS, &x_rec);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("SF rx failsafe preset %s: %u %u %u %u %u %u %u %u\r\n", ok ? "stored" : "NOT stored", x_fs_us[0], x_fs_us[1], x_fs_us[2], x_fs_us[3],
                x_fs_us[4], x_fs_us[5], x_fs_us[6], x_fs_us[7]);
#else
        (void)ok;
#endif
      }
    } else { x_btn_since = 0u; x_btn_done = false; }
  } else {                                                   /* bind / search: the next channel after the dwell */
    if ((now - x_dwell_us) >= SFX_DWELL_US) { x_dwell_us = now; x_listen((uint8_t)((x_ch + 7u) % SFHSS_CHANNELS)); }
    if (x_state == 0u && x_have && (now - x_bind_since) >= SFX_BIND_TIMEOUT_US) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("SF rx no transmitter to bind to in 8 s: back to the stored one\r\n");
#endif
      x_set_state(1u, now);
    }
  }

  /* LED_B: bind fast blink, search slow blink, link steady; LED_A toggles per 20 frames */
  static uint32_t tb = 0; static uint32_t la = 0;
  if (x_state == 2u && !x_in_fs) sl_gpio_set_pin(&f_led_b);
  else if ((now - tb) > (x_state == 0u ? 100000u : 500000u)) { tb = now; sl_gpio_toggle_pin(&f_led_b); }
  if (x_frames / 20u != la) { la = x_frames / 20u; sl_gpio_toggle_pin(&f_led_a); }

#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if ((now - f_uart_us) >= SFHSS_LOG_US) {                   /* every 5 s: the line itself costs a frame or two */
    f_uart_us = now;
    static uint32_t lf = 0, ll = 0;
    app_log("SF rx %s frames%lu lost%lu bad%lu parks%lu rssi%d h%u code%u%s ch %u %u %u %u %u %u %u %u\r\n",
            x_state == 0u ? "bind" : x_state == 1u ? "search" : "link", (unsigned long)(x_frames - lf), (unsigned long)(x_lost - ll),
            (unsigned long)x_bad, (unsigned long)x_parks, (int)x_rssi,
            (unsigned)x_ch, (unsigned)x_code, x_in_fs ? " FAILSAFE" : "",
            x_out_us[0], x_out_us[1], x_out_us[2], x_out_us[3], x_out_us[4], x_out_us[5], x_out_us[6], x_out_us[7]);
    lf = x_frames; ll = x_lost;
  }
#endif
}
