/* nrf24_rx.c — see nrf24_rx.h.  #included into app_process.c.
 *
 * Air format = the nRF24 transmitter in app_process.c (= Rf_module_3_packet_tx.ino):
 *   PHY nrf24_250k, sync word = 4-byte pipe address (bytes bit-reversed), fixed 11 bytes:
 *   9-byte payload + CRC16-CCITT (init 0xFFFF) over the address MSByte-first + payload.
 *   41 hop channels from the pipe (xorshift32 + Fisher-Yates), one packet every 11 ms,
 *   cadence PRIMARY, PRIMARY, AUX.  Bind: pipe "000" on hop[0] of the bind sequence.
 *
 * RAIL note: sl_rail_config_channels() only caches the PHY; the first radio operation
 * applies it and resets the sync word to the radio-configuration default (which is the
 * bind pipe, so bind "works" while the private pipe is silently lost).  Hence
 * sl_rail_prepare_channel() right after the config and n_ensure_sync() before every RX. */
#include "nrf24_rx.h"
#include "servo_pins.h"
#include "rail_config.h"
#include "rx_store.h"
#include "sbus_out.h"
#include "sl_gpio.h"
#include <string.h>
#if defined(SL_CATALOG_APP_LOG_PRESENT)
#include "app_log.h"
#endif

extern const RAIL_ChannelConfig_t nrf24_250k_channelConfig;   /* PHY: 2GFSK 250 kbps, 1 MHz channels */

#define NRX_CHANNELS        41u
#define NRX_FRAME_US        11000u
#define NRX_LEN             11u
#define NRX_PAYLOAD         9u
#define NRX_PKT_PRIMARY     0u
#define NRX_PKT_BIND        1u
#define NRX_PKT_AUX         2u
#define NRX_MISS_PAD_US     3000u      /* packet this late -> counted as missed              */
#define NRX_FS_MISSES       40u        /* 440 ms without packets -> failsafe (as the AVR RX) */
#define NRX_LOST_MISSES     60u        /* 660 ms -> search the hop table again               */
#define NRX_SEARCH_DWELL_US 600000u    /* per channel; the TX visits it once per 451 ms      */
#define NRX_BIND_TIMEOUT_US 8000000u   /* BIND at boot with a stored bind: revert after this  */
#define NRX_FS_PRESS_US     300000u    /* BIND held this long while linked -> store preset    */
#define NRX_FS_FLAG_PKTS    3u         /* consecutive flagged PRIMARY packets before storing  */
#define NRX_PPM_CHANNELS    8u
#define NRX_PPM_FRAME_US    22500u
#define NRX_PPM_PULSE_US    300u
#define NRX_THR_FAILSAFE    1100u      /* default failsafe: centre, throttle here           */
#define NRX_THR_IDX         0u         /* throttle = channel 1 (Tobe's encoder sends TAER)  */
#ifndef NRX_UART_STATUS
#define NRX_UART_STATUS     1          /* 1 = status line on the bay UART (PA05) 5x/s      */
#endif
#define NRX_UART_US         200000u

static const uint8_t nrx_bind_pipe[4] = { '0', '0', '0', 0x00 };

volatile int32_t nrf_diag[NRX_DIAG_N];
static sl_rail_handle_t n_rail;
static uint8_t  n_hop[NRX_CHANNELS], n_bhop[NRX_CHANNELS];
static uint8_t  n_pipe[4];
static uint8_t  n_state;                  /* 0 bind, 1 search, 2 link */
static uint8_t  n_idx;
static uint32_t n_expect, n_dwell_us, n_bind_deadline, n_arm_us, n_uart_us;
static uint32_t n_misses;
static bool     n_open, n_binding, n_have_saved, n_in_fs, n_btn_done, n_btn_released, n_flag_armed;
static uint32_t n_btn_since;
static uint8_t  n_flag_run;
static uint16_t n_out_us[8];
static uint8_t  n_rx_buf[NRX_LEN + 8u];
static rx_record_t n_rec, n_saved;
static uint32_t n_want_sync;

static sl_gpio_t n_led_a   = { .port = SL_GPIO_PORT_C, .pin = 4 };
static sl_gpio_t n_led_b   = { .port = SL_GPIO_PORT_C, .pin = 5 };

/* ---- verbatim from the .ino: PRNG, hop table, seed ---- */
static uint32_t n_xs(uint32_t *s){ uint32_t x=*s; x^=x<<13; x^=x>>17; x^=x<<5; *s=x; return x; }
static uint16_t n_seed(const uint8_t *pipe){ uint16_t s=0; for(uint8_t i=0;i<3;i++) s=(uint16_t)(s+pipe[i]); return s; }
static void n_make_hop(uint8_t *h, uint16_t seed)
{
  uint32_t st = seed ? (((uint32_t)seed << 16) | seed) : 0xDEADBEEFu;
  for (uint8_t i = 0; i < NRX_CHANNELS; ++i) h[i] = (uint8_t)((2u*i) + 2u + (uint8_t)(n_xs(&st) & 1u));
  for (uint8_t i = NRX_CHANNELS-1u; i > 0; --i) {
    uint8_t j = (uint8_t)(n_xs(&st) % (i+1u));
    if (j != i) { uint8_t t=h[j]; h[j]=h[i]; h[i]=t; }
  }
}

/* ---- CRC16 as append_crc() in the TX: address MSByte-first (register order is LSByte-first) ---- */
static uint16_t n_crc8(uint16_t crc, uint8_t b)
{
  crc ^= (uint16_t)b << 8;
  for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
  return crc;
}
static uint16_t n_crc(const uint8_t addr[4], const uint8_t *p)
{
  uint16_t c = 0xFFFFu;
  for (int8_t i = 3; i >= 0; i--) c = n_crc8(c, addr[i]);
  for (uint8_t i = 0; i < NRX_PAYLOAD; i++) c = n_crc8(c, p[i]);
  return c;
}

/* ---- sync word = pipe address, bit-reversed bytes (as set_air_address() in the TX) ---- */
static uint8_t n_rev8(uint8_t b)
{
  b = (uint8_t)((b >> 4) | (b << 4));
  b = (uint8_t)(((b & 0xCCu) >> 2) | ((b & 0x33u) << 2));
  b = (uint8_t)(((b & 0xAAu) >> 1) | ((b & 0x55u) << 1));
  return b;
}
static uint32_t n_addr_word(const uint8_t a[4]){ uint32_t w=0; for(uint8_t i=0;i<4;i++) w=(w<<8)|n_rev8(a[i]); return w; }
static void n_apply_sync(void)
{
  sl_rail_idle(n_rail, SL_RAIL_IDLE, true);
  sl_rail_sync_word_config_t sc = { .sync_word_bits=32, .sync_word_0=n_want_sync, .sync_word_1=0 };
  nrf_diag[N_SYNC_ST] = (int32_t)sl_rail_config_sync_words(n_rail, &sc);
}
static void n_set_sync(uint32_t w){ n_want_sync=w; n_apply_sync(); }
static void n_ensure_sync(void)
{
  sl_rail_sync_word_config_t cur;
  if (sl_rail_get_sync_words(n_rail, &cur) != SL_RAIL_STATUS_NO_ERROR || cur.sync_word_0 != n_want_sync) {
    nrf_diag[N_SYNC_FIX]++; n_apply_sync();
  }
}
static void n_listen(uint8_t ch)
{
  nrf_diag[N_CH] = ch; nrf_diag[N_HOP] = n_idx;
  sl_rail_idle(n_rail, SL_RAIL_IDLE, false);
  n_ensure_sync();
  sl_rail_set_fixed_length(n_rail, NRX_LEN);
  if (sl_rail_start_rx(n_rail, ch, NULL) != SL_RAIL_STATUS_NO_ERROR) nrf_diag[N_CFG_ERR] |= 8;
  n_open = true; n_arm_us = (uint32_t)sl_rail_get_time(n_rail);
}


/* ---- PPM + 8 servo pulses: servo_out.c (TIMER2) reads n_out_us ---- */
static void n_ppm_enable(bool on) { servo_out_enable(on); }
static uint32_t n_last_rx;                 /* time of the last valid data packet (dual receiver) */
bool nrf_rx_fresh(uint32_t now, uint32_t within_us) { return n_last_rx != 0u && (now - n_last_rx) <= within_us; }

/* ---- state changes ---- */
static void n_adopt_pipe(const uint8_t *p)
{
  memcpy(n_pipe, p, 4);
  n_make_hop(n_hop, n_seed(n_pipe));
  n_set_sync(n_addr_word(n_pipe));
  nrf_diag[N_PIPE] = (int32_t)(((uint32_t)n_pipe[0]<<24)|((uint32_t)n_pipe[1]<<16)|((uint32_t)n_pipe[2]<<8)|n_pipe[3]);
}
static void n_search(uint32_t now)
{
  n_state = 1; nrf_diag[N_STATE] = 1; nrf_diag[N_SEARCHES]++;
  n_idx = 0; n_dwell_us = now;
  n_listen(n_hop[n_idx]);
}
static void n_failsafe(void)
{
  n_in_fs = true; nrf_diag[N_FS] = 1;
  if (n_rec.fs_valid) memcpy(n_out_us, n_rec.fs_us, sizeof(n_out_us));
  else { for (uint8_t i = 0; i < 8; i++) n_out_us[i] = 1500u; n_out_us[NRX_THR_IDX] = NRX_THR_FAILSAFE; }
}
static void n_store_fs(void)
{
  if (n_state != 2u || n_in_fs) return;
  if (n_rec.fs_valid && memcmp(n_rec.fs_us, n_out_us, sizeof(n_out_us)) == 0) return;   /* unchanged */
  memcpy(n_rec.fs_us, n_out_us, sizeof(n_rec.fs_us)); n_rec.fs_valid = 1;
  if (rx_store_save(RX_SLOT_NRF, &n_rec)) { n_saved = n_rec; nrf_diag[N_FS_STORE]++; }
}

void nrf_rx_start(sl_rail_handle_t rh, bool bind_held)
{
  n_rail = rh;
  for (uint8_t i = 0; i < NRX_DIAG_N; i++) nrf_diag[i] = 0;
  uint32_t now = (uint32_t)sl_rail_get_time(rh);
  n_make_hop(n_bhop, n_seed(nrx_bind_pipe));
  for (uint8_t i = 0; i < 8; i++) n_out_us[i] = 1500u;
  n_out_us[NRX_THR_IDX] = NRX_THR_FAILSAFE;
  n_misses = 0; n_in_fs = false; n_btn_since = 0; n_btn_done = false; n_last_rx = 0;
  n_flag_run = 0; n_flag_armed = false; n_uart_us = now;
  n_btn_released = !bind_held;                     /* a BIND jumper left on from boot stores nothing */

  sl_rail_idle(rh, SL_RAIL_IDLE_ABORT, true);
  if (sl_rail_config_channels(rh, (const sl_rail_channel_config_t *)&nrf24_250k_channelConfig, NULL) != SL_RAIL_STATUS_NO_ERROR) nrf_diag[N_CFG_ERR] |= 1;
  sl_rail_prepare_channel(rh, n_bhop[0]);            /* apply the PHY now: later sync-word writes stick */
  if (sl_rail_set_fixed_length(rh, NRX_LEN) != NRX_LEN) nrf_diag[N_CFG_ERR] |= 2;
  sl_rail_config_events(rh, SL_RAIL_EVENT_RX_PACKET_RECEIVED, SL_RAIL_EVENT_RX_PACKET_RECEIVED);

  sbus_init();
  servo_out_init(); servo_out_source(n_out_us);                     /* pulses: servo_out.c */

  n_have_saved = (rx_store_load(RX_SLOT_NRF, &n_saved) && n_saved.bound);
  if (bind_held) nrf_diag[N_CFG_ERR] |= 128;
  if (bind_held || !n_have_saved) {                  /* listen for the TX's bind burst */
    n_state = 0; nrf_diag[N_STATE] = 0;
    n_binding = bind_held && n_have_saved;           /* stored bind stays the fallback */
    n_bind_deadline = now + NRX_BIND_TIMEOUT_US;
    if (n_have_saved) n_rec = n_saved; else memset(&n_rec, 0, sizeof(n_rec));
    n_set_sync(n_addr_word(nrx_bind_pipe));
    n_idx = 0; n_listen(n_bhop[0]);
  } else {
    n_rec = n_saved;
    n_adopt_pipe(n_rec.hop);
    n_search(now);
  }
}

/* BIND held >300 ms while linked: store the current positions as the failsafe preset. */
static void n_button(bool bind_held, uint32_t now)
{
  if (!bind_held) n_btn_released = true;
  if (!bind_held || !n_btn_released || n_state == 0u) { n_btn_since = 0; n_btn_done = false; return; }
  if (n_btn_since == 0) { n_btn_since = now ? now : 1u; return; }
  if (n_btn_done || (now - n_btn_since) < NRX_FS_PRESS_US) return;
  n_btn_done = true; n_store_fs();
}

void nrf_rx_service(bool bind_held)
{
  sbus_service();
  uint32_t now = (uint32_t)sl_rail_get_time(n_rail);

  if (n_state == 0u) {                                              /* bind */
    if (n_binding && (int32_t)(now - n_bind_deadline) >= 0) {      /* nothing heard: back to the stored TX */
      n_binding = false; n_rec = n_saved; n_adopt_pipe(n_rec.hop); n_search(now);
    } else if (!n_open || (now - n_arm_us) > 100000u) n_listen(n_bhop[0]);   /* keep the radio in RX */
  } else if (n_state == 1u) {                                       /* search: dwell per channel */
    if ((now - n_dwell_us) >= NRX_SEARCH_DWELL_US) {
      n_idx = (uint8_t)((n_idx + 1u) % NRX_CHANNELS); n_dwell_us = now; n_listen(n_hop[n_idx]);
    }
  } else {                                                          /* link: one packet every 11 ms */
    if ((int32_t)(now - (n_expect + NRX_MISS_PAD_US)) >= 0) {       /* missed this frame */
      n_misses++; nrf_diag[N_MISS] = (int32_t)n_misses; nrf_diag[N_LOST]++;
      n_idx = (uint8_t)((n_idx + 1u) % NRX_CHANNELS); n_expect += NRX_FRAME_US;
      n_listen(n_hop[n_idx]);
      if (n_misses == NRX_FS_MISSES) n_failsafe();
      (void)sbus_send(n_out_us, true, n_in_fs);
      if (n_misses >= NRX_LOST_MISSES) n_search(now);
    }
  }
  n_button(bind_held, now);

  /* LED_B: bind fast blink, search slow blink, link steady.  LED_A: toggles per 10 packets, off without link. */
  static uint32_t tb = 0; static int32_t la = 0;
  if (n_state == 2u) sl_gpio_set_pin(&n_led_b);
  else if ((now - tb) > (n_state == 0u ? 100000u : 500000u)) { tb = now; sl_gpio_toggle_pin(&n_led_b); }
  if (n_state != 2u) sl_gpio_clear_pin(&n_led_a);
  else { int32_t a = nrf_diag[N_RX] / 10; if (a != la) { la = a; sl_gpio_toggle_pin(&n_led_a); } }

#if NRX_UART_STATUS && defined(SL_CATALOG_APP_LOG_PRESENT)
  if ((now - n_uart_us) >= NRX_UART_US && sbus_idle()) {            /* ~40 chars = 3.5 ms; only while SBUS is quiet */
    n_uart_us = now;
    static int32_t prx = 0, plost = 0;
    app_log("NR s%d r%ld c%ld m%ld l%ld h%d q%ld f%d\r\n", (int)n_state, (long)(nrf_diag[N_RX] - prx),
            (long)nrf_diag[N_CRC_BAD], (long)n_misses, (long)(nrf_diag[N_LOST] - plost), (int)n_idx,
            (long)nrf_diag[N_RSSI], n_in_fs ? 1 : 0);
    prx = nrf_diag[N_RX]; plost = nrf_diag[N_LOST];
  }
#endif
}

static inline uint16_t n_us10(uint16_t v){ return (uint16_t)((uint32_t)v * 1200u / 1023u + 900u); }
static inline uint16_t n_us8(uint8_t v)  { return (uint16_t)((uint32_t)v * 1200u / 255u + 900u); }

void nrf_rx_process(void)
{
  sl_rail_rx_packet_info_t info;
  sl_rail_rx_packet_handle_t h = sl_rail_get_rx_packet_info(n_rail, SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &info);
  if (h == SL_RAIL_RX_PACKET_HANDLE_INVALID) return;
  uint16_t n = info.packet_bytes; if (n > sizeof(n_rx_buf)) n = sizeof(n_rx_buf);
  sl_rail_rx_packet_details_t det; memset(&det, 0, sizeof(det));
  (void)sl_rail_get_rx_packet_details(n_rail, h, &det);
  (void)sl_rail_get_rx_time_frame_end(n_rail, &det);               /* timestamp = end of packet */
  sl_rail_copy_rx_packet(n_rail, n_rx_buf, &info);
  sl_rail_release_rx_packet(n_rail, h);
  nrf_diag[N_RX]++; nrf_diag[N_RSSI] = det.rssi_dbm;
  uint32_t t   = (uint32_t)det.time_received.packet_time;
  uint32_t now = (uint32_t)sl_rail_get_time(n_rail);

  const uint8_t *addr = (n_state == 0u) ? nrx_bind_pipe : n_pipe;
  bool ok = (n == NRX_LEN) && n_crc(addr, n_rx_buf) == (uint16_t)(((uint16_t)n_rx_buf[NRX_PAYLOAD] << 8) | n_rx_buf[NRX_PAYLOAD + 1u]);
  if (!ok) { nrf_diag[N_CRC_BAD]++; n_listen(n_state == 0u ? n_bhop[0] : n_hop[n_idx]); return; }
  uint8_t type = n_rx_buf[0];

  if (n_state == 0u) {                                              /* bind: take the TX's pipe */
    if (type != NRX_PKT_BIND) { nrf_diag[N_ODD]++; n_listen(n_bhop[0]); return; }
    nrf_diag[N_BIND]++;
    if (!n_have_saved) memset(&n_rec, 0, sizeof(n_rec));
    memcpy(n_rec.hop, &n_rx_buf[2], 4); n_rec.bound = 1;
    if (n_rx_buf[1] == 1u) n_rec.fs_valid = 0;                      /* TX F/S held at bind: default failsafe */
    if (rx_store_save(RX_SLOT_NRF, &n_rec)) { n_saved = n_rec; n_have_saved = true; }
    n_binding = false;
    n_adopt_pipe(n_rec.hop);
    n_search(now);
    return;
  }
  if (type == NRX_PKT_PRIMARY) {
    const uint8_t *p = &n_rx_buf[2];
    n_out_us[0] = n_us10((uint16_t)(p[0] | ((p[1] & 0x03u) << 8)));
    n_out_us[1] = n_us10((uint16_t)((p[1] >> 2) | ((p[2] & 0x0Fu) << 6)));
    n_out_us[2] = n_us10((uint16_t)((p[2] >> 4) | ((p[3] & 0x3Fu) << 4)));
    n_out_us[3] = n_us10((uint16_t)((p[3] >> 6) | ((uint16_t)p[4] << 2)));
    n_out_us[4] = n_us10((uint16_t)(p[5] | ((p[6] & 0x03u) << 8)));
    nrf_diag[N_PRIM]++; nrf_diag[N_SEQ] = p[6] >> 2;
    if (n_rx_buf[1] == 1u) {                                        /* TX F/S button held */
      if (n_flag_run < 255u) n_flag_run++;
      if (n_flag_run >= NRX_FS_FLAG_PKTS && !n_flag_armed) { n_flag_armed = true; n_store_fs(); }
    } else { n_flag_run = 0; n_flag_armed = false; }
  } else if (type == NRX_PKT_AUX) {
    n_out_us[5] = n_us8(n_rx_buf[1]); n_out_us[6] = n_us8(n_rx_buf[2]); n_out_us[7] = n_us8(n_rx_buf[3]);
    nrf_diag[N_AUX]++;
  } else { nrf_diag[N_ODD]++; n_listen(n_hop[n_idx]); return; }

  /* our transmitter, on the channel we listen to: lock the timeline to this packet */
  if (n_state != 2u) { nrf_diag[N_LINKS]++; n_ppm_enable(true); }
  n_state = 2; nrf_diag[N_STATE] = 2;
  n_in_fs = false; nrf_diag[N_FS] = 0; n_misses = 0; nrf_diag[N_MISS] = 0;
  n_last_rx = now;
  n_expect = t + NRX_FRAME_US;
  n_idx = (uint8_t)((n_idx + 1u) % NRX_CHANNELS);
  n_listen(n_hop[n_idx]);
  if (sbus_send(n_out_us, false, false)) nrf_diag[N_SBUS]++;
}
