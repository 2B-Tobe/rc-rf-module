/* jeti_cap.c — see jeti_cap.h.  #included into app_process.c.   v1.0 */
#include "jeti_cap.h"
#include "sl_rail_ieee802154.h"
#include "sl_gpio.h"
#include <stdio.h>
#include <string.h>

#ifndef JC_DWELL_US
#define JC_DWELL_US        400000u   /* sweep: time on each channel                                     */
#endif
#ifndef JC_LOCK_SWEEPS
#define JC_LOCK_SWEEPS     2u        /* BIND at power-up: lock on the busiest channel after this many   */
#endif
#ifndef JC_FIXED_CH
#define JC_FIXED_CH        11u       /* F/S at power-up: lock on this channel straight away             */
#endif
#ifndef JC_PRINT_PER_DWELL
#define JC_PRINT_PER_DWELL 8u        /* sweep: frames printed per channel visit, the rest only counted  */
#endif
#ifndef JC_PRINT_PER_SEC
#define JC_PRINT_PER_SEC   25u       /* locked: frames printed per second                                */
#endif
#define JC_CH_FIRST        11u
#define JC_CH_LAST         26u
#define JC_CH_N            16u
#define JC_BUF             140u      /* PHR + 127 + FCS, with room                                       */

volatile int32_t jeti_cap_diag[JETI_CAP_DIAG_N];

static sl_rail_handle_t jc_rail;
static bool     jc_lock_opt, jc_locked;
static uint8_t  jc_ch;                      /* channel listened to now                                   */
static uint32_t jc_ch_since;                /* when we tuned to it                                       */
static uint32_t jc_t0;                      /* start time, for the printed time stamps                   */
static uint32_t jc_last_us[JC_CH_N];        /* last frame time per channel (0 = none yet)                */
static uint32_t jc_ok[JC_CH_N], jc_bad[JC_CH_N], jc_ok_sweep[JC_CH_N], jc_bad_sweep[JC_CH_N];
static uint32_t jc_sweeps, jc_printed, jc_sec_us, jc_printed_sec, jc_skipped;
static volatile bool jc_ovf;
static sl_gpio_t jc_led_a = { .port = SL_GPIO_PORT_C, .pin = 4 };
static sl_gpio_t jc_led_b = { .port = SL_GPIO_PORT_C, .pin = 5 };

static void jc_tune(uint8_t ch)
{
  sl_rail_idle(jc_rail, SL_RAIL_IDLE_ABORT, true);
  jc_ch = ch; jc_ch_since = (uint32_t)sl_rail_get_time(jc_rail);
  jc_printed = 0; jeti_cap_diag[JC_CH] = ch;
  (void)sl_rail_start_rx(jc_rail, ch, NULL);
}

void jeti_cap_start(sl_rail_handle_t rh, bool lock_after_sweeps, bool lock_fixed_ch)
{
  jc_rail = rh;
  memset((void *)jeti_cap_diag, 0, sizeof(jeti_cap_diag));
  memset(jc_last_us, 0, sizeof(jc_last_us));
  memset(jc_ok, 0, sizeof(jc_ok)); memset(jc_bad, 0, sizeof(jc_bad));
  memset(jc_ok_sweep, 0, sizeof(jc_ok_sweep)); memset(jc_bad_sweep, 0, sizeof(jc_bad_sweep));
  jc_sweeps = 0; jc_printed = 0; jc_printed_sec = 0; jc_skipped = 0; jc_ovf = false;
  jc_lock_opt = lock_after_sweeps; jc_locked = false;

  sl_rail_idle(rh, SL_RAIL_IDLE_ABORT, true);
  /* 802.15.4 hardware mode: no addresses, no auto-ACK, every frame type, promiscuous (RAIL then filters
   * nothing but the FCS, and the FCS failures come through as well with the option below). */
  sl_rail_ieee802154_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.p_addresses = NULL;
  cfg.ack_config.enable = false;
  cfg.ack_config.ack_timeout_us = 0;
  cfg.ack_config.rx_transitions.success = SL_RAIL_RF_STATE_RX;
  cfg.ack_config.rx_transitions.error   = SL_RAIL_RF_STATE_RX;
  cfg.ack_config.tx_transitions.success = SL_RAIL_RF_STATE_RX;
  cfg.ack_config.tx_transitions.error   = SL_RAIL_RF_STATE_RX;
  cfg.timings.idle_to_rx = 100; cfg.timings.tx_to_rx = 182; cfg.timings.idle_to_tx = 100; cfg.timings.rx_to_tx = 192;
  cfg.timings.rxsearch_timeout = 0; cfg.timings.tx_to_rxsearch_timeout = 0; cfg.timings.tx_to_tx = 0;
  cfg.frames_mask = SL_RAIL_IEEE802154_ACCEPT_STANDARD_FRAMES | SL_RAIL_IEEE802154_ACCEPT_MULTIPURPOSE_FRAMES;
  cfg.promiscuous_mode = true;
  cfg.is_pan_coordinator = false;
  cfg.default_frame_pending_in_outgoing_acks = false;
  sl_rail_status_t st = sl_rail_ieee802154_init(rh, &cfg);
  if (st != SL_RAIL_STATUS_NO_ERROR) { jeti_cap_diag[JC_INIT] = 100 + (int32_t)st; }
  else {
    st = sl_rail_ieee802154_config_2p4_ghz_radio(rh);           /* the built-in O-QPSK 250 kbps PHY, ch 11..26 */
    if (st != SL_RAIL_STATUS_NO_ERROR) jeti_cap_diag[JC_INIT] = 200 + (int32_t)st;
  }
  if (jeti_cap_diag[JC_INIT] == 0) {
    (void)sl_rail_ieee802154_set_promiscuous_mode(rh, true);
    st = sl_rail_config_rx_options(rh, SL_RAIL_RX_OPTION_STORE_CRC | SL_RAIL_RX_OPTION_IGNORE_CRC_ERRORS,
                                       SL_RAIL_RX_OPTION_STORE_CRC | SL_RAIL_RX_OPTION_IGNORE_CRC_ERRORS);
    if (st != SL_RAIL_STATUS_NO_ERROR) jeti_cap_diag[JC_INIT] = 300 + (int32_t)st;
  }
  sl_gpio_clear_pin(&jc_led_a); sl_gpio_clear_pin(&jc_led_b);
  jc_t0 = (uint32_t)sl_rail_get_time(rh); jc_sec_us = jc_t0;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("JC boot jeti_cap v1.0: 802.15.4 O-QPSK 250 kbps, channels %u..%u (%u MHz steps of 5), dwell %lu ms, init %ld (0 = ok)\r\n",
          JC_CH_FIRST, JC_CH_LAST, 2405u, (unsigned long)(JC_DWELL_US / 1000u), (long)jeti_cap_diag[JC_INIT]);
  app_log("JC mode: %s. Line: JC <s since start> ch<n> <RSSI> <bytes>B ok|FCS dt<ms since the last frame on that channel> | PHR payload FCS\r\n",
          lock_fixed_ch ? "locked on channel 11 (F/S at power-up)" : lock_after_sweeps ? "sweep, then lock on the busiest channel (BIND at power-up)" : "sweep (BIND at power-up = lock on the busiest channel, F/S = lock on ch 11)");
#endif
  if (jeti_cap_diag[JC_INIT] != 0) return;                        /* nothing to listen with: the log says why */
  if (lock_fixed_ch) { jc_locked = true; jeti_cap_diag[JC_LOCKED] = 1; sl_gpio_set_pin(&jc_led_a); jc_tune(JC_FIXED_CH); }
  else jc_tune(JC_CH_FIRST);
}

static void jc_print_frame(const uint8_t *b, uint16_t n, uint8_t ch, int8_t rssi, bool ok, uint32_t t, uint32_t dt, bool have_dt)
{
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  static char line[64 + 3 * JC_BUF];
  static const char hexd[] = "0123456789ABCDEF";
  uint32_t ms = (uint32_t)(t - jc_t0) / 1000u;
  int k;
  if (have_dt) k = snprintf(line, sizeof(line), "JC %lu.%03lu ch%u %4d dBm %3uB %s dt %lu.%01lu |",
                            (unsigned long)(ms / 1000u), (unsigned long)(ms % 1000u), ch, (int)rssi, (unsigned)n,
                            ok ? "ok " : "FCS", (unsigned long)(dt / 1000u), (unsigned long)((dt % 1000u) / 100u));
  else        k = snprintf(line, sizeof(line), "JC %lu.%03lu ch%u %4d dBm %3uB %s dt -    |",
                            (unsigned long)(ms / 1000u), (unsigned long)(ms % 1000u), ch, (int)rssi, (unsigned)n,
                            ok ? "ok " : "FCS");
  if (k < 0) k = 0;
  for (uint16_t i = 0; i < n && (size_t)k + 4u < sizeof(line); i++) {
    line[k++] = ' '; line[k++] = hexd[b[i] >> 4]; line[k++] = hexd[b[i] & 0x0Fu];
  }
  line[k] = 0;
  app_log("%s\r\n", line);
#else
  (void)b; (void)n; (void)ch; (void)rssi; (void)ok; (void)t; (void)dt; (void)have_dt;
#endif
}

void jeti_cap_rx_process(void)
{
  for (;;) {
    sl_rail_rx_packet_info_t info;
    sl_rail_rx_packet_handle_t h = sl_rail_get_rx_packet_info(jc_rail, SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &info);
    if (h == SL_RAIL_RX_PACKET_HANDLE_INVALID) return;
    static uint8_t frame[JC_BUF];
    uint16_t n = info.packet_bytes; if (n > JC_BUF) n = JC_BUF;
    sl_rail_rx_packet_details_t det; memset(&det, 0, sizeof(det));
    det.time_received.time_position = SL_RAIL_PACKET_TIME_DEFAULT;
    det.time_received.total_packet_bytes = info.packet_bytes;
    (void)sl_rail_get_rx_packet_details(jc_rail, h, &det);
    bool ok = (info.packet_status == SL_RAIL_RX_PACKET_READY_SUCCESS);
    info.packet_bytes = n;
    sl_rail_copy_rx_packet(jc_rail, frame, &info);
    sl_rail_release_rx_packet(jc_rail, h);

    uint32_t t = (uint32_t)det.time_received.packet_time;
    uint8_t  ch = jc_ch, ci = (uint8_t)(ch - JC_CH_FIRST);
    if (ci >= JC_CH_N) ci = 0;
    if (ok) { jc_ok[ci]++; jc_ok_sweep[ci]++; jeti_cap_diag[JC_FRAMES]++; }
    else    { jc_bad[ci]++; jc_bad_sweep[ci]++; jeti_cap_diag[JC_BAD]++; }
    bool have_dt = (jc_last_us[ci] != 0u);
    uint32_t dt = have_dt ? (uint32_t)(t - jc_last_us[ci]) : 0u;
    jc_last_us[ci] = t ? t : 1u;
    sl_gpio_toggle_pin(&jc_led_b);

    bool print;
    if (jc_locked) {
      uint32_t now = (uint32_t)sl_rail_get_time(jc_rail);
      if ((uint32_t)(now - jc_sec_us) >= 1000000u) { jc_sec_us = now; jc_printed_sec = 0; }
      print = (jc_printed_sec < JC_PRINT_PER_SEC); if (print) jc_printed_sec++;
    } else {
      print = (jc_printed < JC_PRINT_PER_DWELL); if (print) jc_printed++;
    }
    if (print) jc_print_frame(frame, n, ch, det.rssi_dbm, ok, t, dt, have_dt); else jc_skipped++;
  }
}

void jeti_cap_service(void)
{
  uint32_t now = (uint32_t)sl_rail_get_time(jc_rail);
  if (jc_ovf) { jc_ovf = false; jc_tune(jc_ch); }               /* FIFO overflow: RAIL stops receiving, restart it */
  if (jeti_cap_diag[JC_INIT] != 0) return;

  if (jc_locked) {
    static uint32_t rep = 0;
    if (rep == 0u) rep = now;
    if ((uint32_t)(now - rep) >= 10000000u) {                      /* every 10 s: the count on the locked channel */
      rep = now;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      uint8_t ci = (uint8_t)(jc_ch - JC_CH_FIRST);
      app_log("JC locked ch%u: ok %lu, FCS bad %lu, not printed %lu, overflows %ld\r\n", jc_ch,
              (unsigned long)jc_ok[ci], (unsigned long)jc_bad[ci], (unsigned long)jc_skipped, (long)jeti_cap_diag[JC_OVF]);
#endif
    }
    return;
  }

  if ((uint32_t)(now - jc_ch_since) < JC_DWELL_US) return;
  if (jc_ch < JC_CH_LAST) { jc_tune((uint8_t)(jc_ch + 1u)); return; }

  /* a sweep is complete: census, then lock or go round again */
  jc_sweeps++; jeti_cap_diag[JC_SWEEPS] = (int32_t)jc_sweeps;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  {
    static char line[200];
    int k = snprintf(line, sizeof(line), "JC sweep %lu, frames per channel 11..26 (ok/bad):", (unsigned long)jc_sweeps);
    for (uint8_t i = 0; i < JC_CH_N && k > 0 && (size_t)k < sizeof(line) - 12u; i++)
      k += snprintf(line + k, sizeof(line) - (size_t)k, " %lu/%lu", (unsigned long)jc_ok_sweep[i], (unsigned long)jc_bad_sweep[i]);
    app_log("%s | not printed %lu, overflows %ld\r\n", line, (unsigned long)jc_skipped, (long)jeti_cap_diag[JC_OVF]);
  }
#endif
  if (jc_lock_opt && jc_sweeps >= JC_LOCK_SWEEPS) {
    uint8_t best = 0; uint32_t best_n = 0;
    for (uint8_t i = 0; i < JC_CH_N; i++) { uint32_t v = jc_ok[i] + jc_bad[i]; if (v > best_n) { best_n = v; best = i; } }
    if (best_n > 0u) {
      jc_locked = true; jeti_cap_diag[JC_LOCKED] = 1; sl_gpio_set_pin(&jc_led_a);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("JC lock on ch%u (%lu frames so far): every frame there is printed now, up to %u a second\r\n",
              (unsigned)(JC_CH_FIRST + best), (unsigned long)best_n, (unsigned)JC_PRINT_PER_SEC);
#endif
      jc_sec_us = now; jc_printed_sec = 0;
      jc_tune((uint8_t)(JC_CH_FIRST + best));
      return;
    }
  }
  memset(jc_ok_sweep, 0, sizeof(jc_ok_sweep)); memset(jc_bad_sweep, 0, sizeof(jc_bad_sweep));
  jc_tune(JC_CH_FIRST);
}

void jeti_cap_on_events(sl_rail_events_t ev)
{
  if (ev & SL_RAIL_EVENT_RX_FIFO_OVERFLOW) { jeti_cap_diag[JC_OVF]++; jc_ovf = true; }
  if (ev & (SL_RAIL_EVENT_RX_PACKET_ABORTED | SL_RAIL_EVENT_RX_FRAME_ERROR)) jeti_cap_diag[JC_ABORT]++;
}
