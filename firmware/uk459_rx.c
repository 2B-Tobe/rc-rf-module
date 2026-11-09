/* uk459_rx.c — see uk459_rx.h.  #included into app_process.c after uk459_tx.c.   v1.0 (branch dual-serial)
 *
 * What is the same as V7 (the transmitter sees no difference):
 *   - every SX register value (init_sx1278 / SetFSKParams, shared with the transmitter: uk_config()), RXON's
 *     write order, PayloadReady + CrcOk as "packet received", the 10-byte packet read from the FIFO
 *   - bind on 458.5375 MHz: three matching packets (ID not 0/255, lane 0 or 1, channel bytes not all 0xFF)
 *   - hop table from ID + lane with V7's generator (the transmitter's uk_gen_hops arithmetic)
 *   - beacon -> hopTo(next); 250 ms fallback hop; 1.2 s -> failsafe, park on hop 0
 *   - failsafe capture packet (byte 9 = 0xFF) and lockout packet (byte 9 = 0x00) handled as V7
 * What differs:
 *   - bind and failsafe preset live in the module's bind page (rx_store slot 7), not an EEPROM
 *   - BIND is the jumper at power-up (bind until heard; 8 s with a stored bind), not a button loop
 *   - the 2 ms settle after a frequency change is a scheduled RXON, not a delay(): nothing blocks
 *   - outputs: 8 servos + PPM on the RAIL timer and SBUS, as the module's other receiver roles
 */
#include "uk459_rx.h"
#include "servo_pins.h"
#include "rx_store.h"
#include "sbus_out.h"
#include "sl_gpio.h"
#include <string.h>
#if defined(SL_CATALOG_APP_LOG_PRESENT)
#include "app_log.h"
#endif

#define UR_SLOT            7u          /* rx_store slot: hop[0] = ID0, hop[1] = ID1, hop[2] = lane            */
#define UR_BIND_TIMEOUT_US 8000000u    /* BIND at boot with a stored bind: revert after this                   */
#define UR_BIND_RESET_US  10000000u    /* V7: bind packet count starts over after this long without one        */
#define UR_DWELL_US         200000u    /* V7 DWELL_MS: 5 packets x 40 ms                                       */
#define UR_GUARD_US          50000u    /* V7 HOP_GUARD_MS                                                      */
#define UR_FS_US           1200000u    /* V7: failsafe after this long without a valid packet                  */
#define UR_SETTLE_US          2000u    /* V7 hopTo(): delay(2) between SetFreq and RXON                        */
#define UR_FS_PRESS_US      300000u    /* BIND held this long while linked -> store preset                     */
#define UR_PPM_CHANNELS          8u
#define UR_PPM_FRAME_US      22500u
#define UR_PPM_PULSE_US        300u
#ifndef UR_UART_STATUS
#define UR_UART_STATUS           1     /* 1 = status line on the bay UART (PA05) 5x/s                          */
#endif
#define UR_UART_US          200000u
#define UK_OP_RX            (UK_OPMODE_MOD | 0x05u)
#define UK_IRQ2_PAYLOAD     0x04u
#define UK_IRQ2_CRCOK       0x02u
#define UK_R_RSSI           0x11u

volatile int32_t ukrx_diag[UR_DIAG_N];
static sl_rail_handle_t ur_rail;
static uint8_t  ur_id[2], ur_lane, ur_n, ur_hops[UK_HOPS_MAX], ur_hop;
static uint8_t  ur_state;                 /* 0 bind, 1 data */
static bool     ur_synced, ur_in_fs, ur_binding, ur_have_saved, ur_rxon_pending, ur_beacon_valid;
static bool     ur_btn_done, ur_btn_released, ur_linked;
static bool     ur_alone;                 /* true: the UK receiver role (outputs, LEDs, log); false: beside a 2.4 GHz role */
static uint8_t  ur_beacon_next, ur_bind_cnt, ur_bind_id[2], ur_bind_lane;
static uint32_t ur_last_hop, ur_last_valid, ur_bind_deadline, ur_bind_last, ur_rxon_at, ur_btn_since, ur_uart_us;
static uint16_t ur_out_us[8];
static uint8_t  ur_buf[UK_PKT_LEN];
static rx_record_t ur_rec, ur_saved;

static sl_gpio_t ur_led_a   = { .port = SL_GPIO_PORT_C, .pin = 4 };
static sl_gpio_t ur_led_b   = { .port = SL_GPIO_PORT_C, .pin = 5 };

/* ---- radio (uk459_tx.c helpers: uk_write, uk_read, uk_xfer, uk_set_freq, uk_config) ---- */
static void ur_read_fifo(uint8_t *p, uint8_t n)       /* V7 spiReadPacket */
{
  UK_LO(uk_nss); uk_ns();
  (void)uk_xfer(UK_R_FIFO);
  for (uint8_t i = 0; i < n; i++) p[i] = uk_xfer(0);
  uk_ns(); UK_HI(uk_nss); uk_ns();
}
static void ur_rxon(void)                             /* V7 RXON */
{
  uk_write(UK_R_IRQ1, 0xFF);
  uk_write(UK_R_IRQ2, 0xFF);
  uk_write(UK_R_OPMODE, UK_OP_STDBY);
  uk_write(UK_R_PKTCFG1, UK_PKTCFG1_VAL);
  uk_write(UK_R_PAYLEN, UK_PKT_LEN);
  uk_write(UK_R_FIFOTHR, (uint8_t)(0x80u | (UK_PKT_LEN - 1u)));
  uk_write(UK_R_PACONFIG, UK_PA_IDLE);
  uk_write(UK_R_OPMODE, UK_OP_RX);
  ur_rxon_pending = false;
}
static inline uint32_t ur_hop_hz(uint8_t idx) { return UK_BASE_HZ + UK_STEP_HZ * (uint32_t)ur_hops[idx]; }
static void ur_tune(uint32_t hz, uint32_t now)        /* V7 hopTo(): RTXOFF, SetFreq, 2 ms, RXON (scheduled) */
{
  uk_write(UK_R_OPMODE, UK_OP_STDBY);
  uk_write(UK_R_PACONFIG, UK_PA_IDLE);
  uk_set_freq(hz);
  ur_rxon_pending = true; ur_rxon_at = now + UR_SETTLE_US;
  ur_last_hop = now;
}
static void ur_hop_to(uint8_t idx, uint32_t now)
{
  ur_hop = idx; ukrx_diag[UR_HOP] = idx;
  ur_tune(ur_hop_hz(idx), now);
}

/* ---- hop table: V7 generateHopSequence = V9's, verbatim arithmetic (16-bit seed) ---- */
static void ur_gen_hops(void)
{
  uint16_t seed = (uint16_t)(ur_id[0] + ((uint16_t)ur_id[1] << 8));
  if (seed == 0u) seed = 0x2A5Bu;
  ur_n = (ur_lane == 1u) ? 19u : 20u;
  for (uint8_t i = 0; i < ur_n; i++) ur_hops[i] = (ur_lane == 1u) ? (uint8_t)(2u * i + 1u) : (uint8_t)(2u * i);
  for (uint8_t i = (uint8_t)(ur_n - 1u); i > 0u; i--) {
    seed = (uint16_t)(seed * 25173u + 13849u);
    uint8_t j = (uint8_t)(seed % (uint16_t)(i + 1u));
    uint8_t t = ur_hops[i]; ur_hops[i] = ur_hops[j]; ur_hops[j] = t;
  }
}
static void ur_adopt(const rx_record_t *r)
{
  ur_id[0] = r->hop[0]; ur_id[1] = r->hop[1]; ur_lane = (r->hop[2] == 1u) ? 1u : 0u;
  ur_gen_hops();
}

/* ---- outputs: servo_out.c (TIMER2) reads ur_out_us; nothing of it in "beside" mode ---- */
static void ur_ppm_enable(bool on) { if (ur_alone) servo_out_enable(on); }

/* ---- failsafe ---- */
static void ur_apply_failsafe(void)                   /* V7 applyFailsafe */
{
  if (ur_rec.fs_valid) memcpy(ur_out_us, ur_rec.fs_us, sizeof(ur_out_us));
  else { for (uint8_t i = 0; i < 8; i++) ur_out_us[i] = 1500u; ur_out_us[0] = 1000u; }
}
static void ur_store_fs(const uint16_t *us)
{
  if (ur_rec.fs_valid && memcmp(ur_rec.fs_us, us, sizeof(ur_rec.fs_us)) == 0) return;   /* unchanged */
  memcpy(ur_rec.fs_us, us, sizeof(ur_rec.fs_us)); ur_rec.fs_valid = 1;
  if (rx_store_save(UR_SLOT, &ur_rec)) { ur_saved = ur_rec; ukrx_diag[UR_FSSTORE]++; }
}
static inline uint16_t ur_us(uint8_t b){ uint16_t us = (uint16_t)(950u + 4u * b); return us < 1000u ? 1000u : us > 2000u ? 2000u : us; }

/* ---- start ---- */
void uk_rx_start(sl_rail_handle_t rh, bool bind_held, bool alone)
{
  ur_rail = rh; ur_alone = alone;
  for (uint8_t i = 0; i < UR_DIAG_N; i++) ukrx_diag[i] = 0;
  uint32_t now = (uint32_t)sl_rail_get_time(rh);
  for (uint8_t i = 0; i < 8; i++) ur_out_us[i] = 1500u;
  ur_out_us[0] = 1000u;
  ur_synced = false; ur_in_fs = false; ur_linked = false; ur_beacon_valid = false;
  ur_btn_since = 0; ur_btn_done = false; ur_btn_released = !bind_held; ur_uart_us = now;
  ur_bind_cnt = 0; ur_bind_last = 0; ur_rxon_pending = false;

  if (ur_alone) {
    sl_rail_idle(rh, SL_RAIL_IDLE, true);                          /* the 2.4 GHz radio stays idle in this role */
    sbus_init();
    servo_out_init(); servo_out_source(ur_out_us);                 /* pulses: servo_out.c */
  }

  uk_config();                                                     /* V7 init_sx1278 (same registers as V9) */

  ur_have_saved = (rx_store_load(UR_SLOT, &ur_saved) && ur_saved.bound);
  if (bind_held || !ur_have_saved) {                               /* bind: listen on the bind frequency */
    ur_state = 0; ukrx_diag[UR_STATE] = 0;
    ur_binding = bind_held && ur_have_saved;                       /* stored bind stays the fallback */
    ur_bind_deadline = now + UR_BIND_TIMEOUT_US;
    if (ur_have_saved) ur_rec = ur_saved; else memset(&ur_rec, 0, sizeof(ur_rec));
    ur_tune(UK_BIND_HZ, now);
  } else {
    ur_rec = ur_saved;
    ur_adopt(&ur_rec);
    ur_state = 1; ukrx_diag[UR_STATE] = 1;
    ur_hop_to(0, now);                                             /* V7: park on hop 0 and listen */
  }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("UR boot uk459_rx v1.0: %s%s; ID %02X %02X lane %s (%u channels); failsafe %s\r\n",
          ur_state == 0u ? "bind on 458.5375 MHz" : "data from hop 0",
          (ur_state == 0u && ur_have_saved) ? " (8 s, then the stored transmitter)" : "",
          ur_state == 0u ? 0u : ur_id[0], ur_state == 0u ? 0u : ur_id[1], ur_lane ? "odd" : "even", (unsigned)ur_n,
          ur_rec.fs_valid ? "stored preset" : "throttle 1000, others centre");
#endif
}

/* ---- packets ---- */
static bool ur_bind_packet(uint32_t now)              /* V7 processBindPacket */
{
  uint8_t id0 = ur_buf[8], id1 = ur_buf[9], lane = ur_buf[7];
  if (id0 == 0u || id0 == 255u || id1 == 0u || id1 == 255u) return false;
  uint8_t ff = 0; for (uint8_t i = 0; i < 7; i++) if (ur_buf[i] == 0xFFu) ff++;
  if (ff >= 6u) return false;
  if (lane > 1u) return false;
  if (ur_bind_cnt != 0u && (now - ur_bind_last) > UR_BIND_RESET_US) ur_bind_cnt = 0;
  ur_bind_last = now;
  if (ur_bind_cnt == 0u || id0 != ur_bind_id[0] || id1 != ur_bind_id[1] || lane != ur_bind_lane) {
    ur_bind_id[0] = id0; ur_bind_id[1] = id1; ur_bind_lane = lane; ur_bind_cnt = 1;
  } else if (++ur_bind_cnt >= 3u) {                               /* three matching packets: bound */
    if (!ur_have_saved) memset(&ur_rec, 0, sizeof(ur_rec));
    ur_rec.hop[0] = id0; ur_rec.hop[1] = id1; ur_rec.hop[2] = lane; ur_rec.bound = 1;
    if (rx_store_save(UR_SLOT, &ur_rec)) { ur_saved = ur_rec; ur_have_saved = true; }
    ur_adopt(&ur_rec);
    ur_binding = false; ur_bind_cnt = 0;
    ur_state = 1; ukrx_diag[UR_STATE] = 1; ukrx_diag[UR_BIND]++;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("UR bound: ID %02X %02X lane %s (%u channels), stored; data from hop 0\r\n", id0, id1, lane ? "odd" : "even", (unsigned)ur_n);
#endif
    ur_hop_to(0, now);
    return true;
  }
  for (uint8_t i = 0; i < 8; i++) ur_out_us[i] = ur_us(ur_buf[i]);  /* V7: servos follow during bind */
  return true;
}

static void ur_packet(uint32_t now)                   /* V7 validatePacket + the loop's valid-packet branch */
{
  if (ur_state == 0u) { (void)ur_bind_packet(now); return; }
  uint8_t b8 = ur_buf[8], b9 = ur_buf[9];
  if (b8 == ur_id[0] && b9 == 0xFFu) {                             /* failsafe capture (V9 D5) */
    uint16_t us[8]; for (uint8_t i = 0; i < 8; i++) us[i] = ur_us(ur_buf[i]);
    ur_store_fs(us); return;
  }
  if (b8 == ur_id[0] && b9 == 0x00u) return;                       /* lockout packet: ignore */
  bool beacon = false;
  if (b9 == 0xFEu) {                                               /* sync beacon: byte 8 = ID0 ^ next index */
    uint8_t idx = (uint8_t)(b8 ^ ur_id[0]);
    if (idx >= ur_n) { ukrx_diag[UR_ODD]++; return; }
    ur_beacon_next = idx; beacon = true;
  } else if (!(b8 == ur_id[0] && b9 == ur_id[1])) { ukrx_diag[UR_ODD]++; return; }

  ukrx_diag[UR_RX]++;
  ur_last_valid = now; ur_synced = true;
  if (ur_in_fs) { ur_in_fs = false; ukrx_diag[UR_FS] = 0; }
  if (!ur_linked) { ur_linked = true; ukrx_diag[UR_LINKS]++; ur_ppm_enable(true); }
  for (uint8_t i = 0; i < 8; i++) ur_out_us[i] = ur_us(ur_buf[i]);
  if (ur_alone && sbus_send(ur_out_us, false, false)) ukrx_diag[UR_SBUS]++;
  if (beacon) { ukrx_diag[UR_BEACON]++; ur_hop_to(ur_beacon_next, now); }
}

/* BIND held >300 ms while linked: store the current positions as the failsafe preset. */
static void ur_button(bool bind_held, uint32_t now)
{
  if (!bind_held) ur_btn_released = true;
  if (!bind_held || !ur_btn_released || ur_state == 0u || !ur_linked || ur_in_fs) { ur_btn_since = 0; ur_btn_done = false; return; }
  if (ur_btn_since == 0) { ur_btn_since = now ? now : 1u; return; }
  if (ur_btn_done || (now - ur_btn_since) < UR_FS_PRESS_US) return;
  ur_btn_done = true; ur_store_fs(ur_out_us);
}

void uk_rx_service(bool bind_held)
{
  if (ur_alone) sbus_service();
  uint32_t now = (uint32_t)sl_rail_get_time(ur_rail);

  if (ur_rxon_pending) {                                           /* 2 ms after a frequency change: listen */
    if ((int32_t)(now - ur_rxon_at) >= 0) ur_rxon();
  } else {
    uint8_t i2 = uk_read(UK_R_IRQ2);                               /* V7 isRxDone: payload ready with CRC ok */
    if ((i2 & UK_IRQ2_PAYLOAD) && (i2 & UK_IRQ2_CRCOK)) {
      ur_read_fifo(ur_buf, UK_PKT_LEN);
      ukrx_diag[UR_RSSI] = -(int32_t)(uk_read(UK_R_RSSI) >> 1);
      ur_packet(now);
    }
  }

  if (ur_state == 0u) {                                            /* bind */
    if (ur_binding && (int32_t)(now - ur_bind_deadline) >= 0) {    /* nothing heard: back to the stored TX */
      ur_binding = false; ur_rec = ur_saved; ur_adopt(&ur_rec);
      ur_state = 1; ukrx_diag[UR_STATE] = 1;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("UR bind: nothing heard in 8 s, stored transmitter ID %02X %02X kept; data from hop 0\r\n", ur_id[0], ur_id[1]);
#endif
      ur_hop_to(0, now);
    }
  } else {                                                         /* data */
    if (ur_synced && (int32_t)(now - (ur_last_hop + UR_DWELL_US + UR_GUARD_US)) >= 0) {   /* missed beacon */
      ukrx_diag[UR_FALLBACK]++;
      ur_hop_to((uint8_t)((ur_hop + 1u) % ur_n), now);
    }
    if (ur_last_valid != 0u && (now - ur_last_valid) > UR_FS_US) { /* V7: failsafe, park on hop 0 */
      ur_apply_failsafe();
      ur_in_fs = true; ukrx_diag[UR_FS] = 1;
      ur_synced = false;
      ur_last_valid = 0;
      if (ur_alone) (void)sbus_send(ur_out_us, true, true);
      ur_hop_to(0, now);
    }
  }
  if (ur_alone) ur_button(bind_held, now);
  if (!ur_alone) {                                                 /* beside a 2.4 GHz role: its LEDs and log */
#if UR_UART_STATUS && defined(SL_CATALOG_APP_LOG_PRESENT)
    if ((now - ur_uart_us) >= 1000000u && sbus_idle()) { ur_uart_us = now;
      static int32_t prx2 = 0; app_log("UR s%d r%ld h%d q%ld f%d (beside)\r\n", (int)ur_state, (long)(ukrx_diag[UR_RX] - prx2), (int)ur_hop, (long)ukrx_diag[UR_RSSI], ur_in_fs ? 1 : 0); prx2 = ukrx_diag[UR_RX]; }
#endif
    return;
  }

  /* LED_B: bind fast blink, no link slow blink, link steady.  LED_A: toggles per 10 packets, off without link. */
  static uint32_t tb = 0; static int32_t la = 0;
  bool link = (ur_state == 1u) && ur_last_valid != 0u;
  if (link) sl_gpio_set_pin(&ur_led_b);
  else if ((now - tb) > (ur_state == 0u ? 100000u : 500000u)) { tb = now; sl_gpio_toggle_pin(&ur_led_b); }
  if (!link) sl_gpio_clear_pin(&ur_led_a);
  else { int32_t a = ukrx_diag[UR_RX] / 10; if (a != la) { la = a; sl_gpio_toggle_pin(&ur_led_a); } }

#if UR_UART_STATUS && defined(SL_CATALOG_APP_LOG_PRESENT)
  if ((now - ur_uart_us) >= UR_UART_US && sbus_idle()) {           /* ~40 chars = 3.5 ms; only while SBUS is quiet */
    ur_uart_us = now;
    static int32_t prx = 0, pbc = 0, pfb = 0;
    app_log("UR s%d r%ld b%ld k%ld h%d q%ld f%d\r\n", (int)ur_state, (long)(ukrx_diag[UR_RX] - prx),
            (long)(ukrx_diag[UR_BEACON] - pbc), (long)(ukrx_diag[UR_FALLBACK] - pfb), (int)ur_hop,
            (long)ukrx_diag[UR_RSSI], ur_in_fs ? 1 : 0);
    prx = ukrx_diag[UR_RX]; pbc = ukrx_diag[UR_BEACON]; pfb = ukrx_diag[UR_FALLBACK];
  }
#endif
}

const volatile uint16_t *uk_rx_channels(void) { return (const volatile uint16_t *)ur_out_us; }
bool uk_rx_fresh(uint32_t now, uint32_t within_us) { return ur_last_valid != 0u && (now - ur_last_valid) <= within_us; }
