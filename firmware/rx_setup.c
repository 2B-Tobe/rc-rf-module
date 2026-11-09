/* rx_setup.c — see rx_setup.h.  #included into app_process.c after rlink.c and cfg.c (it uses RLINK's radio
 * helpers: same PHY, same 16-byte length; only the channel's sync word differs).   v0.1 bench proof, RLINK */
#include "rx_setup.h"

static bool rxs_pairing;                         /* the bind plug was on servo connector 8 at power-up (app_process.c) */

static void rxs_owner_id(uint8_t id[6])          /* this module's chip number, EUI-64 low 48 bits */
{
  uint64_t u = ((uint64_t)DEVINFO->EUI64H << 32) | DEVINFO->EUI64L;
  for (uint8_t i = 0; i < 6u; i++) id[i] = (uint8_t)(u >> (8u * i));
}

/* ------------------------------------------------------------------------------------------- TX module */
static bool     rxs_tx_want, rxs_tx_on;
static uint8_t  rxs_tx_proto, rxs_tx_sub, rxs_tx_layout;
static uint32_t rxs_tx_next, rxs_tx_sent;

void rxs_tx_request(bool on, uint8_t proto, uint8_t sub, uint8_t layout)
{
  rxs_tx_want = on; rxs_tx_proto = proto; rxs_tx_sub = sub; rxs_tx_layout = (uint8_t)(layout & RXOUT_MAX);
}

static void rxs_tx_pack(void)
{
  memset(r_pkt, 0, RL_FRAME_LEN);
  r_pkt[0] = RXS_TYPE;
  rxs_owner_id(&r_pkt[1]);
  r_pkt[7] = rxs_tx_proto; r_pkt[8] = rxs_tx_sub; r_pkt[9] = rxs_tx_layout;
  if (rxs_tx_proto == 0u && rxs_tx_sub == 1u) memcpy(&r_pkt[10], r_id, 4);   /* RLINK: the ID we fly with; other modes: bound later */
  uint16_t c = r_crc(r_pkt, RL_PAYLOAD);
  r_pkt[RL_PAYLOAD] = (uint8_t)(c >> 8); r_pkt[RL_PAYLOAD + 1u] = (uint8_t)c;
}

/* rlink.c TX role, every loop: true = the setup owns the radio (no flying data) */
static bool rxs_tx_service(uint32_t now)
{
  if (rxs_tx_want && !rxs_tx_on) {               /* start: our private RLINK ID, as an RLINK bind would take it */
    rxs_tx_on = true; rxs_tx_sent = 0; rxs_tx_next = now;
    if (rxs_tx_proto == 0u && rxs_tx_sub == 1u) {
      r_gen_id();
      rx_record_t r; memset(&r, 0, sizeof r); r.bound = 1; (void)rx_store_save(RX_SLOT_RLINKTX, &r);
    }
    sl_rail_set_tx_power_dbm(r_rail, RXS_POWER_DDBM);
    r_set_sync(RXS_SYNC);
    r_tx_done = true;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("RXS sending setup packets: Prot %u Sub %u, outputs %s, -10 dBm on 2460 MHz (RxSetup)\r\n",
            rxs_tx_proto, rxs_tx_sub, servo_out_layout_name(rxs_tx_layout));
#endif
  }
  if (!rxs_tx_want && rxs_tx_on) {               /* stop: fly with the private ID */
    rxs_tx_on = false;
    sl_rail_idle(r_rail, SL_RAIL_IDLE_ABORT, true);
    r_set_sync(r_data_sync);
    sl_rail_set_tx_power_dbm(r_rail, RL_TX_POWER_DDBM);
    r_frame_us = now + RL_FRAME_US; r_tx_state = 0; r_base = 0;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("RXS setup over: %lu packets sent; flying RLINK with this module's own ID\r\n", (unsigned long)rxs_tx_sent);
#endif
    return false;
  }
  if (!rxs_tx_on) return false;
  if ((int32_t)(now - rxs_tx_next) >= 0 && (r_tx_done || (int32_t)(now - rxs_tx_next) >= (int32_t)RXS_TX_EVERY_US)) {
    rxs_tx_next = now + RXS_TX_EVERY_US;
    r_tx_done = false;
    rxs_tx_pack(); r_tx_now(RXS_CH); rxs_tx_sent++;
  }
  return true;
}

/* ------------------------------------------------------------------------------------------- receiver */
static bool     rxs_rx_in;                       /* the slot is listening */
static uint32_t rxs_rx_next, rxs_rx_until;
static uint8_t  rxs_last[RL_PAYLOAD], rxs_same;
static uint32_t rxs_heard, rxs_weak, rxs_stranger;
static bool     rxs_done_noted;

static bool rxs_rx_listening(void) { return rxs_rx_in; }

/* Bind plug at power-up, any receiver mode: before the role starts, listen on the setup channel for 'us'. A setup
 * heard there is stored and the board restarts (rxs_apply); otherwise the role starts as stored, binding. This is
 * how a receiver in any mode (DSMX, D8, D16, nRF24) is given another mode: its own role never listens for setup. */
static void rxs_window(sl_rail_handle_t rh, uint32_t us)
{
  static uint8_t buf[RL_FRAME_LEN + 2u];
  sl_rail_idle(rh, SL_RAIL_IDLE_ABORT, true);
  sl_rail_config_channels(rh, (const sl_rail_channel_config_t *)&nrf24_250k_channelConfig, NULL);
  sl_rail_prepare_channel(rh, RXS_CH);                  /* the PHY now, then our sync word (it would be overwritten) */
  sl_rail_set_fixed_length(rh, RL_FRAME_LEN);
  sl_rail_sync_word_config_t sc = { .sync_word_bits = 32, .sync_word_0 = RXS_SYNC, .sync_word_1 = 0 };
  (void)sl_rail_config_sync_words(rh, &sc);
  sl_rail_config_events(rh, SL_RAIL_EVENT_RX_PACKET_RECEIVED, SL_RAIL_EVENT_RX_PACKET_RECEIVED);
  rx_held = false; rxs_rx_in = true;
  (void)sl_rail_start_rx(rh, RXS_CH, NULL);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("RXS bind plug: listening %lu s for setup packets (RxSetup), then the stored mode binds\r\n", (unsigned long)(us / 1000000u));
#endif
  uint32_t t0 = (uint32_t)sl_rail_get_time(rh);
  while ((uint32_t)sl_rail_get_time(rh) - t0 < us) {
    if (!rx_held) continue;
    rx_held = false;
    sl_rail_rx_packet_info_t info;
    sl_rail_rx_packet_handle_t h = sl_rail_get_rx_packet_info(rh, SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &info);
    if (h != SL_RAIL_RX_PACKET_HANDLE_INVALID) {
      uint16_t n = info.packet_bytes; if (n > sizeof buf) n = sizeof buf;
      sl_rail_copy_rx_packet(rh, buf, &info);
      sl_rail_rx_packet_details_t det; int8_t rssi = -128;
      if (sl_rail_get_rx_packet_details(rh, h, &det) == SL_RAIL_STATUS_NO_ERROR) rssi = det.rssi_dbm;
      sl_rail_release_rx_packet(rh, h);
      rxs_rx_packet(buf, n, rssi);                        /* 3 alike: stored, restart (does not return) */
    }
    (void)sl_rail_start_rx(rh, RXS_CH, NULL);
  }
  rxs_rx_in = false;
  sl_rail_idle(rh, SL_RAIL_IDLE_ABORT, true);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("RXS no setup packets (%lu heard, %lu too weak): starting the stored mode, binding\r\n", (unsigned long)rxs_heard, (unsigned long)rxs_weak);
#endif
}

static uint8_t rxs_rx_slot(uint32_t now, bool searching)
{
  if (!searching) { if (rxs_rx_in) { rxs_rx_in = false; return 2u; } return 0u; }
  if (rxs_rx_in) {
    if ((int32_t)(now - rxs_rx_until) < 0) return 1u;
    rxs_rx_in = false;
    return 2u;
  }
  if ((int32_t)(now - rxs_rx_next) < 0) return 0u;
  rxs_rx_next = now + RXS_SLOT_EVERY_US; rxs_rx_until = now + RXS_SLOT_US;
  rxs_rx_in = true;
  r_set_sync(RXS_SYNC); r_listen(RXS_CH);
  return 1u;
}

static void rxs_apply(const uint8_t *b)
{
  cfg_t c; memset(&c, 0, sizeof c);
  c.role = CFG_ROLE_RX; c.proto = b[7]; c.sub = b[8]; c.outputs = (uint8_t)(b[9] & RXOUT_MAX);
  const char *nm = cfg_proto_name(CFG_ROLE_RX, c.proto, c.sub);
  if (nm == NULL) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("RXS refused: Prot %u Sub %u is not a receiver mode of this module\r\n", c.proto, c.sub);
#endif
    cfg_blink(false); rxs_same = 0; return;
  }
  { cfg_t now_c; rx_record_t r;                 /* already set up exactly so (we restarted, the TX still sends): nothing to do */
    bool same = cfg_load(&now_c) && now_c.role == c.role && now_c.proto == c.proto && now_c.sub == c.sub && now_c.outputs == c.outputs
             && (!rxs_pairing || cfg_owner_known(&b[1]));
    if (same && c.proto == 0u && c.sub == 1u) same = rx_store_load(RX_SLOT_RLINK, &r) && r.bound && memcmp(r.hop, &b[10], 4) == 0;
    if (same) {
      if (!rxs_done_noted) {
        rxs_done_noted = true;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("RXS already set up so: end RxSetup on the transmitter and it links\r\n");
#endif
      }
      rxs_same = 0; return;
    } }
  bool paired = false;
  if (rxs_pairing && !cfg_owner_known(&b[1])) paired = cfg_owner_add(&b[1]);
  bool ok = cfg_save(&c);
  if (ok && c.proto == 0u && c.sub == 1u) {      /* RLINK: bound to the sender's ID now (the failsafe preset stays) */
    rx_record_t r;
    if (!(rx_store_load(RX_SLOT_RLINK, &r) && r.bound)) memset(&r, 0, sizeof r);
    memcpy(r.hop, &b[10], 4); r.bound = 1;
    ok = rx_store_save(RX_SLOT_RLINK, &r);
  }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("RXS %s: %s, outputs %s%s%s; restarting\r\n", ok ? "set up" : "store FAILED", nm, servo_out_layout_name(c.outputs),
          paired ? ", paired with this TX module" : "", (c.proto == 0u && c.sub == 1u) ? ", bound" : " (bind it the usual way)");
#endif
  cfg_blink(ok);
  cfg_restart();
}

static void rxs_rx_packet(const uint8_t *b, uint16_t n, int8_t rssi)
{
  if (n != RL_FRAME_LEN || b[0] != RXS_TYPE) return;
  if (r_crc(b, RL_PAYLOAD) != (uint16_t)((b[RL_PAYLOAD] << 8) | b[RL_PAYLOAD + 1u])) return;
  rxs_heard++;
  if (rssi < RXS_RSSI_MIN) { rxs_weak++; rxs_same = 0; return; }           /* too far: a bench affair */
  if (!rxs_pairing && !cfg_owner_known(&b[1])) {                           /* not one of our owners */
    if (rxs_stranger++ == 0u) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("RXS setup packets from a TX module that is not an owner: ignored (pair it: bind plug on servo connector 8 at power-up)\r\n");
#endif
    }
    return;
  }
  if (rxs_same && memcmp(rxs_last, b, RL_PAYLOAD) == 0) rxs_same++;
  else { memcpy(rxs_last, b, RL_PAYLOAD); rxs_same = 1; }
  if (rxs_same >= RXS_NEED) rxs_apply(b);
}
