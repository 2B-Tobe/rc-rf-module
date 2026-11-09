/* tx_test.c — #included into app_process.c.   v1.0
 *
 * TX test (PC00 + PC01 + PC02 + F/S low at power-up): an unmodulated carrier, to measure the module's output
 * power and to check that the supply holds up at full load. Connect a dummy load, or an attenuator of at
 * least 30 dB in front of a spectrum analyser such as a TinySA -- never an antenna: the carrier is continuous.
 *
 *   Sweep (default): 2402, 2440 and 2480 MHz, each at 0, 10, 14, 18 and 20 dBm requested, 4 s per step,
 *                    repeating. The requested value is what the protocols ask for (deci-dBm: 180 = 18 dBm).
 *   BIND held at power-up: 2440 MHz at the maximum (200) without stopping -- supply and heat test.
 *
 * The UART (PA05, 115200) prints every step: frequency, requested power and the power RAIL actually set
 * (RAIL caps a request at what the PA can deliver). Compare with the analyser reading plus the attenuation.
 * The carrier sits on the dsm_raw channel grid: channel = MHz above 2400.
 */
#include "rail_config.h"
#if defined(SL_CATALOG_APP_LOG_PRESENT)
#include "app_log.h"
#endif

extern const RAIL_ChannelConfig_t dsm_raw_channelConfig;

#define TT_STEP_US      4000000u      /* each step of the sweep                        */
#define TT_HOLD_CH           40u      /* 2440 MHz                                      */
#define TT_HOLD_DDBM        200       /* the maximum a protocol can ask for             */
#define TT_ALIVE_US    10000000u      /* hold mode: a status line every 10 s           */

static const uint8_t tt_chan[3] = { 2u, 40u, 80u };             /* 2402, 2440, 2480 MHz */
static const int16_t tt_pwr[5]  = { 0, 100, 140, 180, 200 };    /* deci-dBm             */
static sl_rail_handle_t tt_rail;
static bool     tt_hold;
static uint8_t  tt_ci, tt_pi, tt_cur_ch;
static int16_t  tt_cur_ddbm;
static uint32_t tt_next;

static inline uint32_t tt_now(void) { return (uint32_t)sl_rail_get_time(tt_rail); }

static void tt_carrier(uint8_t ch, int16_t ddbm)
{
  (void)sl_rail_stop_tx_stream(tt_rail);
  sl_rail_idle(tt_rail, SL_RAIL_IDLE_ABORT, true);
  sl_rail_set_tx_power_dbm(tt_rail, (sl_rail_tx_power_t)ddbm);
  sl_rail_tx_power_t set = sl_rail_get_tx_power_dbm(tt_rail);
  sl_rail_status_t s = sl_rail_start_tx_stream(tt_rail, ch, SL_RAIL_STREAM_CARRIER_WAVE, SL_RAIL_TX_OPTIONS_DEFAULT);
  tt_cur_ch = ch; tt_cur_ddbm = ddbm;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("TT carrier %u MHz: requested %d ddBm, RAIL set %d ddBm%s\r\n", 2400u + ch, (int)ddbm, (int)set,
          (s == SL_RAIL_STATUS_NO_ERROR) ? "" : "  -- CARRIER DID NOT START");
#else
  (void)s; (void)set;
#endif
}

static void tx_test_start(sl_rail_handle_t rh, bool bind_held)
{
  tt_rail = rh; tt_hold = bind_held; tt_ci = 0; tt_pi = 0;
  sl_rail_idle(rh, SL_RAIL_IDLE_ABORT, true);
  (void)sl_rail_config_channels(rh, (const sl_rail_channel_config_t *)&dsm_raw_channelConfig, NULL);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("TT boot tx_test v1.0: continuous carrier -- DUMMY LOAD OR ATTENUATOR ONLY, NO ANTENNA. %s\r\n",
          bind_held ? "Hold: 2440 MHz at the maximum." : "Sweep: 2402/2440/2480 MHz x 0/10/14/18/20 dBm, 4 s a step.");
#endif
  if (tt_hold) tt_carrier(TT_HOLD_CH, TT_HOLD_DDBM);
  else         tt_carrier(tt_chan[0], tt_pwr[0]);
  tt_next = tt_now() + (tt_hold ? TT_ALIVE_US : TT_STEP_US);
}

static void tx_test_action(void)
{
  uint32_t now = tt_now();
  if ((int32_t)(now - tt_next) < 0) return;
  if (tt_hold) {
    tt_next = now + TT_ALIVE_US;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("TT holding %u MHz, requested %d ddBm, RAIL set %d ddBm\r\n", 2400u + tt_cur_ch, (int)tt_cur_ddbm,
            (int)sl_rail_get_tx_power_dbm(tt_rail));
#endif
    return;
  }
  tt_next += TT_STEP_US;
  if (++tt_pi >= 5u) { tt_pi = 0; if (++tt_ci >= 3u) tt_ci = 0; }
  tt_carrier(tt_chan[tt_ci], tt_pwr[tt_pi]);
}
