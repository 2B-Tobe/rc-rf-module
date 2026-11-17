/*
 * tx_panel.c — TX panel: the module's own display, three or four buttons and a red / green LED.   v0.3
 * Build options in panel_config.h: input MULTI / PPM / MULTI + PPM, 3 or 4 buttons.
 * #included into app_process.c just before app_process_action(); its prototypes are at the top of app_process.c.
 *
 * Hardware: wired to pins the Multi_Mode board already has (the receiver's servo pins); see docs/TX_PANEL.md
 *   display   0.42" OLED 72 x 40, SSD1315 (SSD1306 commands), I2C address 0x3C (0x3D tried too)
 *             SDA PA08 (servo 6), SCL PA09 (servo 7); pull-ups to 3.3 V on the display module or added (needed: see below)
 *   buttons   Up PC00 (PROTOCOL_00), Down PC01 (PROTOCOL_01), Enter/Bind PB00 (BIND), each to GND
 *   LED       red PA00 (servo 5), green PB04 (servo 2), common cathode; TIMER4 CC0 / CC1 PWM
 *
 * Detection: at power-up PA08 / PA09 are read with the chip's pull-DOWNs. Only when both read high (the display's
 * pull-ups) is the I2C bus tried, so a board with servos or nothing on those pins sees no I2C traffic at all.
 * Display found = TX panel: transmitter only (a stored receiver setup, F/S and the bind plug are ignored), the
 * protocol jumpers are buttons, nothing stored = "cfg tx rlink" stored. No display = the module as before.
 *
 * Input (menu Input, stored with the mode in the setup record's outputs byte):
 *   Auto      MULTI frames (EdgeTX, the STM32 encoders) on J3 RX, or PPM on PB01 or J3 RX, whichever comes first
 *   PPM only  PPM on PB01 alone; J3 RX is never listened to for MULTI (the log line and cfg keep it)
 * With MULTI frames the radio chooses the mode; the panel shows it (Mode / Opt read-only), Bind and Range work.
 *
 * Display traffic never blocks the radio: the frame is built in RAM and sent PANEL_SLICE bytes per main-loop pass
 * (bit-banged I2C at about PANEL_I2C_HZ: 12 bytes = about 0.3 ms), and only when the picture has changed.
 *
 * Buttons: debounced; a press counts on release (short) or after PANEL_HOLD_MS held (long, once).
 *   Home:  Enter = menu.
 *   Menu (3 lines, 7 x 14 font; the home page 4 lines, 6 x 10):  Up / Down = move, Enter = choose. Mode, Opt, Input, Bind, Range, Info, Back. 15 s without a key = home.
 *   Input: Up / Down = Auto / PPM only, Enter short = cancel, Enter held 2 s = store and restart.
 *   Mode / Opt: Up / Down = change, Enter short = cancel, Enter held 2 s = store and restart.
 *   Bind: Enter held 2 s = restart with BIND for 8 s (as "cfg bind"). Range: on / off at once, no restart.
 *   Enter held at power-up = BIND held, as the BIND jumper (until it is released; then it is a button).
 *
 * LED (red / green, yellow = both):
 *   green steady  linked (RLINK telemetry, FrSky link) or sending (modes without telemetry)
 *   yellow steady weak link            red blinking  link lost / receiver battery low / mode not supported
 *   red/green alternating fast = bind  yellow slow blink = range check   green short blink every 1 s = no input
 *   yellow short flash = power-up
 */
#include "em_gpio.h"
#include "em_timer.h"
#define PANEL_VERSION_TEXT (PANEL_PPM_ONLY ? "v0.3 PPM" : PANEL_MULTI_ONLY ? "v0.3 MULTI" : "Panel v0.3")
#include "panel_font.h"     /* 6 x 10: the home page, 4 lines of 12 */
#include "panel_font14.h"   /* 7 x 14: the menu pages, 3 lines of 10 */

#ifndef PANEL_ROT180
#define PANEL_ROT180     0         /* 1 = picture turned 180 degrees (display mounted the other way up)        */
#endif
#define PANEL_ADDR       0x3Cu     /* 7-bit I2C address; 0x3D is tried when this one does not answer             */
#define PANEL_I2C_HZ     400000u   /* bit-banged clock, about                                                     */
#define PANEL_SLICE      12u       /* bytes per main-loop pass                                                    */
#define PANEL_W          72u
#define PANEL_H          40u
#define PANEL_PAGES      5u
#define PANEL_XOFS       28u       /* the 72 columns sit at columns 28..99 of the controller's 128               */
#define PANEL_CONTRAST   0x7Fu
#define PANEL_DEBOUNCE_MS  30u
#define PANEL_HOLD_MS    2000u
#define PANEL_MENU_IDLE_MS 15000u
#define PANEL_REFRESH_MS  200u     /* the home page is rebuilt this often (telemetry), sent only if it changed   */
#define PANEL_LED_HZ     1000u     /* LED PWM                                                                    */
#define PANEL_LED_RED    600u      /* full brightness of each colour, per mille: even out red and green          */
#define PANEL_LED_GREEN  1000u

static bool panel_boot_bind;       /* Enter (BIND) held at power-up and not yet released: BIND held              */
static bool panel_range;           /* range check from the menu                                                  */
static uint8_t panel_addr = PANEL_ADDR;
static uint32_t panel_half;        /* CPU cycles per half clock                                                  */

/* ------------------------------------------------------------------ bit-banged I2C on PA08 (SDA) / PA09 (SCL) */
#define P_SDA_PORT gpioPortA
static uint8_t P_SDA_PIN = 8;   /* PA08; swapped to PA09 when the display answers only that way (SDA / SCK crossed) */
#define P_SCL_PORT gpioPortA
static uint8_t P_SCL_PIN = 9;

static inline void p_wait(void) { uint32_t t = DWT->CYCCNT; while ((uint32_t)(DWT->CYCCNT - t) < panel_half) { } }
static uint32_t panel_cpu_mhz;     /* measured: CPU cycles per microsecond */
static inline void p_hold(void) { uint32_t t = DWT->CYCCNT; while ((uint32_t)(DWT->CYCCNT - t) < panel_cpu_mhz) { } }   /* 1 us: data held after SCK falls */
static inline void p_sda(bool h) { if (h) GPIO_PinOutSet(P_SDA_PORT, P_SDA_PIN); else GPIO_PinOutClear(P_SDA_PORT, P_SDA_PIN); }
static inline void p_scl(bool h) { if (h) GPIO_PinOutSet(P_SCL_PORT, P_SCL_PIN); else GPIO_PinOutClear(P_SCL_PORT, P_SCL_PIN); }
static void p_start(void) { p_sda(true); p_scl(true); p_wait(); p_sda(false); p_wait(); p_scl(false); p_wait(); }
static void p_stop(void)  { p_sda(false); p_wait(); p_scl(true); p_wait(); p_sda(true); p_wait(); }
static bool p_byte(uint8_t b)                       /* true = ACK */
{
  for (uint8_t m = 0x80u; m; m >>= 1) { p_sda((b & m) != 0u); p_wait(); p_scl(true); p_wait(); p_scl(false); p_hold(); }
  p_sda(true); p_wait(); p_scl(true); p_wait();
  bool ack = GPIO_PinInGet(P_SDA_PORT, P_SDA_PIN) == 0u;
  p_scl(false); p_wait();
  return ack;
}
static bool p_cmds(const uint8_t *c, uint8_t n)     /* one command stream (control byte 0x00), blocking: boot only */
{
  p_start();
  bool ok = p_byte((uint8_t)(panel_addr << 1)) && p_byte(0x00u);
  for (uint8_t i = 0; ok && i < n; i++) ok = p_byte(c[i]);
  p_stop();
  return ok;
}

/* ------------------------------------------------------------------ frame buffer, text */
static uint8_t pf_fb[PANEL_PAGES][PANEL_W];         /* page p, column x: bit (y & 7) of row y = 8 p + (y & 7)      */
static uint8_t pf_sent[PANEL_PAGES][PANEL_W];       /* what the display shows                                    */
static bool    pf_valid;                            /* pf_sent is known (after the first full frame)             */

static void pf_clear(void) { memset(pf_fb, 0, sizeof pf_fb); }
static void pf_pixel(uint8_t x, uint8_t y) { if (x < PANEL_W && y < PANEL_H) pf_fb[y >> 3][x] |= (uint8_t)(1u << (y & 7u)); }
static void pf_text(uint8_t line, uint8_t col, const char *s)   /* line 0..3 (10 rows each), col 0..11 (6 columns each) */
{
  uint8_t y0 = (uint8_t)(line * 10u);
  for (uint8_t x0 = (uint8_t)(col * 6u); *s && x0 + 6u <= PANEL_W; s++, x0 = (uint8_t)(x0 + 6u)) {
    const uint8_t *g = pf_glyph((uint8_t)*s);
    for (uint8_t r = 0; r < 10u; r++) for (uint8_t c = 0; c < 6u; c++) if (g[r] & (0x20u >> c)) pf_pixel((uint8_t)(x0 + c), (uint8_t)(y0 + r));
  }
}
static void pf_text14(uint8_t line, const char *s)  /* menu pages: line 0..2 (13 rows each), 10 characters from x = 1 */
{
  uint8_t y0 = (uint8_t)(line * 13u);
  for (uint8_t x0 = 1u; *s && x0 + 7u <= PANEL_W; s++, x0 = (uint8_t)(x0 + 7u)) {
    const uint8_t *g = pf14_glyph((uint8_t)*s);
    for (uint8_t r = 0; r < 13u; r++) for (uint8_t c = 0; c < 7u; c++) if (g[r] & (0x40u >> c)) pf_pixel((uint8_t)(x0 + c), (uint8_t)(y0 + r));
  }
}
static void pf_invert14(uint8_t line)               /* the cursor / the value being changed: light on dark      */
{
  for (uint8_t y = (uint8_t)(line * 13u); y < (uint8_t)(line * 13u + 13u) && y < PANEL_H; y++)
    for (uint8_t x = 0; x < PANEL_W; x++) pf_fb[y >> 3][x] ^= (uint8_t)(1u << (y & 7u));
}

/* ------------------------------------------------------------------ sending: one transaction, a slice per pass */
static uint8_t  ps_state;                           /* 0 idle, 1 window, 2 data                                  */
static uint16_t ps_i;

static void ps_service(void)
{
  if (ps_state == 0u) {
    if (pf_valid && memcmp(pf_fb, pf_sent, sizeof pf_fb) == 0) return;
    memcpy(pf_sent, pf_fb, sizeof pf_fb);           /* the frame to send; a change meanwhile goes in the next one */
    ps_state = 1u;
  }
  if (ps_state == 1u) {                             /* the window: columns 28..99, pages 0..4, then data       */
    static const uint8_t win[] = { 0x21u, PANEL_XOFS, PANEL_XOFS + PANEL_W - 1u, 0x22u, 0x00u, PANEL_PAGES - 1u };
    (void)p_cmds(win, sizeof win);
    p_start(); (void)p_byte((uint8_t)(panel_addr << 1)); (void)p_byte(0x40u);   /* data stream: stays open over passes */
    ps_i = 0; ps_state = 2u;
    return;
  }
  const uint8_t *d = &pf_sent[0][0];
  for (uint8_t k = 0; k < PANEL_SLICE && ps_i < sizeof pf_sent; k++) (void)p_byte(d[ps_i++]);
  if (ps_i >= sizeof pf_sent) { p_stop(); ps_state = 0u; pf_valid = true; }
}

/* ------------------------------------------------------------------ LED: TIMER4 CC0 -> PA00 red, CC1 -> PB04 green */
static uint32_t pl_top;
static void pl_init(void)
{
  CMU_ClockEnable(cmuClock_TIMER4, true);
  TIMER_Init_TypeDef ti = TIMER_INIT_DEFAULT; ti.enable = false; ti.prescale = timerPrescale16;
  TIMER_Init(TIMER4, &ti);
  TIMER_InitCC_TypeDef cc = TIMER_INITCC_DEFAULT; cc.mode = timerCCModePWM;
  TIMER_InitCC(TIMER4, 0, &cc);
  TIMER_InitCC(TIMER4, 1, &cc);
  pl_top = CMU_ClockFreqGet(cmuClock_TIMER4) / 16u / PANEL_LED_HZ;
  if (pl_top > 0xFFFFu) pl_top = 0xFFFFu;
  TIMER_TopSet(TIMER4, pl_top);
  TIMER_CompareSet(TIMER4, 0, 0); TIMER_CompareSet(TIMER4, 1, 0);
  GPIO_PinModeSet(gpioPortA, 0, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortB, 4, gpioModePushPull, 0);
  GPIO->TIMERROUTE[4].CC0ROUTE = ((uint32_t)gpioPortA << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (0u << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  GPIO->TIMERROUTE[4].CC1ROUTE = ((uint32_t)gpioPortB << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT) | (4u << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
  GPIO->TIMERROUTE[4].ROUTEEN = GPIO_TIMER_ROUTEEN_CC0PEN | GPIO_TIMER_ROUTEEN_CC1PEN;
  TIMER_Enable(TIMER4, true);
}
static void pl_set(bool red, bool green)
{
  TIMER_CompareBufSet(TIMER4, 0, red   ? pl_top * PANEL_LED_RED   / 1000u : 0u);
  TIMER_CompareBufSet(TIMER4, 1, green ? pl_top * PANEL_LED_GREEN / 1000u : 0u);
}

/* ------------------------------------------------------------------ buttons */
#define PK_UP    0
#define PK_DOWN  1
#define PK_ENTER 2
#define PK_BACK  3       /* 4-button panel only */
#define PK_SHORT 1u
#define PK_LONG  2u
static const sl_gpio_t pk_pin[4] = { { .port = SL_GPIO_PORT_C, .pin = 0 }, { .port = SL_GPIO_PORT_C, .pin = 1 }, { .port = SL_GPIO_PORT_B, .pin = 0 },
                                     { .port = SL_GPIO_PORT_C, .pin = 2 } };   /* Up, Down, Enter (BIND), Back (PC02) */
static bool     pk_down[4], pk_raw[4], pk_long_sent[4];
static uint32_t pk_change[4], pk_since[4];

static uint8_t pk_poll(uint8_t k, uint32_t now)     /* 0, PK_SHORT (on release) or PK_LONG (once, while held)  */
{
  bool v = false;
  sl_gpio_get_pin_input(&pk_pin[k], &v);
  bool raw = !v;
  if (raw != pk_raw[k]) { pk_raw[k] = raw; pk_change[k] = now; return 0; }
  if ((uint32_t)(now - pk_change[k]) < PANEL_DEBOUNCE_MS * 1000u || raw == pk_down[k]) {
    if (pk_down[k] && !pk_long_sent[k] && (uint32_t)(now - pk_since[k]) >= PANEL_HOLD_MS * 1000u) { pk_long_sent[k] = true; return PK_LONG; }
    return 0;
  }
  pk_down[k] = raw;
  if (raw) { pk_since[k] = now; pk_long_sent[k] = false; return 0; }
  return pk_long_sent[k] ? 0u : PK_SHORT;
}

/* ------------------------------------------------------------------ modes (as the Controller's TX list) */
typedef struct { uint8_t proto, sub; const char *label; bool opt; } pm_t;
static const pm_t pm_list[] = {
  {  0, 0, "nRF24",     true  }, {  0, 1, "RLINK",     true  },
  {  0, 2, "UK-459",    false }, {  3, 0, "FrSky D8",  true  }, { 15, 0, "D16v1 EU",  true  },
  { 15, 1, "D16v2 EU",  true  }, { 15, 2, "D16v1 FCC", true  }, { 15, 3, "D16v2 FCC", true  },
  { 15, 5, "D16 FCC8",  true  }, {  6, 2, "DSMX",      true  },
  {  0, 6, "Jeti cap",  false }, {  0, 7, "TX test",   false },
};
#define PM_N ((uint8_t)(sizeof pm_list / sizeof pm_list[0]))
static int8_t pm_find(uint8_t p, uint8_t s)
{
  for (uint8_t i = 0; i < PM_N; i++) if (pm_list[i].proto == p && pm_list[i].sub == s) return (int8_t)i;
  if (p == 3u) for (uint8_t i = 0; i < PM_N; i++) if (pm_list[i].proto == p) return (int8_t)i;   /* any Sub */
  return -1;
}
static void panel_running(uint8_t *p, uint8_t *s, uint8_t *o)   /* what runs now: the radio's choice or the stored setup */
{
  if (bay_mode) { *p = bay_boot.proto; *s = bay_boot.sub; *o = bay_boot.option; }
  else          { *p = cfg_boot.proto; *s = cfg_boot.sub; *o = cfg_boot.option; }
}

/* ------------------------------------------------------------------ restart with a new setup / with BIND */
static void panel_reset(void)
{
  sl_rail_handle_t rh = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);
  uint32_t t0 = (uint32_t)sl_rail_get_time(rh);
  while (((uint32_t)sl_rail_get_time(rh) - t0) < 30000u) { }
  NVIC_SystemReset();
}
static bool panel_store(uint8_t p, uint8_t s, uint8_t o, uint8_t in)   /* in: CFG_TXIN_AUTO / CFG_TXIN_PPM */
{
  cfg_t c; memset(&c, 0, sizeof c);
  c.role = CFG_ROLE_TX; c.proto = p; c.sub = s; c.option = o; c.outputs = in;
  return cfg_save(&c);
}
static uint8_t panel_in(void) { return (PANEL_PPM_ONLY || (PANEL_INPUT == PANEL_IN_MULTI_PPM && cfg_boot.outputs == CFG_TXIN_PPM)) ? CFG_TXIN_PPM : CFG_TXIN_AUTO; }   /* the stored input
 */
static bool panel_bind_request(void)
{
  if (cfg_bind_request()) return true;
  cfg_t c;                                          /* the word was used before: rewriting the record erases it */
  if (cfg_load(&c) && cfg_save(&c)) return cfg_bind_request();
  return false;
}

/* ------------------------------------------------------------------ pages */
enum { PG_HOME, PG_MENU, PG_MODE, PG_OPT, PG_INPUT, PG_BIND, PG_INFO, PG_MSG };
static uint8_t  pg = PG_HOME, pg_cursor, pg_mode_i, pg_opt, pg_in;
static uint32_t pg_last_key, pg_msg_until, pg_built;
static const char *pg_msg1, *pg_msg2;
static bool     pg_dirty = true;
enum { M_MODE, M_OPT, M_INPUT, M_BIND, M_RANGE, M_INFO, M_BACK };
static const char *const pg_menu[] = { "Mode", "Opt", "Input", "Bind", "Range", "Info", "Back" };   /* by M_* */
static const uint8_t pg_items[] = { M_MODE, M_OPT,
#if PANEL_INPUT == PANEL_IN_MULTI_PPM
                                    M_INPUT,                 /* only the MULTI + PPM build has the Input choice */
#endif
                                    M_BIND, M_RANGE, M_INFO,
#if PANEL_BUTTONS == 3
                                    M_BACK                   /* 4 buttons: the Back key instead */
#endif
                                  };
#define PG_MENU_N ((uint8_t)sizeof pg_items)

static bool panel_binding(uint32_t now) { return cfg_bind_until != 0u && (int32_t)(now - cfg_bind_until) < 0; }

static void pg_message(const char *a, const char *b, uint32_t now) { pg_msg1 = a; pg_msg2 = b; pg_msg_until = now + 1500000u; pg = PG_MSG; pg_dirty = true; }

static bool panel_input(uint32_t now) { return (int32_t)(now - ppm_last_sync) <= 100000; }

static void pg_build_home(uint32_t now)
{
  sl_rail_handle_t rh = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);
  bay_telem_t t; bay_telem_fill(rh, now, &t);
  uint8_t p, s, o; panel_running(&p, &s, &o);
  int8_t i = pm_find(p, s);
  char b[20];
  /* line 0: the mode (+UK when the 459 MHz link runs beside it) */
  if (bay_unsupported) snprintf(b, sizeof b, "Prot %u/%u ?", (unsigned)p, (unsigned)s);
  else snprintf(b, sizeof b, "%s%s", i >= 0 ? pm_list[i].label : "Prot ?", uk_link_on ? "+UK" : "");
  pf_text(0, 0, b);
  /* line 1: the input, and Single / Dual */
  const char *in = !panel_input(now) ? "No input" : bay_mode ? "MULTI" : "PPM";
  const char *op = (i >= 0 && pm_list[i].opt) ? (o ? "Dual" : "Sngl") : "";
  snprintf(b, sizeof b, "%-8s%4s", in, op);
  pf_text(1, 0, b);
  /* line 2: the link */
  if (t.lq != BAY_NO_LQ && t.rssi_rx != BAY_NO_RSSI) snprintf(b, sizeof b, "LQ%3u %4ddB", (unsigned)t.lq, (int)t.rssi_rx);
  else if (t.rssi_rx != BAY_NO_RSSI) snprintf(b, sizeof b, "RSSI %d", (int)t.rssi_rx);
  else snprintf(b, sizeof b, "Pwr %ddBm", (int)t.power_dbm);
  pf_text(2, 0, b);
  /* line 3: what needs attention first, else the receiver battery */
  if (t.binding || panel_binding(now)) snprintf(b, sizeof b, "BIND");
  else if (range_in())                   snprintf(b, sizeof b, "RANGE CHECK");
  else if (t.link_alarm == 2u && t.bound) snprintf(b, sizeof b, "LINK LOST");
  else if (t.vbat_alarm)                 snprintf(b, sizeof b, "RX BATT %u.%02uV", (unsigned)(t.vbat_10mv / 100u), (unsigned)(t.vbat_10mv % 100u));
  else if (t.vbat_sensor)                snprintf(b, sizeof b, "Rx %u.%02uV", (unsigned)(t.vbat_10mv / 100u), (unsigned)(t.vbat_10mv % 100u));
  else b[0] = 0;
  pf_text(3, 0, b);
}

static void pg_build(uint32_t now)                 /* home: 4 lines, 6 x 10; menu pages: 3 lines, 7 x 14 */
{
  char b[20];
  pf_clear();
  switch (pg) {
    case PG_HOME: pg_build_home(now); break;
    case PG_MENU: {
      uint8_t first = pg_cursor > 2u ? (uint8_t)(pg_cursor - 2u) : 0u;
      for (uint8_t l = 0; l < 3u && first + l < PG_MENU_N; l++) {
        uint8_t k = pg_items[first + l];
        if (k == M_RANGE) snprintf(b, sizeof b, "Range %s", range_in() ? "on" : "off");
        else if (k == M_INPUT) snprintf(b, sizeof b, "Input %s", panel_in() == CFG_TXIN_PPM ? "PPM" : "Auto");
        else snprintf(b, sizeof b, "%s", pg_menu[k]);
        pf_text14(l, b);
        if (first + l == pg_cursor) pf_invert14(l);
      }
      break;
    }
    case PG_MODE:
      pf_text14(0, "Mode"); pf_text14(1, pm_list[pg_mode_i].label); pf_invert14(1); pf_text14(2, "Hold=save");
      break;
    case PG_OPT:
      pf_text14(0, "Opt"); pf_text14(1, pg_opt ? "Dual" : "Single"); pf_invert14(1); pf_text14(2, "Hold=save");
      break;
    case PG_INPUT:
      pf_text14(0, "Input"); pf_text14(1, pg_in == CFG_TXIN_PPM ? "PPM only" : "Auto"); pf_invert14(1); pf_text14(2, "Hold=save");
      break;
    case PG_BIND:
      pf_text14(0, "Bind 8 s"); pf_text14(1, "restarts"); pf_text14(2, "Hold=bind");
      break;
    case PG_INFO: {
      uint8_t p, s, o; panel_running(&p, &s, &o);
      pf_text14(0, PANEL_VERSION_TEXT);
      snprintf(b, sizeof b, "Prot %u/%u", (unsigned)p, (unsigned)s); pf_text14(1, b);
      pf_text14(2, uk_have_sx ? "SX127x yes" : "SX127x no");
      break;
    }
    case PG_MSG:
      pf_text14(0, pg_msg1 ? pg_msg1 : ""); pf_text14(1, pg_msg2 ? pg_msg2 : "");
      break;
  }
}

static void pg_key(uint8_t k, uint8_t ev, uint32_t now)
{
  pg_last_key = now; pg_dirty = true;
  uint8_t p, s, o; panel_running(&p, &s, &o);
  if (k == PK_BACK) {                               /* 4 buttons: Back = one level up, never a store */
    if (ev == PK_SHORT) pg = (pg == PG_MENU || pg == PG_HOME) ? PG_HOME : PG_MENU;
    return;
  }
  switch (pg) {
    case PG_HOME:
      if (k == PK_ENTER && ev == PK_SHORT) { pg = PG_MENU; pg_cursor = 0; }
      break;
    case PG_MENU:
      if (ev != PK_SHORT) break;
      if (k == PK_UP)   { pg_cursor = pg_cursor ? (uint8_t)(pg_cursor - 1u) : (uint8_t)(PG_MENU_N - 1u); break; }
      if (k == PK_DOWN) { pg_cursor = (uint8_t)((pg_cursor + 1u) % PG_MENU_N); break; }
      switch (pg_items[pg_cursor]) {
        case M_MODE: if (bay_mode || PANEL_MULTI_ONLY) pg_message("Set by", "the radio", now);
                else { int8_t i = pm_find(p, s); pg_mode_i = i >= 0 ? (uint8_t)i : 1u; pg = PG_MODE; }
                break;
        case M_OPT: { int8_t i = pm_find(p, s);
                  if (bay_mode || PANEL_MULTI_ONLY) pg_message("Set by", "the radio", now);
                  else if (i < 0 || !pm_list[i].opt) pg_message("No Opt for", "this mode", now);
                  else { pg_opt = o ? 1u : 0u; pg = PG_OPT; } }
                break;
        case M_INPUT: pg_in = panel_in(); pg = PG_INPUT; break;
        case M_BIND: pg = PG_BIND; break;
        case M_RANGE: panel_range = !panel_range;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
                app_log("PANEL range check %s\r\n", panel_range ? "on" : "off");
#endif
                break;
        case M_INFO: pg = PG_INFO; break;
        default: pg = PG_HOME; break;
      }
      break;
    case PG_MODE:
      if (k == PK_UP   && ev == PK_SHORT) pg_mode_i = pg_mode_i ? (uint8_t)(pg_mode_i - 1u) : (uint8_t)(PM_N - 1u);
      if (k == PK_DOWN && ev == PK_SHORT) pg_mode_i = (uint8_t)((pg_mode_i + 1u) % PM_N);
      if (k == PK_ENTER && ev == PK_SHORT) pg = PG_MENU;
      if (k == PK_ENTER && ev == PK_LONG) {
        const pm_t *m = &pm_list[pg_mode_i];
        bool ok = panel_store(m->proto, m->sub, m->opt ? o : 0u, panel_in());
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("PANEL mode %s stored%s, restarting\r\n", m->label, ok ? "" : " - FAILED");
#endif
        if (ok) { pf_clear(); pf_text14(0, "Stored,"); pf_text14(1, "restarting"); while (ps_state || memcmp(pf_fb, pf_sent, sizeof pf_fb)) ps_service(); panel_reset(); }
        else pg_message("Store", "FAILED", now);
      }
      break;
    case PG_OPT:
      if ((k == PK_UP || k == PK_DOWN) && ev == PK_SHORT) pg_opt ^= 1u;
      if (k == PK_ENTER && ev == PK_SHORT) pg = PG_MENU;
      if (k == PK_ENTER && ev == PK_LONG) {
        bool ok = panel_store(p, s, pg_opt, panel_in());
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("PANEL Opt %s stored%s, restarting\r\n", pg_opt ? "dual" : "single", ok ? "" : " - FAILED");
#endif
        if (ok) { pf_clear(); pf_text14(0, "Stored,"); pf_text14(1, "restarting"); while (ps_state || memcmp(pf_fb, pf_sent, sizeof pf_fb)) ps_service(); panel_reset(); }
        else pg_message("Store", "FAILED", now);
      }
      break;
    case PG_INPUT:
      if ((k == PK_UP || k == PK_DOWN) && ev == PK_SHORT) pg_in = (pg_in == CFG_TXIN_PPM) ? CFG_TXIN_AUTO : CFG_TXIN_PPM;
      if (k == PK_ENTER && ev == PK_SHORT) pg = PG_MENU;
      if (k == PK_ENTER && ev == PK_LONG) {
        bool ok = panel_store(cfg_boot.proto, cfg_boot.sub, cfg_boot.option, pg_in);   /* the stored mode, whatever the radio runs */
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("PANEL input %s stored%s, restarting\r\n", pg_in == CFG_TXIN_PPM ? "PPM only (PB01)" : "auto (MULTI or PPM)", ok ? "" : " - FAILED");
#endif
        if (ok) { pf_clear(); pf_text14(0, "Stored,"); pf_text14(1, "restarting"); while (ps_state || memcmp(pf_fb, pf_sent, sizeof pf_fb)) ps_service(); panel_reset(); }
        else pg_message("Store", "FAILED", now);
      }
      break;
    case PG_BIND:
      if (k == PK_ENTER && ev == PK_SHORT) pg = PG_MENU;
      if (k == PK_ENTER && ev == PK_LONG) {
        bool ok = panel_bind_request();
#if defined(SL_CATALOG_APP_LOG_PRESENT)
        app_log("PANEL bind %s, restarting\r\n", ok ? "asked for" : "request FAILED");
#endif
        if (ok) { pf_clear(); pf_text14(0, "Binding,"); pf_text14(1, "restarting"); while (ps_state || memcmp(pf_fb, pf_sent, sizeof pf_fb)) ps_service(); panel_reset(); }
        else pg_message("Bind", "FAILED", now);
      }
      break;
    case PG_INFO:
      if (k == PK_ENTER && ev == PK_SHORT) pg = PG_MENU;
      break;
    default: break;
  }
}

/* ------------------------------------------------------------------ LED pattern */
static uint32_t pl_boot;
static void pl_service(uint32_t now)
{
  uint32_t ms = now / 1000u;
  if ((uint32_t)(now - pl_boot) < 300000u) { pl_set(true, true); return; }          /* power-up: yellow flash */
  if (bay_unsupported) { pl_set((ms / 125u) & 1u, false); return; }                 /* mode not supported: red fast */
  if (panel_binding(now) || panel_boot_bind || (bay_mode && bay_bind_now)) { bool ph = (ms / 125u) & 1u; pl_set(ph, !ph); return; }
  if (range_in()) { bool on = (ms % 1000u) < 500u; pl_set(on, on); return; }        /* range check: yellow slow */
  if (!panel_input(now)) { pl_set(false, (ms % 1000u) < 80u); return; }             /* no input: green short blink */
  sl_rail_handle_t rh = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);
  bay_telem_t t; bay_telem_fill(rh, now, &t);
  if (t.link_alarm == 2u || t.vbat_alarm) { pl_set((ms % 500u) < 250u, false); return; }   /* lost / RX battery */
  if (t.link_alarm == 1u) { pl_set(true, true); return; }                            /* weak: yellow */
  pl_set(false, true);                                                               /* linked / sending */
}

/* ------------------------------------------------------------------ the interface to app_process.c */
static bool panel_start(sl_rail_handle_t rh)
{
  /* pull-ups present? read both pins with the chip's pull-downs */
  GPIO_PinModeSet(P_SDA_PORT, P_SDA_PIN, gpioModeInputPull, 0);
  GPIO_PinModeSet(P_SCL_PORT, P_SCL_PIN, gpioModeInputPull, 0);
  uint32_t t0 = (uint32_t)sl_rail_get_time(rh);
  while (((uint32_t)sl_rail_get_time(rh) - t0) < 200u) { }
  bool pulled = GPIO_PinInGet(P_SDA_PORT, P_SDA_PIN) && GPIO_PinInGet(P_SCL_PORT, P_SCL_PIN);
  if (!pulled) {
    GPIO_PinModeSet(P_SDA_PORT, P_SDA_PIN, gpioModeDisabled, 0);
    GPIO_PinModeSet(P_SCL_PORT, P_SCL_PIN, gpioModeDisabled, 0);
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    app_log("PANEL no display (PA08 / PA09 not pulled up): the module as before\r\n");
#endif
    return false;
  }
  DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;               /* cycle counter for the I2C timing */
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  { uint32_t r0 = (uint32_t)sl_rail_get_time(rh), c0 = DWT->CYCCNT;   /* the CPU clock, measured against the radio's 1 us clock */
    while (((uint32_t)sl_rail_get_time(rh) - r0) < 1000u) { }
    panel_cpu_mhz = (DWT->CYCCNT - c0 + 500u) / 1000u;
    if (panel_cpu_mhz < 1u) panel_cpu_mhz = 1u; }
  panel_half = panel_cpu_mhz * 1000000u / (2u * PANEL_I2C_HZ);
  if (panel_half < 8u) panel_half = 8u;
  GPIO_PinModeSet(P_SDA_PORT, P_SDA_PIN, gpioModeWiredAndPullUp, 1);
  GPIO_PinModeSet(P_SCL_PORT, P_SCL_PIN, gpioModeWiredAndPullUp, 1);
  p_wait();
  for (uint8_t i = 0; i < 9u; i++) { p_scl(false); p_wait(); p_scl(true); p_wait(); }   /* free a bus left mid-byte */
  p_stop();
  /* Look for the display for up to 300 ms (its own reset after power-up), at 100 kHz, both ways round
     (SDA / SCK crossed), at 0x3C and 0x3D. */
  uint32_t fast = panel_half;
  panel_half = panel_cpu_mhz * 1000000u / (2u * 100000u);
  bool ack = false, swapped = false;
  t0 = (uint32_t)sl_rail_get_time(rh);
  while (!ack && ((uint32_t)sl_rail_get_time(rh) - t0) < 300000u) {
    for (uint8_t w = 0; w < 2u && !ack; w++) {
      P_SDA_PIN = w ? 9u : 8u; P_SCL_PIN = w ? 8u : 9u;
      GPIO_PinModeSet(P_SDA_PORT, P_SDA_PIN, gpioModeWiredAndPullUp, 1);
      GPIO_PinModeSet(P_SCL_PORT, P_SCL_PIN, gpioModeWiredAndPullUp, 1);
      p_wait();
      for (uint8_t i = 0; i < 9u; i++) { p_scl(false); p_wait(); p_scl(true); p_wait(); }
      p_stop();
      for (uint8_t a = 0; a < 2u && !ack; a++) {
        panel_addr = (uint8_t)(PANEL_ADDR + a);
        p_start(); ack = p_byte((uint8_t)(panel_addr << 1)); p_stop();
        swapped = ack && w;
      }
    }
  }
  if (!ack) {
#if defined(SL_CATALOG_APP_LOG_PRESENT)
    { /* what is on the bus at all: every address, PA08 = SDA; and can the pins pull the lines low? */
      P_SDA_PIN = 8u; P_SCL_PIN = 9u;
      GPIO_PinModeSet(P_SDA_PORT, P_SDA_PIN, gpioModeWiredAndPullUp, 1);
      GPIO_PinModeSet(P_SCL_PORT, P_SCL_PIN, gpioModeWiredAndPullUp, 1);
      p_sda(false); p_wait(); uint32_t sda_lo = GPIO_PinInGet(P_SDA_PORT, P_SDA_PIN); p_sda(true); p_wait();
      p_scl(false); p_wait(); uint32_t scl_lo = GPIO_PinInGet(P_SCL_PORT, P_SCL_PIN); p_scl(true); p_wait();
      char found[40]; size_t n = 0; found[0] = 0;
      for (uint8_t a = 0x08u; a < 0x78u; a++) {
        p_start(); bool k = p_byte((uint8_t)(a << 1)); p_stop();
        if (k && n < sizeof found - 6u) n += (size_t)snprintf(found + n, sizeof found - n, " %02X", a);
      }
      app_log("PANEL PA08 / PA09 pulled up but no display answered at 0x3C / 0x3D (tried 300 ms, both ways round): "
              "the module as before. Lines driven low read SDA %lu SCK %lu (0 = good); addresses answering:%s; CPU %lu MHz, I2C 100 kHz\r\n",
              (unsigned long)sda_lo, (unsigned long)scl_lo, n ? found : " none", (unsigned long)panel_cpu_mhz);
    }
#endif
    GPIO_PinModeSet(P_SDA_PORT, P_SDA_PIN, gpioModeDisabled, 0);
    GPIO_PinModeSet(P_SCL_PORT, P_SCL_PIN, gpioModeDisabled, 0);
    return false;
  }
  panel_half = fast;
  static const uint8_t init[] = {
    0xAEu,                       /* display off                                         */
    0xD5u, 0x80u,                /* clock                                               */
    0xA8u, PANEL_H - 1u,         /* multiplex 40                                        */
    0xD3u, 0x00u,                /* display offset                                      */
    0x40u,                       /* start line 0                                        */
    0x8Du, 0x14u,                /* charge pump on                                      */
    0x20u, 0x00u,                /* horizontal addressing                               */
#if PANEL_ROT180
    0xA0u, 0xC0u,                /* segment / COM direction: turned 180 degrees         */
#else
    0xA1u, 0xC8u,                /* segment remap, COM scan down                        */
#endif
    0xDAu, 0x12u,                /* COM pins                                            */
    0x81u, PANEL_CONTRAST,       /* contrast                                            */
    0xD9u, 0xF1u,                /* pre-charge                                          */
    0xDBu, 0x40u,                /* VCOMH                                               */
    0xADu, 0x30u,                /* internal IREF (the 0.42" panels)                    */
    0x2Eu,                       /* no scrolling                                        */
    0xA4u, 0xA6u,                /* show RAM, normal (not inverted)                     */
  };
  bool ok = p_cmds(init, sizeof init);
  { static const uint8_t full[] = { 0x21u, 0x00u, 0x7Fu, 0x22u, 0x00u, 0x07u };     /* clear all of the 128 x 64 RAM */
    ok = ok && p_cmds(full, sizeof full);
    p_start(); (void)p_byte((uint8_t)(panel_addr << 1)); (void)p_byte(0x40u);
    for (uint16_t i = 0; i < 1024u; i++) (void)p_byte(0x00u);
    p_stop(); }
  pf_clear(); pf_text14(0, "MiTo TX"); pf_text14(1, PANEL_VERSION_TEXT);
  while (ps_state || !pf_valid) ps_service();
  { static const uint8_t on[] = { 0xAFu }; ok = ok && p_cmds(on, 1); }
  /* buttons: Up / Down on the protocol jumper pins, Enter = BIND */
  sl_gpio_set_pin_mode(&pk_pin[PK_UP],   SL_GPIO_MODE_INPUT_PULL, 1);
  sl_gpio_set_pin_mode(&pk_pin[PK_DOWN], SL_GPIO_MODE_INPUT_PULL, 1);
#if PANEL_BUTTONS == 4
  sl_gpio_set_pin_mode(&pk_pin[PK_BACK], SL_GPIO_MODE_INPUT_PULL, 1);
#endif
  { bool v = true; sl_gpio_get_pin_input(&pk_pin[PK_ENTER], &v); panel_boot_bind = !v;
    pk_raw[PK_ENTER] = pk_down[PK_ENTER] = !v; pk_long_sent[PK_ENTER] = true; }   /* that press is BIND, not a key */
  pl_init();
  pl_boot = (uint32_t)sl_rail_get_time(rh);
  panel_on = true;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  if (swapped) app_log("PANEL SDA / SCK are crossed (display SDA on PA09): working that way round\r\n");
  app_log("PANEL display at 0x%02X%s: TX panel%s (transmitter only; PC00 / PC01 / BIND = Up / Down / Enter; LED PA00 / PB04)%s\r\n",
          panel_addr, ok ? "" : " (init not acknowledged)", PANEL_PPM_ONLY ? ", PPM-only build" : PANEL_MULTI_ONLY ? ", MULTI-only build" : ", MULTI + PPM build", panel_boot_bind ? "; Enter held: BIND until it is released" : "");
#endif
  return true;
}

static bool panel_bind_pin(void)                    /* BIND from the Enter key: only the press held at power-up */
{
  if (!panel_boot_bind) return false;
  bool v = true; sl_gpio_get_pin_input(&pk_pin[PK_ENTER], &v);
  if (v) panel_boot_bind = false;                   /* released: from now on it is a key */
  return panel_boot_bind;
}
static bool panel_range_on(void) { return panel_on && panel_range; }

static void panel_service(uint32_t now)             /* every main-loop pass (and while waiting for the input) */
{
  if (!panel_on) return;
  (void)panel_bind_pin();
  for (uint8_t k = 0; k < (uint8_t)PANEL_BUTTONS; k++) {
    uint8_t ev = pk_poll(k, now);
    if (k == PK_ENTER && panel_boot_bind) ev = 0;
    if (ev) pg_key(k, ev, now);
  }
  if (pg == PG_MSG && (int32_t)(now - pg_msg_until) >= 0) { pg = PG_MENU; pg_dirty = true; }
  if (pg != PG_HOME && pg != PG_MSG && (uint32_t)(now - pg_last_key) >= PANEL_MENU_IDLE_MS * 1000u) { pg = PG_HOME; pg_dirty = true; }
  if (ps_state == 0u && (pg_dirty || (uint32_t)(now - pg_built) >= PANEL_REFRESH_MS * 1000u)) {
    pg_build(now); pg_built = now; pg_dirty = false;
  }
  ps_service();
  pl_service(now);
}
