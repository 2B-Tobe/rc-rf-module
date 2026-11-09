/* servo_out.c — see servo_out.h.  #included into app_process.c.   v1.2 (output layouts: SBUS / PPM on connectors 6 / 7)
 *
 * The pulse sequence is the one the receiver roles ran in their RAIL-timer callbacks (nrf24_rx.c n_ppm_cb and
 * the others, all alike): PPM pulse n (300 us low on PC03) starts channel n; servo n is high from PPM pulse n to
 * PPM pulse n+1; after channel 8 the sync gap fills the 22.5 ms frame.
 *
 * v1.1: every edge is made by timer hardware. The nine outputs are the compare outputs of TIMER2 CC0 (PPM),
 * TIMER2 CC1-2, TIMER3 CC0-2 and TIMER4 CC0-2 (servos 1..8), routed to the pins; the three timers run free at
 * 1 MHz with the same TOP, so a compare value is "the master time plus this timer's constant offset".
 * The PPM channel's compare interrupt runs AFTER each PPM edge and only arms the next edges: the end of the PPM
 * pulse 300 us ahead, the servo edges 800..2200 us ahead. An interrupt held off for 100 us by RAIL or the SDK
 * (the stretches the transmitter's PPM capture was moved to hardware for) no longer moves an edge.
 * v1.0 made the edges in the interrupt itself, so every late interrupt was a late edge: a quick servo jump.
 *
 * Diagnostics (servo_out_report, every 2 s from the main loop): interrupt latency (late = more than 20 us
 * after the edge, and the worst since the last line) and channel jumps (a channel changing by more than 100 us
 * from one frame to the next, and the worst) - with the sticks still, both counts must stay put.
 */
#include "servo_out.h"
#include "servo_pins.h"
#include "sl_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"
#if defined(SL_CATALOG_APP_LOG_PRESENT)
#include "app_log.h"
#endif

#define SO_CHANNELS   8u
#define SO_FRAME_US   22500u
#define SO_PULSE_US   300u
#define SO_TIMER      TIMER2             /* master timebase and the PPM channel (CC0, the only interrupt) */
#define SO_IRQN       TIMER2_IRQn
#define SO_IRQ_PRIO   0                  /* above RAIL: the handler is a few register writes */
#define SO_MARGIN     10u                /* an edge closer than this (or already passed) is made right away */
#define SO_LATE_US    20u                /* interrupt latency counted as late */
#define SO_JUMP_US    100u               /* frame-to-frame channel change counted as a jump */
#define SO_PPM        8u                 /* output index of the PPM line */

typedef struct { TIMER_TypeDef *t; uint8_t ch; } so_cc_t;
static const so_cc_t so_cc[9] = {                                  /* outputs 0..7 = servos 1..8, 8 = PPM */
  { TIMER2, 1 }, { TIMER2, 2 }, { TIMER3, 0 }, { TIMER3, 1 }, { TIMER3, 2 }, { TIMER4, 0 }, { TIMER4, 1 }, { TIMER4, 2 },
  { TIMER2, 0 } };
static const sl_gpio_t so_servo[8] = SERVO_PINS_INIT;              /* servo_pins.h: dual build map */
static const sl_gpio_t so_ppm_pc03 = { .port = SL_GPIO_PORT_C, .pin = 3 };   /* PC03: PPM out (layout without PPM bit) */
static uint8_t so_layout;                                          /* RXOUT_* bits */
static bool    so_held[SO_CHANNELS];                               /* connector kept an input (servo_out_hold) */

static const volatile uint16_t *so_src, *so_over;                  /* the role's channels, the override's */
static volatile bool so_on, so_running;
static bool     so_inited, so_park_pending;
static uint32_t so_when, so_next, so_mask, so_used, so_park_at;
static uint32_t so_off[9];                                         /* each output's timer counter minus TIMER2's */
static uint8_t  so_idx;
static uint16_t so_prev[SO_CHANNELS];
static volatile uint32_t so_frames, so_late_n, so_late_max, so_jump_n, so_jump_max;

/* the pin of output o (0..7 servos, 8 = PPM) in this layout; NULL = the output has no pin (its connector does SBUS / PPM) */
static const sl_gpio_t *so_pin(uint8_t o)
{
  if (o == SO_PPM) return (so_layout & RXOUT_PPM) ? &so_servo[6] : &so_ppm_pc03;
  if (o == 5u && (so_layout & RXOUT_SBUS)) return NULL;
  if (o == 6u && (so_layout & RXOUT_PPM))  return NULL;
  return &so_servo[o];
}
/* timer compare output o to its pin on / off */
static void so_route(uint8_t o, bool on)
{
  const so_cc_t *k = &so_cc[o];
  const sl_gpio_t *pin = so_pin(o);
  GPIO_TIMERROUTE_TypeDef *r = &GPIO->TIMERROUTE[TIMER_NUM(k->t)];
  uint32_t en = (k->ch == 0u) ? GPIO_TIMER_ROUTEEN_CC0PEN : (k->ch == 1u) ? GPIO_TIMER_ROUTEEN_CC1PEN : GPIO_TIMER_ROUTEEN_CC2PEN;
  if (!on || pin == NULL) { r->ROUTEEN &= ~en; return; }
  uint32_t route = ((uint32_t)pin->port << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                 | ((uint32_t)pin->pin  << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  if (k->ch == 0u)      r->CC0ROUTE = route;
  else if (k->ch == 1u) r->CC1ROUTE = route;
  else                  r->CC2ROUTE = route;
  r->ROUTEEN |= en;
}

void servo_out_layout(uint8_t layout) { if (!so_inited) so_layout = (uint8_t)(layout & RXOUT_MAX); }

const char *servo_out_layout_name(uint8_t layout)
{
  static const char *const n[4] = { "8 PWM", "5 PWM + SBUS", "5 PWM + PPM", "5 PWM + SBUS + PPM" };
  return n[layout & RXOUT_MAX];
}

void servo_out_hold(uint8_t servo, bool hold)
{
  if (servo >= SO_CHANNELS || so_pin(servo) == NULL) return;
  so_held[servo] = hold;
  if (!so_inited) return;                                          /* servo_out_init takes it from here */
  if (hold) { so_route(servo, false); sl_gpio_set_pin_mode(so_pin(servo), SL_GPIO_MODE_INPUT_PULL, 1); }
  else      { sl_gpio_set_pin_mode(so_pin(servo), SL_GPIO_MODE_PUSH_PULL, 0); so_route(servo, true); }
}


/* program output o to go high/low at master time 'at' (hardware compare action on its timer channel) */
static void so_edge(uint8_t o, bool high, uint32_t at)
{
  const so_cc_t *k = &so_cc[o];
  uint32_t ctrl = k->t->CC[k->ch].CTRL & ~_TIMER_CC_CTRL_CMOA_MASK;
  k->t->CC[k->ch].CTRL = ctrl | (high ? TIMER_CC_CTRL_CMOA_SET : TIMER_CC_CTRL_CMOA_CLEAR);
  uint32_t cnt = TIMER_CounterGet(k->t);
  uint32_t oc = (at + so_off[o]) & so_mask;
  uint32_t d = (oc - cnt) & so_mask;                               /* ticks until the edge, modulo the counter width */
  if (d <= SO_MARGIN || d > 60000u) oc = (cnt + SO_MARGIN) & so_mask;   /* passed or too close: right away */
  TIMER_CompareSet(k->t, k->ch, oc);
}

static void so_park(void)                                          /* servos low, PPM high, no more edges */
{
  uint32_t at = TIMER_CounterGet(SO_TIMER) + SO_MARGIN;
  for (uint8_t i = 0; i < SO_CHANNELS; i++) so_edge(i, false, at);
  so_edge(SO_PPM, true, at);
  TIMER_IntDisable(SO_TIMER, TIMER_IEN_CC0);
  so_park_at = at & so_mask; so_park_pending = true;
  so_running = false;
}

static void so_step(void)                                          /* CC0 matched: the PPM edge at so_when was just made */
{
  const volatile uint16_t *src = so_over ? so_over : so_src;
  if (!so_running) return;
  if ((!so_on && so_over == NULL) || src == NULL) { so_park(); return; }
  uint8_t n = (uint8_t)(so_idx >> 1);
  if (!(so_idx & 1u)) {                                            /* PPM pulse n started: arm its end and channel n's end */
    so_edge(SO_PPM, true, so_when + SO_PULSE_US);
    if (n < SO_CHANNELS) {
      uint16_t us = src[n]; if (us < 800u) us = 800u; if (us > 2200u) us = 2200u;
      if (so_frames != 0u) {
        uint16_t d = us > so_prev[n] ? us - so_prev[n] : so_prev[n] - us;
        if (d > so_jump_max) so_jump_max = d;
        if (d > SO_JUMP_US) so_jump_n++;
      }
      so_prev[n] = us;
      so_next = so_when + us; so_used += us;                       /* PPM pulse n+1: servo n ends, servo n+1 starts */
      so_edge(n, false, so_next);
      if (n + 1u < SO_CHANNELS) so_edge(n + 1u, true, so_next);
    } else {                                                       /* sync pulse: the gap, then the next frame */
      uint32_t gap = (so_used + SO_PULSE_US < SO_FRAME_US) ? SO_FRAME_US - so_used - SO_PULSE_US : 4000u;
      so_next = so_when + SO_PULSE_US + gap; so_used = 0; so_frames++;
      so_edge(0, true, so_next);
    }
    so_when += SO_PULSE_US; so_idx++;
  } else {                                                         /* PPM pulse n ended: arm the next PPM pulse */
    so_edge(SO_PPM, false, so_next);
    so_when = so_next;
    so_idx = (n >= SO_CHANNELS) ? 0u : (uint8_t)(so_idx + 1u);
  }
}

void TIMER2_IRQHandler(void)
{
  uint32_t f = TIMER_IntGet(SO_TIMER);
  TIMER_IntClear(SO_TIMER, f);
  if (f & TIMER_IF_CC0) {
    uint32_t lat = (TIMER_CounterGet(SO_TIMER) - SO_TIMER->CC[0].OC) & so_mask;   /* how long after the edge */
    if (lat < 32768u) { if (lat > so_late_max) so_late_max = lat; if (lat > SO_LATE_US) so_late_n++; }
    so_step();
  }
}

static void so_start(void)
{
  if (so_running) return;
  if (so_park_pending) {                                           /* let the park edges happen first */
    while (((so_park_at - TIMER_CounterGet(SO_TIMER)) & so_mask) < 64u) { }
    so_park_pending = false;
  }
  so_running = true; so_idx = 0; so_used = 0;
  so_when = TIMER_CounterGet(SO_TIMER) + 1000u;
  so_edge(SO_PPM, false, so_when);                                 /* PPM pulse 0 = start of channel 0 */
  so_edge(0, true, so_when);
  TIMER_IntClear(SO_TIMER, TIMER_IF_CC0);
  NVIC_ClearPendingIRQ(SO_IRQN);
  TIMER_IntEnable(SO_TIMER, TIMER_IEN_CC0);
}

static void so_timer_init(TIMER_TypeDef *t, CMU_Clock_TypeDef clk)
{
  CMU_ClockEnable(clk, true);
  uint32_t tfreq = CMU_ClockFreqGet(clk);
  TIMER_Init_TypeDef ti = TIMER_INIT_DEFAULT;
  ti.enable   = false;
  ti.prescale = (TIMER_Prescale_TypeDef)(tfreq / 1000000u - 1u);  /* 1 MHz ticks (39 MHz -> /39) */
  TIMER_Init(t, &ti);
}

void servo_out_init(void)
{
  if (so_inited) return;
  so_inited = true;
  so_mask = TIMER_MaxCount(SO_TIMER);                              /* 0xFFFF: 16-bit timers */
  so_timer_init(TIMER2, cmuClock_TIMER2);
  so_timer_init(TIMER3, cmuClock_TIMER3);
  so_timer_init(TIMER4, cmuClock_TIMER4);
  for (uint8_t o = 0; o < 9u; o++) {
    const so_cc_t *k = &so_cc[o];
    const sl_gpio_t *pin = so_pin(o);
    bool ppm = (o == SO_PPM);
    bool held = !ppm && so_held[o];
    if (pin != NULL) sl_gpio_set_pin_mode(pin, held ? SL_GPIO_MODE_INPUT_PULL : SL_GPIO_MODE_PUSH_PULL, (ppm || held) ? 1 : 0);   /* the level until the route is on */
    TIMER_InitCC_TypeDef cc = TIMER_INITCC_DEFAULT;
    cc.mode  = timerCCModeCompare;
    cc.cmoa  = timerOutputActionNone;                              /* set or clear per edge, in so_edge */
    cc.coist = ppm;                                                /* PPM idles high, servos low */
    TIMER_InitCC(k->t, k->ch, &cc);
    so_route(o, !held);                                            /* no pin in this layout, or held: no route */
  }
  TIMER_TopSet(TIMER2, so_mask); TIMER_TopSet(TIMER3, so_mask); TIMER_TopSet(TIMER4, so_mask);   /* wrap alike */
  TIMER_IntClear(SO_TIMER, _TIMER_IF_MASK);
  TIMER_IntDisable(SO_TIMER, _TIMER_IEN_MASK);
  NVIC_ClearPendingIRQ(SO_IRQN);
  NVIC_SetPriority(SO_IRQN, SO_IRQ_PRIO);
  NVIC_EnableIRQ(SO_IRQN);
  TIMER_Enable(TIMER2, true); TIMER_Enable(TIMER3, true); TIMER_Enable(TIMER4, true);
  for (uint8_t o = 0; o < 9u; o++) {                               /* each timer's counter against TIMER2's: constant */
    TIMER_TypeDef *t = so_cc[o].t;
    if (t == SO_TIMER) { so_off[o] = 0; continue; }
    for (;;) {
      uint32_t a = TIMER_CounterGet(SO_TIMER), b = TIMER_CounterGet(t), c = TIMER_CounterGet(SO_TIMER);
      if (a == c) { so_off[o] = (b - a) & so_mask; break; }
    }
  }
  for (uint8_t i = 0; i < SO_CHANNELS; i++) so_prev[i] = 1500u;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("SO servo/PPM edges by TIMER2/3/4 compare outputs, %lu Hz -> 1 MHz, offsets %lu %lu; outputs %s (%s%s)\r\n",
          (unsigned long)CMU_ClockFreqGet(cmuClock_TIMER2), (unsigned long)so_off[2], (unsigned long)so_off[5],
          servo_out_layout_name(so_layout), (so_layout & RXOUT_SBUS) ? "SBUS on connector 6 PA08, " : "SBUS PA03, ",
          (so_layout & RXOUT_PPM) ? "PPM on connector 7 PA09" : "PPM PC03");
#endif
}

void servo_out_source(const volatile uint16_t *us8) { so_src = us8; }

void servo_out_enable(bool on)
{
  so_on = on;
  if (on && so_src != NULL) so_start();                           /* off: the next edge parks the lines */
}

void servo_out_override(const volatile uint16_t *us8)
{
  so_over = us8;
  if (us8 != NULL) so_start();
}

const volatile uint16_t *servo_out_current(void) { return so_over ? so_over : (so_on ? so_src : NULL); }
bool servo_out_overridden(void) { return so_over != NULL; }

void servo_out_report(void)
{
  if (!so_inited) return;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("SO f%lu %s late%lu/%luus jump%lu/%luus\r\n", (unsigned long)so_frames,
          so_running ? (so_over ? "459" : "on") : "off",
          (unsigned long)so_late_n, (unsigned long)so_late_max, (unsigned long)so_jump_n, (unsigned long)so_jump_max);
#endif
  so_late_max = 0; so_jump_max = 0;
}
