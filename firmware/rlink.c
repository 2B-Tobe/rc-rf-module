/* rlink.c — RLINK step R6 (see rlink.h).  #included into app_process.c.
 * Specified in "RLINK Protocol Specification" and "RLINK System Specification". */
#include "rlink.h"
#include "rx_setup.h"   /* receiver setup over the air: the setup packets share RLINK's PHY (rx_setup.c) */
#include "servo_pins.h"
#include "rail_config.h"
#include "rx_store.h"
#include "em_device.h"
#include <string.h>
#if defined(SL_CATALOG_APP_LOG_PRESENT)
#include "app_log.h"
#endif
#include "em_cmu.h"
#include "em_timer.h"
#include "em_eusart.h"

extern const RAIL_ChannelConfig_t nrf24_250k_channelConfig;   /* carrier PHY: 2GFSK 250 kbps */

/* ---- knobs ---- */
#define RL_HOP_SEED        0x3E91u
#define RL_CHANNELS        39u        /* channel A pool: RF channels 2..40            */
#define RL_B_OFFSET        40u        /* channel B = A + 40  ->  42..80               */
#define RL_FRAME_US        11000u
#define RL_COPY_US         2500u      /* copy B starts this long after copy A          */
#define RL_FRAME_LEN       16u        /* hdr, base, flags, 8 x 11 bit (11), CRC16 (2)  */
#define RL_PAYLOAD         14u
#define RL_FAILSAFE_MS     1000u
#define RL_PPM_CHANNELS    8u
#define RL_PPM_FRAME_US    22500u
#define RL_PPM_PULSE_US    300u
#define RL_TX_POWER_DDBM   200        /* full power: module maximum (RAIL clamps, ~+19.5 dBm) */
#define RL_RANGE_POWER_DDBM (-100)    /* -10 dBm with RANGE low (range check)          */
#define RL_TX_LEAD_US      1000u      /* build + schedule copy A this early            */
#define RL_TX_MIN_SLACK_US 150u       /* closer than this: send immediately, count late */
#define RL_LQ_WINDOW       90u
#define RL_ACQ_STEP_US     4000u
#define RL_LEAD_US         1500u      /* RX: move to copy B's channel this early       */
#define RL_MISS_PAD_US     3000u
#define RL_BIND_CH         60u        /* 2460 MHz                                      */
#define RL_BIND_SYNC       0x6B2E91D3u
#define RL_BIND_MS         1500u      /* TX bind: at least this long; encoder mode: on while BIND is on in the frames */
#define RL_BIND_TIMEOUT_US 8000000u
#define RL_COMMIT_PKTS     20u
#define RL_FS_PRESS_US     300000u
#ifndef RL_FS_NOPULSES
#define RL_FS_NOPULSES     0
#endif
#define RL_DEFAULT_SYNC    0x4A35C9E6u
/* R2 telemetry: every RL_TELE_EVERY-th frame the RX answers RL_TELE_US after copy A,
 * alternating between that frame's A and B channel.  Frame use: A 0-0.7 ms,
 * B 2.5-3.2 ms, telemetry ~6.3-7.0 ms, next A at 11 ms. */
#define RL_TELE_EVERY      4u         /* must be a power of two                        */
#define RL_TELE_US         5500u      /* after the RX's copy-A time (which lags ~0.8 ms) */
#define RL_TELE_LEAD_US    700u       /* RX: load + schedule this early                */
#define RL_TELE_FRESH_US   300000u    /* TX: telemetry counts as live this long         */
/* R3a channel blacklisting (RX measures and proposes, TX announces, both switch at base 0) */
#define RL_EVAL_FRAMES     390u       /* evaluate channel quality every 10 hop cycles (~4.3 s) */
#define RL_BAD_MIN_TRIES   5u         /* need this many tries on a channel before judging    */
#define RL_BAD_PCT         40u        /* copy lost on >= this % of tries -> blacklist        */
#define RL_SUB_STEP        7u         /* substitute = hop slot this far ahead (coprime 39)  */
#define RL_BL_MAX          12u        /* at most this many blacklisted channels per pool     */
#define RL_BL_HOLD_US      60000000u  /* re-test a blacklisted channel after 60 s            */
#define RL_MAP_MISMATCH    6u         /* TX: consecutive RX reports with another map -> reset */
#define RL_SIM_LO          12u        /* RX + RANGE low at boot: pretend A channels          */
#define RL_SIM_HI          20u        /*   12..20 (2412-2420 MHz) are jammed (bench test)    */
/* R3b automatic TX power: fast up, slow down, driven by the RX's telemetry */
#ifndef RL_PC_ENABLE
#define RL_PC_ENABLE       1
#endif
#define RL_PC_TARGET_DBM   (-65)      /* aim for this RSSI at the RX (sensitivity ~-100)  */
#define RL_PC_MIN_DDBM     0          /* never below 0 dBm                                */
#define RL_PC_STEP_DDBM    20         /* 2 dB per step                                    */
#define RL_PC_DOWN_US      500000u    /* at most one step down per 0.5 s                  */
#define RL_PC_LOST_US      150000u    /* no telemetry this long -> full power at once     */
#define RL_PC_LQ_MIN       95         /* RX link quality below this -> full power at once */
/* R3c alarms (TX): beeps on the PD02 buzzer, PA07 high while an alarm is active, status line on the bay UART */
#define RL_ALM_TELE_US     300000u    /* no telemetry this long -> critical              */
#define RL_ALM_LQ_WARN     80         /* RX link quality below this -> warning            */
#define RL_ALM_RSSI_WARN   (-85)      /* RSSI at the RX below this at full power -> warning */
#define RL_ALM_HOLD_US     1000000u   /* an alarm stays on at least this long             */
#ifndef RL_UART_STATUS
#define RL_UART_STATUS     1          /* 1 = status line on the bay UART (PA05) 5x/s      */
#endif
#define RL_UART_EVERY      18u        /* frames between status lines (~200 ms)            */
#ifndef RL_ALARM_TONE
#define RL_ALARM_TONE      1          /* 1 = on-board passive buzzer on PD02 beeps alarms  */
#endif
#define RL_TONE_HZ         2700u      /* typical passive-buzzer resonance                  */
/* R4 listen-before-talk: one clear-channel check right before every transmission.
 * Busy -> that packet is skipped (no retry, no wait); copy B / the next frame carry on. */
#ifndef RL_LBT_ENABLE
#define RL_LBT_ENABLE      1
#endif
#define RL_LBT_CCA_US      100u       /* listen time before each packet (EN 300 328: >= 18 us) */
#define RL_LBT_TIMEOUT_US  400u       /* hard limit for the whole check -> never blocks        */
#define RL_LBT_TL_20DBM    (-70)      /* busy threshold at +20 dBm EIRP, dBm (in ~1 MHz)       */
#define RL_ANT_GAIN_DB     2          /* antenna gain assumed for the EIRP-scaled threshold    */
/* R5 SBUS output (RX): shared writer in sbus_out.c, one frame per received RLINK frame */
#define RL_SBUS_PERIOD_US  0          /* 0 = per received frame (~90 Hz), else a fixed period in us */
/* R6 RX battery: divider Vbatt - 47k - PA04 - 10k - GND (100 nF at PA04), read by IADC0 on the RX,
 * sent in the telemetry stats packet (10 bit, 10 mV), alarmed on the TX. No divider fitted -> "no sensor". */
#ifndef RL_VB_ENABLE
#define RL_VB_ENABLE       1
#endif
#define RL_VB_R_TOP        47000u     /* ohm, Vbatt side                                   */
#define RL_VB_R_BOT        10000u     /* ohm, GND side                                     */
#define RL_VB_TRIM         1000u      /* per mille; = 1000 * multimeter / reported voltage */
#define RL_VB_FACTORY_GAIN 1          /* 1 = apply the chip's factory IADC gain and offset trims (DEVINFO) */
#define RL_VB_OSR          16u        /* IADC oversampling (2..64, power of 2); r_vb_scale() depends on it */
/* Pack type: RL_VB_PACK 1 = always 4-cell / 5 V (4.8 V NiMH, 4 x AA, 5 V BEC), 2 = always 2S LiPo,
 * 0 = the TX picks it from the first reading it hears (above RL_VB_2S_ABOVE_MV = 2S) and keeps it until it
 * is switched off, which gets it wrong when the first pack it hears is not the one flown later.
 * Fixed to 4 x AA since Oct 2026. 5-cell NiMH or 2S LiFe: set fixed values below. */
#define RL_VB_PACK         1
#define RL_VB_2S_ABOVE_MV  6800u      /* RL_VB_PACK 0 only: 4 x AA alkaline fresh is ~6.4 V, 2S LiPo is >= 7.0 V */
#define RL_VB_WARN_MV      4600u      /* 4-cell / 5 V: 1.15 V/cell -> 3 beeps              */
#define RL_VB_CRIT_MV      4400u      /* 4-cell / 5 V: 1.10 V/cell -> continuous           */
#define RL_VB_WARN_2S_MV   7200u      /* 2S LiPo: 3.6 V/cell -> 3 beeps                    */
#define RL_VB_CRIT_2S_MV   6800u      /* 2S LiPo: 3.4 V/cell -> continuous ("land now")    */
#define RL_VB_HYST_MV      200u       /* alarm clears only above WARN + this ...           */
#define RL_VB_TRIG_US      3000000u   /* below a threshold this long -> alarm (servo sag)  */
#define RL_VB_CLEAR_US     10000000u  /* ... for this long (i.e. after a battery swap)     */
#define RL_VB_SAMPLE_US    10000u     /* one conversion per 10 ms, filter tau ~0.64 s      */
#define RL_VB_NONE_MV      200u       /* PA04 below this with pull-down on -> no divider   */
#define RL_HDR_BIND        0x01u
#define RL_HDR_TELE        0x02u

volatile int32_t rlink_diag[RL_DIAG_N];

static sl_rail_handle_t r_rail;
static const volatile uint16_t *r_ch;
static bool     r_rx_role;
static uint8_t  r_hop[RL_CHANNELS];
static uint8_t  r_base;
static uint8_t  r_seq;
static uint8_t  r_pkt[RL_FRAME_LEN];
static uint8_t  r_rx_buf[24];

static sl_gpio_t r_led_a    = { .port = SL_GPIO_PORT_C, .pin = 4 };
static sl_gpio_t r_led_b    = { .port = SL_GPIO_PORT_C, .pin = 5 };
static sl_gpio_t r_alarm_pin= { .port = SL_GPIO_PORT_A, .pin = 7 };   /* TX: alarm level for the encoder (= servo 1 on the RX) */
static sl_gpio_t r_buzz_pin = { .port = SL_GPIO_PORT_D, .pin = 2 };   /* TX: on-board passive buzzer */
/* Tone for the passive buzzer: TIMER1 CC0 in PWM mode at RL_TONE_HZ, routed to PD02
 * only while sounding (route off -> the pin falls back to GPIO low). */
static void r_tone_init(void)
{
  CMU_ClockEnable(cmuClock_TIMER1, true);
  TIMER_Init_TypeDef ti = TIMER_INIT_DEFAULT; ti.enable = false;
  TIMER_Init(TIMER1, &ti);
  TIMER_InitCC_TypeDef cc = TIMER_INITCC_DEFAULT; cc.mode = timerCCModePWM;
  TIMER_InitCC(TIMER1, 0, &cc);
  uint32_t top = CMU_ClockFreqGet(cmuClock_TIMER1) / RL_TONE_HZ;
  if(top > 0xFFFFu) top = 0xFFFFu;
  TIMER_TopSet(TIMER1, top);
  TIMER_CompareSet(TIMER1, 0, top/2u);
  GPIO->TIMERROUTE[1].CC0ROUTE = ((uint32_t)r_buzz_pin.port << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                               | ((uint32_t)r_buzz_pin.pin  << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
  GPIO->TIMERROUTE[1].ROUTEEN = 0;
  TIMER_Enable(TIMER1, true);
}
static void r_tone(bool on){ GPIO->TIMERROUTE[1].ROUTEEN = on ? GPIO_TIMER_ROUTEEN_CC0PEN : 0u; }

/* ---- R6 RX battery (RX side): IADC0 single conversions on PA04, never waits for a result ---- */
static sl_gpio_t r_vb_pin = { .port = SL_GPIO_PORT_A, .pin = 4 };
static uint8_t  r_vb_state;          /* 0 off / no sensor, 1 probing with pull-down, 2 measuring */
static uint32_t r_vb_next, r_vb_t0;
static int32_t  r_vb_f;              /* filtered ADC code x256 */
static uint32_t r_vb_psum; static uint8_t r_vb_pn;
static uint16_t r_vb_mv;             /* RX: battery mV, 0 = no sensor */

/* IADC SCALE (gain and offset correction) for normal mode, analog gain 0.5x, VBGR reference and
 * RL_VB_OSR, worked out as IADC_Init() in Silicon Labs' em_iadc.c does it. The register's reset value
 * only fits OSR 2: the ADC core has a systematic offset of 640 * 256 / OSR result LSBs, and the reset
 * offset (-81920) cancels it for OSR 2. With OSR 16 that same value takes 560 codes too many off every
 * reading (0.33 V at the pin, 1.9 V at the battery): found on the bench in October 2026. */
static uint32_t r_vb_scale(void)
{
  uint32_t g = DEVINFO->IADC0GAIN0 & _DEVINFO_IADC0GAIN0_GAINCANA1_MASK;        /* gain trim for 0.5x and 1x */
  bool trim = (RL_VB_FACTORY_GAIN != 0) && g != 0u && g != 0xFFFFu;
  uint32_t gain = trim ? g : 0x8000u;                                           /* 0x8000 = gain 1.000 */
  float off = 0.0f;
  if(trim){                                                                     /* chip offset trims, 16-bit two's complement */
    int32_t o3    = (int16_t)(DEVINFO->IADC0NORMALOFFSETCAL1 & _DEVINFO_IADC0NORMALOFFSETCAL1_OFFSETANA3NORM_MASK);
    int32_t obase = (int16_t)(DEVINFO->IADC0OFFSETCAL0 & _DEVINFO_IADC0OFFSETCAL0_OFFSETANABASE_MASK);
    off = (float)o3 / ((float)RL_VB_OSR / 2.0f) + (float)obase;                 /* OSR above 2 */
    off *= 1.25f / 1.21f;                                                       /* trims are for 1.25 V, VBGR is 1.21 V */
  }
  off = off * 4.0f + 640.0f * (256.0f / (float)RL_VB_OSR);                      /* systematic offset of the core */
  if(gain != 0x8000u) off = ((float)gain / 32768.0f) * (off + 524288.0f) - 524288.0f;   /* through the gain correction */
  float m = -off;
  int32_t io = (int32_t)(m < 0.0f ? m - 0.5f : m + 0.5f);                       /* round(-off), 18-bit two's complement */
  uint32_t ofs = io > 131071 ? 0x1FFFFu : io < -131072 ? 0x20000u : ((uint32_t)io & 0x3FFFFu);
  return ((gain & 0x1FFFu) << _IADC_SCALE_GAIN13LSB_SHIFT) | ((gain & 0x8000u) ? IADC_SCALE_GAIN3MSB : 0u) | ofs;
}

static void r_vb_init(uint32_t now)
{
  r_vb_mv=0; r_vb_state=0;
  if(!RL_VB_ENABLE) return;
  CMU->CLKEN0_SET  = CMU_CLKEN0_IADC0;
  CMU->IADCCLKCTRL = CMU_IADCCLKCTRL_CLKSEL_FSRCO;                  /* 20 MHz */
  IADC0->EN_CLR = IADC_EN_EN;
  IADC0->CTRL   = (19u << _IADC_CTRL_TIMEBASE_SHIFT) | IADC_CTRL_WARMUPMODE_NORMAL;   /* 1 us timebase */
  IADC0->CFG[0].CFG = (_IADC_CFG_RESETVALUE & ~(_IADC_CFG_OSRHS_MASK|_IADC_CFG_ANALOGGAIN_MASK|_IADC_CFG_REFSEL_MASK))
                    | IADC_CFG_ADCMODE_NORMAL | ((31u - (uint32_t)__builtin_clz(RL_VB_OSR) - 1u) << _IADC_CFG_OSRHS_SHIFT)
                    | IADC_CFG_ANALOGGAIN_ANAGAIN0P5 | IADC_CFG_REFSEL_VBGR;           /* 0..2.42 V */
  IADC0->CFG[0].SCALE = r_vb_scale();                                 /* gain and offset correction for this OSR */
  IADC0->CFG[0].SCHED = 1u;                                           /* CLK_ADC 10 MHz */
  GPIO->ABUSALLOC = (GPIO->ABUSALLOC & ~_GPIO_ABUSALLOC_AEVEN0_MASK) | GPIO_ABUSALLOC_AEVEN0_ADC0;
  IADC0->EN_SET = IADC_EN_EN;
  IADC0->SINGLE = IADC_SINGLE_PORTPOS_PORTA | ((uint32_t)r_vb_pin.pin << _IADC_SINGLE_PINPOS_SHIFT) | IADC_SINGLE_PORTNEG_GND;
  sl_gpio_set_pin_mode(&r_vb_pin, SL_GPIO_MODE_INPUT_PULL, 0);        /* pull-down: an open pin reads 0 */
  r_vb_state=1; r_vb_t0=now; r_vb_next=now; r_vb_psum=0; r_vb_pn=0;
}

static uint32_t r_vb_code_to_mv(uint64_t code_x256)
{
  return (uint32_t)((code_x256 * 2420ull * (RL_VB_R_TOP+RL_VB_R_BOT) * RL_VB_TRIM)
                    / (4095ull * 256ull * RL_VB_R_BOT * 1000ull));
}

static void r_vb_service(uint32_t now)
{
  if(r_vb_state==0u || (int32_t)(now-r_vb_next)<0) return;
  r_vb_next = now + RL_VB_SAMPLE_US;
  if(IADC0->STATUS & IADC_STATUS_SINGLEFIFODV){
    uint32_t code = IADC0->SINGLEFIFODATA & 0xFFFu;
    if(r_vb_state==1u){
      if((now-r_vb_t0) > 50000u){ r_vb_psum+=code; r_vb_pn++; }       /* let the pin settle first */
      if(r_vb_pn>=8u){
        uint32_t pin_mv = (r_vb_psum/8u)*2420u/4095u;
        if(pin_mv < RL_VB_NONE_MV){ r_vb_state=0; IADC0->EN_CLR=IADC_EN_EN; rlink_diag[R_VB_ALM]=0; return; }
        sl_gpio_set_pin_mode(&r_vb_pin, SL_GPIO_MODE_DISABLED, 0);    /* divider found: pure analog */
        r_vb_state=2; r_vb_f=-1;
      }
    } else {
      if(r_vb_f<0) r_vb_f=(int32_t)(code<<8);
      else r_vb_f += ((int32_t)(code<<8) - r_vb_f) >> 6;
      uint32_t mv = r_vb_code_to_mv((uint64_t)r_vb_f);
      r_vb_mv = (uint16_t)(mv<10u ? 10u : mv>10230u ? 10230u : mv);
      rlink_diag[R_VBAT]=r_vb_mv;
    }
    rlink_diag[R_VB_ALM]=r_vb_state;
  }
  IADC0->CMD = IADC_CMD_SINGLESTART;
}

/* ---- R6 RX battery (TX side): 3 beeps below WARN, continuous below CRIT, latched ---- */
static int32_t  r_vb_lvl;
static uint8_t  r_vb_prof;           /* TX: 0 not yet known, 1 4-cell / 5 V, 2 2S LiPo */
static uint32_t r_vb_low_s, r_vb_crit_s, r_vb_ok_s;
static uint32_t r_since(bool c, uint32_t s, uint32_t now){ return c ? (s ? s : (now|1u)) : 0u; }
static void r_vb_alarm(uint32_t now, bool fresh)
{
  uint32_t mv=(uint32_t)rlink_diag[R_VBAT];
  if(mv==0u){ r_vb_lvl=0; r_vb_low_s=r_vb_crit_s=r_vb_ok_s=0; return; }  /* no sensor on the RX */
  if(!fresh){ r_vb_low_s=r_vb_crit_s=r_vb_ok_s=0; return; }              /* no news: keep the level */
  if(r_vb_prof==0u) r_vb_prof = RL_VB_PACK ? (uint8_t)RL_VB_PACK : ((mv > RL_VB_2S_ABOVE_MV) ? 2u : 1u);   /* fixed pack, or the first reading picks it */
  uint32_t warn = (r_vb_prof==2u) ? RL_VB_WARN_2S_MV : RL_VB_WARN_MV;
  uint32_t crit = (r_vb_prof==2u) ? RL_VB_CRIT_2S_MV : RL_VB_CRIT_MV;
  r_vb_crit_s = r_since(mv <  crit, r_vb_crit_s, now);
  r_vb_low_s  = r_since(mv <  warn, r_vb_low_s,  now);
  r_vb_ok_s   = r_since(mv >= warn+RL_VB_HYST_MV, r_vb_ok_s, now);
  if(r_vb_crit_s && (now-r_vb_crit_s)>=RL_VB_TRIG_US) r_vb_lvl=2;
  else if(r_vb_low_s && (now-r_vb_low_s)>=RL_VB_TRIG_US && r_vb_lvl<1) r_vb_lvl=1;
  if(r_vb_ok_s && (now-r_vb_ok_s)>=RL_VB_CLEAR_US) r_vb_lvl=0;
}

static uint8_t  r_id[4];
static volatile bool r_in_ok;                    /* TX: encoder PPM present (set by app_process) */
void rlink_input(bool ok){ r_in_ok = ok; }
static uint16_t r_seed;
static uint32_t r_data_sync;

/* ------------------------------------------------------------ identity/hop */
static uint32_t r_xs(uint32_t *s){ uint32_t x=*s; x^=x<<13; x^=x>>17; x^=x<<5; *s=x; return x; }
static void r_make_hop(uint16_t seed)
{
  uint32_t st = ((uint32_t)seed << 16) | seed;
  if (st == 0u) st = 0xDEADBEEFu;
  for (uint8_t i=0;i<RL_CHANNELS;i++) r_hop[i]=(uint8_t)(2u+i);            /* A pool 2..40 */
  for (uint8_t i=RL_CHANNELS-1u;i>0;--i){ uint8_t j=(uint8_t)(r_xs(&st)%(i+1u)); uint8_t t=r_hop[j]; r_hop[j]=r_hop[i]; r_hop[i]=t; }
}
/* Blacklist masks: bit i = A channel i+2 (maskA) or B channel i+42 (maskB).
 * A blacklisted channel is replaced by the usable one RL_SUB_STEP slots further in
 * hop order (7 is coprime to 39, so runs of bad slots do not repeat one channel); the
 * base sequence (and all timing) is unchanged; only the RF channel differs. */
static uint64_t r_mA, r_mB;                      /* active masks                        */
static uint8_t  r_map_ver;                       /* active map version, 0 = empty       */
static uint8_t r_chan(uint8_t base, uint8_t copy)
{
  uint64_t m = copy ? r_mB : r_mA;
  uint8_t idx = (uint8_t)(base % RL_CHANNELS);
  for(uint8_t k=0;k<RL_CHANNELS;k++){
    uint8_t c=r_hop[idx];
    if(!((m>>(c-2u))&1u)) return (uint8_t)(c + (copy ? RL_B_OFFSET : 0u));
    idx=(uint8_t)((idx+RL_SUB_STEP)%RL_CHANNELS);
  }
  return (uint8_t)(r_hop[base % RL_CHANNELS] + (copy ? RL_B_OFFSET : 0u));
}
static uint8_t r_popc(uint64_t m){ uint8_t n=0; while(m){ m&=m-1u; n++; } return n; }
static void r_put39(uint8_t *d, uint64_t m){ for(uint8_t i=0;i<5;i++) d[i]=(uint8_t)(m>>(8u*i)); }
static uint64_t r_get39(const uint8_t *d){ uint64_t m=0; for(uint8_t i=0;i<5;i++) m|=(uint64_t)d[i]<<(8u*i); return m & ((1ull<<RL_CHANNELS)-1u); }

static void r_derive(const uint8_t id[4])
{
  memcpy(r_id, id, 4);
  bool def = (id[0]|id[1]|id[2]|id[3]) == 0;
  r_seed = def ? RL_HOP_SEED : (uint16_t)(id[0]+id[1]+id[2]+(uint16_t)id[3]*11u+1u);
  r_data_sync = def ? RL_DEFAULT_SYNC
                    : (((uint32_t)id[0]<<24)|((uint32_t)id[1]<<16)|((uint32_t)id[2]<<8)|0x69u);
  r_make_hop(r_seed);
}
static void r_gen_id(void)                     /* TX: unique id from the chip EUI64 */
{
  uint64_t u = ((uint64_t)DEVINFO->EUI64H << 32) | DEVINFO->EUI64L;
  uint8_t id[4]; for(int i=0;i<3;i++) id[i]=(uint8_t)((u>>(i*8))&0xFF);
  id[3] = (uint8_t)((u>>32) ^ (u>>40) ^ 0xA5u);
  id[0] ^= rc_model; id[1] ^= (uint8_t)(rc_model * 37u);    /* per-model binding; 0 = unchanged */
  if((id[0]|id[1]|id[2]|id[3])==0) id[0]=1;
  r_derive(id);
}
static uint16_t r_crc(const uint8_t *d, uint8_t n)
{ uint16_t c=0xFFFFu; for(uint8_t i=0;i<n;i++){ c^=(uint16_t)d[i]<<8; for(uint8_t b=0;b<8;b++) c=(c&0x8000u)?(uint16_t)((c<<1)^0x1021u):(uint16_t)(c<<1);} return c; }

/* Sync word handling.  In this SDK sl_rail_config_channels() only caches the
 * PHY; the first start_tx/start_rx/prepare_channel applies it and thereby
 * resets the sync word to the radio-configuration default.  So we remember the
 * wanted word and verify it (and re-apply if needed) before every TX and RX. */
static uint32_t r_want_sync;
static void r_apply_sync(void)
{
  sl_rail_idle(r_rail, SL_RAIL_IDLE, true);
  sl_rail_sync_word_config_t sc = { .sync_word_bits=32, .sync_word_0=r_want_sync, .sync_word_1=0 };
  rlink_diag[R_SYNC_ST]=(int32_t)sl_rail_config_sync_words(r_rail, &sc);
}
static void r_set_sync(uint32_t word){ r_want_sync=word; r_apply_sync(); }
static void r_ensure_sync(void)                 /* radio must be idle or idling */
{
  sl_rail_sync_word_config_t cur;
  if(sl_rail_get_sync_words(r_rail,&cur)!=SL_RAIL_STATUS_NO_ERROR || cur.sync_word_0!=r_want_sync){
    rlink_diag[R_SYNC_FIX]++; r_apply_sync();
  }
}

/* ------------------------------------------------------------ packing (11 bit) */
static uint16_t r_to11(uint16_t us){ if(us<900u)us=900u; if(us>2100u)us=2100u; return (uint16_t)(((uint32_t)(us-900u)*2047u+600u)/1200u); }
static uint16_t r_from11(uint16_t v){ if(v>2047u)v=2047u; return (uint16_t)(900u+((uint32_t)v*1200u+1023u)/2047u); }

/* TX map state */
static bool     r_tx_announce; static uint8_t r_tx_next_ver; static uint64_t r_tx_nA, r_tx_nB; static uint8_t r_tx_mism;
static void r_pack(bool fs_flag)
{
  r_pkt[0] = (uint8_t)((r_seq & 0x3Fu) << 2);           /* bit0 = bind (0 here) */
  r_pkt[1] = (uint8_t)(r_base | (r_in_ok ? 0u : 0x80u) | (fs_flag ? 0x40u : 0u));   /* bit 7 = TX input lost, bit 6 = store failsafe */
  r_pkt[2] = (uint8_t)((r_map_ver<<4) | (r_tx_announce ? r_tx_next_ver : 0u));  /* active map | announced map */
  uint32_t acc=0; uint8_t nb=0, o=3;
  for(uint8_t i=0;i<8;i++){ acc |= (uint32_t)(r_to11(r_ch[i])&0x7FFu)<<nb; nb+=11; while(nb>=8){ r_pkt[o++]=(uint8_t)(acc&0xFF); acc>>=8; nb-=8; } }
  if(nb) r_pkt[o++]=(uint8_t)(acc&0xFF);                /* 88 bits = exactly 11 bytes */
  uint16_t c=r_crc(r_pkt,RL_PAYLOAD);
  r_pkt[RL_PAYLOAD]=(uint8_t)(c>>8); r_pkt[RL_PAYLOAD+1u]=(uint8_t)c;
}
static void r_pack_bind(void)
{
  memset(r_pkt,0,RL_FRAME_LEN);
  r_pkt[0]=0x01;
  r_pkt[1]=r_id[0]; r_pkt[2]=r_id[1]; r_pkt[3]=r_id[2]; r_pkt[4]=r_id[3];
  uint16_t c=r_crc(r_pkt,RL_PAYLOAD);
  r_pkt[RL_PAYLOAD]=(uint8_t)(c>>8); r_pkt[RL_PAYLOAD+1u]=(uint8_t)c;
}

/* ------------------------------------------------------------ PPM + servo out (RX): servo_out.c (TIMER2) reads r_out_us */
static uint16_t r_out_us[8];
static uint32_t r_last_rx;                 /* time of the last valid data packet (dual receiver) */
static void r_ppm_enable(bool on){ servo_out_enable(on); }
bool rlink_rx_fresh(uint32_t now, uint32_t within_us){ return r_last_rx != 0u && (now - r_last_rx) <= within_us; }

/* ------------------------------------------------------------ radio helpers */
static void r_load(void)
{
  sl_rail_idle(r_rail,SL_RAIL_IDLE,false);
  r_ensure_sync();
  sl_rail_set_fixed_length(r_rail,RL_FRAME_LEN);
  sl_rail_write_tx_fifo(r_rail,r_pkt,RL_FRAME_LEN,true);   /* r_pkt: data, bind or (RX) telemetry */
}
/* LBT settings for the next transmission: single check, no backoff, no retry, hard timeout.
 * Threshold scales with EIRP: TL = -70 dBm + (20 dBm - EIRP), limited to -85..-45 dBm. */
static sl_rail_lbt_config_t r_lbt;
static const sl_rail_lbt_config_t *r_lbt_cfg(void)
{
  int32_t eirp = (int32_t)sl_rail_get_tx_power_dbm(r_rail)/10 + RL_ANT_GAIN_DB;
  int32_t tl = RL_LBT_TL_20DBM + (20 - eirp);
  if(tl < -85) tl = -85;
  if(tl > -45) tl = -45;
  r_lbt.lbt_min_bo_rand = 0; r_lbt.lbt_max_bo_rand = 0;     /* fixed, no random backoff */
  r_lbt.lbt_tries = 1;                                     /* one check: busy -> give up */
  r_lbt.lbt_threshold_dbm = (int8_t)tl;
  r_lbt.lbt_backoff_us = 0;
  r_lbt.lbt_duration_us = RL_LBT_CCA_US;
  r_lbt.lbt_timeout_us = RL_LBT_TIMEOUT_US;
  rlink_diag[R_LBT_TL] = tl;
  return &r_lbt;
}
static bool r_lbt_refused;                       /* radio refused an LBT start at least once */
static sl_rail_status_t r_start_now(uint8_t ch)
{
  if(RL_LBT_ENABLE){
    sl_rail_status_t s=sl_rail_start_cca_lbt_tx(r_rail,ch,SL_RAIL_TX_OPTIONS_DEFAULT,r_lbt_cfg(),NULL);
    if(s==SL_RAIL_STATUS_NO_ERROR) return s;
    r_lbt_refused=true;                            /* never let LBT silence the link: send plain */
  }
  return sl_rail_start_tx(r_rail,ch,SL_RAIL_TX_OPTIONS_DEFAULT,NULL);
}
static void r_tx_now(uint8_t ch)
{
  r_load();
  if(r_start_now(ch)!=SL_RAIL_STATUS_NO_ERROR) rlink_diag[R_TXER]++;
}
/* Launch the loaded packet at absolute time `when` (radio hardware timing). */
static bool r_tx_at(uint8_t ch, uint32_t when)
{
  r_load();
  uint32_t now=(uint32_t)sl_rail_get_time(r_rail);
  sl_rail_status_t s;
  if((int32_t)(when-now) < (int32_t)RL_TX_MIN_SLACK_US){
    rlink_diag[R_LATE]++;
    s=r_start_now(ch);
  } else {
    sl_rail_scheduled_tx_config_t cfg = { .when=when, .mode=SL_RAIL_TIME_ABSOLUTE,
                                          .tx_during_rx=SL_RAIL_SCHEDULED_TX_DURING_RX_POSTPONE_TX };
    s=SL_RAIL_STATUS_INVALID_STATE;
    if(RL_LBT_ENABLE){
      s=sl_rail_start_scheduled_cca_lbt_tx(r_rail,ch,SL_RAIL_TX_OPTIONS_DEFAULT,&cfg,r_lbt_cfg(),NULL);
      if(s!=SL_RAIL_STATUS_NO_ERROR) r_lbt_refused=true;
    }
    if(s!=SL_RAIL_STATUS_NO_ERROR) s=sl_rail_start_scheduled_tx(r_rail,ch,SL_RAIL_TX_OPTIONS_DEFAULT,&cfg,NULL);
  }
  if(s!=SL_RAIL_STATUS_NO_ERROR){ rlink_diag[R_TXER]++; return false; }
  return true;
}
static uint8_t r_cur_ch;                         /* RX: channel currently listened on */
static void r_listen(uint8_t ch){ r_cur_ch=ch; sl_rail_idle(r_rail,SL_RAIL_IDLE,false); r_ensure_sync(); sl_rail_set_fixed_length(r_rail,RL_FRAME_LEN); (void)sl_rail_start_rx(r_rail,ch,NULL); }

/* ------------------------------------------------------------ RX channel statistics + map proposal */
static uint8_t  r_aT[RL_CHANNELS], r_aB[RL_CHANNELS], r_bT[RL_CHANNELS], r_bB[RL_CHANNELS];  /* tries / bad per channel */
static uint32_t r_blA_t[RL_CHANNELS], r_blB_t[RL_CHANNELS];   /* when each channel was blacklisted */
static uint64_t r_pA, r_pB; static uint8_t r_prop_ver;         /* proposal (== active when nothing pending) */
static bool     r_map_armed;                                    /* TX announced our proposal: switch at base 0 */
static uint16_t r_eval_n;
static bool     r_sim_jam;
static void r_stat(uint8_t ch, bool bad)
{
  if(ch>=2u && ch<2u+RL_CHANNELS){ uint8_t i=(uint8_t)(ch-2u); if(r_aT[i]<250u){ r_aT[i]++; if(bad) r_aB[i]++; } }
  else if(ch>=2u+RL_B_OFFSET && ch<2u+RL_B_OFFSET+RL_CHANNELS){ uint8_t i=(uint8_t)(ch-2u-RL_B_OFFSET); if(r_bT[i]<250u){ r_bT[i]++; if(bad) r_bB[i]++; } }
}
static void r_map_diag(void)
{
  rlink_diag[R_MAP_VER]=r_map_ver; rlink_diag[R_MAP_NA]=r_popc(r_mA); rlink_diag[R_MAP_NB]=r_popc(r_mB);
  rlink_diag[R_MAP_A_LO]=(int32_t)(uint32_t)r_mA; rlink_diag[R_MAP_A_HI]=(int32_t)(uint32_t)(r_mA>>32);
  rlink_diag[R_MAP_B_LO]=(int32_t)(uint32_t)r_mB; rlink_diag[R_MAP_B_HI]=(int32_t)(uint32_t)(r_mB>>32);
}
static void r_map_activate(uint8_t ver, uint64_t a, uint64_t b)
{
  r_mA=a; r_mB=b; r_map_ver=ver; rlink_diag[R_MAP_SW]++; r_map_diag();
}
static void r_map_reset(void)
{
  r_mA=0; r_mB=0; r_map_ver=0; r_pA=0; r_pB=0; r_prop_ver=0; r_map_armed=false;
  r_tx_announce=false; r_tx_next_ver=0; r_tx_mism=0;
  memset(r_aT,0,sizeof r_aT); memset(r_aB,0,sizeof r_aB); memset(r_bT,0,sizeof r_bT); memset(r_bB,0,sizeof r_bB);
  r_eval_n=0; r_map_diag(); rlink_diag[R_MAP_PEND]=0;
}
static uint8_t r_ver_next(uint8_t v){ return (uint8_t)((v % 15u) + 1u); }   /* 1..15, 0 = empty map */
static void r_eval(uint32_t now)                  /* RX: every RL_EVAL_FRAMES frames */
{
  if(r_map_armed) return;                          /* TX is already switching to our proposal: keep it */
  uint64_t a=r_pA, b=r_pB;
  for(uint8_t i=0;i<RL_CHANNELS;i++){
    uint64_t bit=1ull<<i;
    if((a&bit) && (now-r_blA_t[i])>=RL_BL_HOLD_US) a&=~bit;       /* age out: re-test */
    if((b&bit) && (now-r_blB_t[i])>=RL_BL_HOLD_US) b&=~bit;
    if(!(a&bit) && r_aT[i]>=RL_BAD_MIN_TRIES && (uint32_t)r_aB[i]*100u >= (uint32_t)r_aT[i]*RL_BAD_PCT && r_popc(a)<RL_BL_MAX){ a|=bit; r_blA_t[i]=now; }
    if(!(b&bit) && r_bT[i]>=RL_BAD_MIN_TRIES && (uint32_t)r_bB[i]*100u >= (uint32_t)r_bT[i]*RL_BAD_PCT && r_popc(b)<RL_BL_MAX){ b|=bit; r_blB_t[i]=now; }
  }
  memset(r_aT,0,sizeof r_aT); memset(r_aB,0,sizeof r_aB); memset(r_bT,0,sizeof r_bT); memset(r_bB,0,sizeof r_bB);
  if(a!=r_pA || b!=r_pB){
    r_pA=a; r_pB=b;
    if(a==r_mA && b==r_mB) r_prop_ver=r_map_ver;
    else {                                         /* new version, never reusing one the TX may hold */
      uint8_t v=r_ver_next(r_prop_ver!=r_map_ver ? r_prop_ver : r_map_ver);
      if(v==r_map_ver) v=r_ver_next(v);
      r_prop_ver=v;
    }
  }
  rlink_diag[R_MAP_PEND]=(r_prop_ver!=r_map_ver)?r_prop_ver:0;
}
static void r_frame_tick(uint32_t now){ if(++r_eval_n>=RL_EVAL_FRAMES){ r_eval_n=0; r_eval(now); } }
static void r_rx_base_step(void)                  /* RX: r_base just advanced; switch maps on base 0 */
{
  if(r_base==0u && r_map_armed){ r_map_armed=false; r_map_activate(r_prop_ver, r_pA, r_pB); }
}

/* ------------------------------------------------------------ state */
static uint16_t r_win_frames, r_win_lost, r_worst_ms;
static uint32_t r_dropout_start_us; static bool r_in_dropout;
/* TX */
static uint32_t r_frame_us;                 /* absolute start of the current frame's copy A */
static uint8_t  r_tx_state;                 /* 0 idle, 1 copy A in flight, 2 copy B in flight */
static volatile bool r_tx_done;             /* set by the RAIL ISR on any TX completion */
static bool     r_tx_bind; static uint32_t r_tx_bind_end, r_tx_bind_next;
static bool     r_bind_hold, r_bind_noted;      /* TX: BIND on in the serial frames (rlink_bind); "still binding" logged */
static bool     r_range_low;
static uint32_t r_tele_last_us;             /* TX: time of the last telemetry packet heard  */
static uint32_t r_sbus_next_us;            /* RX: next SBUS frame in fixed-period mode */
static bool     r_status_due;               /* TX: print the UART status line now          */
static bool     r_rx_line_due; static uint16_t r_rx_line_n;   /* RX: status line every ~2 s, right after a frame */
static bool     r_fs_cmd_seen;              /* RX: the transmitter's store-failsafe flag, acted on once per request */
static uint32_t r_alm_until; static int32_t r_alm_level; static bool r_ever_linked;
static bool     r_pc_on, r_pc_dirty; static int16_t r_pc_req; static uint32_t r_pc_last_us;   /* TX power control */
static void r_pc_set(int16_t ddbm, uint32_t now)
{
  if(ddbm>RL_TX_POWER_DDBM) ddbm=RL_TX_POWER_DDBM;
  if(ddbm<RL_PC_MIN_DDBM)   ddbm=RL_PC_MIN_DDBM;
  if(ddbm!=r_pc_req){ r_pc_req=ddbm; r_pc_dirty=true; r_pc_last_us=now; rlink_diag[R_PC_REQ]=ddbm; }
}
static void r_pc_update(uint32_t now, int32_t lq, int32_t rssi)   /* per telemetry stats packet */
{
  if(!r_pc_on) return;
  rlink_diag[R_PC_RSSI]=rssi;
  if(lq<RL_PC_LQ_MIN || rssi<RL_PC_TARGET_DBM-6){                  /* trouble: full power now */
    if(r_pc_req!=RL_TX_POWER_DDBM){ rlink_diag[R_PC_UPS]++; r_pc_set(RL_TX_POWER_DDBM,now); }
  } else if(rssi<RL_PC_TARGET_DBM-2){
    r_pc_set((int16_t)(r_pc_req+RL_PC_STEP_DDBM),now);
  } else if(rssi>RL_PC_TARGET_DBM+4 && lq>=99 && (now-r_pc_last_us)>=RL_PC_DOWN_US && r_pc_req>RL_PC_MIN_DDBM){
    rlink_diag[R_PC_DOWNS]++; r_pc_set((int16_t)(r_pc_req-RL_PC_STEP_DDBM),now);
  }
}
/* RX telemetry */
static bool     r_tele_pending, r_tele_busy; static uint32_t r_tele_at; static uint8_t r_tele_ch;
/* RX */
static bool     r_acq;
static uint32_t r_scan_us; static uint8_t r_scan_ch;
static uint32_t r_expect_us; static uint8_t r_rx_copy; static bool r_frame_got; static uint32_t r_last_good_us;
static bool     r_rxbind; static uint32_t r_rxbind_deadline, r_rxbind_arm;
static bool     r_provisional; static uint16_t r_good;
static rx_record_t r_saved; static bool r_have_saved;
static sl_gpio_t r_bind_pin = { .port = SL_GPIO_PORT_B, .pin = 0 };
static uint32_t r_btn_since; static bool r_btn_done;
static bool     r_in_fs;
static bool     r_fs_valid; static uint16_t r_fs_us[8];

static void r_start_acq(uint32_t now)
{
  r_acq=true; r_scan_ch=0; r_scan_us=now+RL_ACQ_STEP_US; r_frame_got=false; r_rx_copy=0;
  r_listen(r_chan(0,0));
}

/* RANGE followed at any time on the transmitter (it used to be read only at power-up): low -> fixed -10 dBm
 * with power control off; back high -> full power with power control on again. Applied at the next frame
 * start while the radio is idle (r_pc_dirty). The receiver role keeps its power-up meaning (bench jam test). */
void rlink_range(bool low){
  if(r_rx_role || low==r_range_low) return;
  r_range_low=low;
  r_pc_on = RL_PC_ENABLE && !low;
  r_pc_req = low ? (int16_t)RL_RANGE_POWER_DDBM : (int16_t)RL_TX_POWER_DDBM;
  r_pc_dirty=true; r_pc_last_us=(uint32_t)sl_rail_get_time(r_rail); rlink_diag[R_PC_REQ]=r_pc_req;
}

/* BIND from the serial frames on the transmitter: the bind at power-up goes on while it is selected, so the receiver can
 * be put into bind at any time during it. Leaving BIND (or the frames stopping) ends it after the first RL_BIND_MS. */
void rlink_bind(bool held){ r_bind_hold = held; }

void rlink_start(sl_rail_handle_t rh, const volatile uint16_t *ch_us, bool rx_role, bool range_low, bool bind_held)
{
  r_rail=rh; r_ch=ch_us; r_rx_role=rx_role; r_range_low=range_low;
  for(int i=0;i<(int)RL_DIAG_N;i++) rlink_diag[i]=0;
  rlink_diag[R_ROLE]=rx_role?1:0;
  for(uint8_t i=0;i<8;i++) r_out_us[i]=1500u;
  r_provisional=false; r_good=0; r_rxbind=false; r_tx_bind=false; r_tx_state=0; r_tx_done=false; r_bind_noted=false;
  sl_rail_idle(r_rail,SL_RAIL_IDLE_ABORT,true);
  sl_rail_config_channels(r_rail,(const sl_rail_channel_config_t*)&nrf24_250k_channelConfig,NULL);
  sl_rail_prepare_channel(r_rail,RL_BIND_CH);    /* apply the PHY now, so our sync word is not overwritten later */
  sl_rail_set_fixed_length(r_rail,RL_FRAME_LEN);
  sl_rail_config_events(r_rail, SL_RAIL_EVENT_RX_PACKET_RECEIVED, SL_RAIL_EVENT_RX_PACKET_RECEIVED);
  uint32_t now=(uint32_t)sl_rail_get_time(r_rail);
  r_tele_pending=false; r_tele_busy=false; r_tele_last_us=now-RL_TELE_FRESH_US;
  r_status_due=false; r_alm_until=now; r_alm_level=0; r_ever_linked=false;
  r_vb_lvl=0; r_vb_prof=0; r_vb_low_s=r_vb_crit_s=r_vb_ok_s=0;
  r_map_reset(); r_sim_jam = rx_role && range_low;
  if(rx_role){
    sl_rail_set_tx_power_dbm(r_rail, RL_TX_POWER_DDBM);         /* RX transmits telemetry */
    sbus_init(); r_sbus_next_us=now;
    r_vb_init(now);
    servo_out_init(); servo_out_source(r_out_us);                   /* pulses: servo_out.c */
    r_have_saved = (rx_store_load(RX_SLOT_RLINK,&r_saved) && r_saved.bound);
    r_btn_since=0; r_btn_done=false; r_in_fs=false;
    r_fs_valid = r_have_saved && r_saved.fs_valid;
    if(r_fs_valid) for(uint8_t i=0;i<8;i++) r_fs_us[i]=r_saved.fs_us[i];
    sl_gpio_set_pin_mode(&r_bind_pin, SL_GPIO_MODE_INPUT_PULL, 1);
    r_last_good_us=now;
    if(bind_held){
      r_rxbind=true; r_rxbind_deadline=now+RL_BIND_TIMEOUT_US; r_rxbind_arm=now;
      if(r_have_saved) r_derive(r_saved.hop); else { uint8_t d[4]={0,0,0,0}; r_derive(d); }
      r_set_sync(RL_BIND_SYNC); r_listen(RL_BIND_CH); rlink_diag[R_LQ]=0;
    } else {
      if(r_have_saved) r_derive(r_saved.hop); else { uint8_t d[4]={0,0,0,0}; r_derive(d); }
      r_set_sync(r_data_sync);
      r_start_acq(now);
    }
  } else {
    rx_record_t txr; bool tx_priv = (rx_store_load(RX_SLOT_RLINKTX,&txr) && txr.bound);
    if(bind_held){
      r_gen_id(); r_tx_bind=true; r_tx_bind_end=now+RL_BIND_MS*1000u; r_tx_bind_next=now; r_set_sync(RL_BIND_SYNC);
      rx_record_t r; memset(&r,0,sizeof(r)); r.bound=1; rx_store_save(RX_SLOT_RLINKTX,&r);
    } else if(tx_priv){ r_gen_id(); r_set_sync(r_data_sync); }
    else { uint8_t d[4]={0,0,0,0}; r_derive(d); r_set_sync(r_data_sync); }
    sl_rail_set_tx_power_dbm(r_rail, range_low ? RL_RANGE_POWER_DDBM : RL_TX_POWER_DDBM);
    rlink_diag[R_PWR]=sl_rail_get_tx_power_dbm(r_rail);
    r_pc_on = RL_PC_ENABLE && !range_low;           /* range check keeps its fixed low power */
    sl_gpio_set_pin_mode(&r_alarm_pin,SL_GPIO_MODE_PUSH_PULL,0);
    if(RL_ALARM_TONE){ sl_gpio_set_pin_mode(&r_buzz_pin,SL_GPIO_MODE_PUSH_PULL,0); r_tone_init(); }
    r_pc_req=RL_TX_POWER_DDBM; r_pc_dirty=false; r_pc_last_us=now; rlink_diag[R_PC_REQ]=r_pc_on?r_pc_req:(range_low?RL_RANGE_POWER_DDBM:RL_TX_POWER_DDBM);
    r_base=0; r_seq=0; r_frame_us=now+RL_FRAME_US;
  }
}

void rlink_on_events(sl_rail_events_t events)
{
  if(events & SL_RAIL_EVENT_TX_PACKET_SENT){ rlink_diag[R_TXOK]++; r_tx_done=true; }
  else if(events & SL_RAIL_EVENT_TX_CHANNEL_BUSY){ rlink_diag[R_LBT_BUSY]++; r_tx_done=true; }   /* LBT: skipped */
  else if(events & SL_RAIL_EVENTS_TX_COMPLETION){ rlink_diag[R_TXER]++; r_tx_done=true; }
}

/* ------------------------------------------------------------ TX */
static void rlink_tx_action(uint32_t now)
{
  if(rxs_tx_service(now)) return;                /* RxSetup: setup packets instead of data */
  if(r_tx_bind){
    bool over = (int32_t)(now-r_tx_bind_end)>=0;
    if(over && r_bind_hold && !r_bind_noted){
      r_bind_noted=true;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("RL bind goes on while BIND is on in the frames: put the receiver into bind now\r\n");
#endif
    }
    if(over && !r_bind_hold){
      r_tx_bind=false; r_set_sync(r_data_sync);
      r_frame_us=now+RL_FRAME_US; r_tx_state=0;
#if defined(SL_CATALOG_APP_LOG_PRESENT)
      app_log("RL bind done: %ld bind packets sent, flying data now\r\n", (long)rlink_diag[R_BIND_TX]);
#endif
    } else if((int32_t)(now-r_tx_bind_next)>=0){
      r_pack_bind(); r_tx_now(RL_BIND_CH); rlink_diag[R_FTX]++; rlink_diag[R_BIND_TX]++;
      r_tx_bind_next=now+9000u;
    }
    return;
  }
  switch(r_tx_state){
    case 0:                                          /* build copy A just before its slot */
      if((int32_t)(now-(r_frame_us-RL_TX_LEAD_US))<0) return;
      r_seq=(uint8_t)((r_seq+1u)&0x3Fu); rlink_diag[R_SEQ]=r_seq;
      if(r_base==0u){                              /* map changes only at the start of a hop cycle */
        if(r_tx_announce){ r_tx_announce=false; r_map_activate(r_tx_next_ver, r_tx_nA, r_tx_nB); r_tx_mism=0; }
        else if(r_tx_next_ver!=0u && r_tx_next_ver!=r_map_ver){ r_tx_announce=true; }   /* announce for one full cycle */
        rlink_diag[R_MAP_PEND]=r_tx_announce?r_tx_next_ver:0;
      }
      if(r_pc_on && (now-r_tele_last_us)>RL_PC_LOST_US && r_pc_req!=RL_TX_POWER_DDBM){   /* telemetry gone */
        rlink_diag[R_PC_UPS]++; r_pc_set(RL_TX_POWER_DDBM,now);
      }
      if(r_pc_dirty){                              /* change power only while the radio is idle */
        r_pc_dirty=false; sl_rail_idle(r_rail,SL_RAIL_IDLE,true);
        sl_rail_set_tx_power_dbm(r_rail,(sl_rail_tx_power_t)r_pc_req);
        rlink_diag[R_PWR]=sl_rail_get_tx_power_dbm(r_rail);
      }
      r_pack(fs_cmd_active(now));                  /* encoder failsafe frame: the receiver stores the positions in these frames */
      r_tx_done=false;
      r_tx_state=1;
      if(!r_tx_at(r_chan(r_base,0), r_frame_us)) r_tx_done=true;   /* A failed: still send B */
      rlink_diag[R_FTX]++;
      break;
    case 1:                                          /* A done -> schedule B at A + COPY_US */
      if(!r_tx_done){
        if((int32_t)(now-(r_frame_us+RL_COPY_US))>0){ r_tx_done=true; }   /* lost event guard */
        return;
      }
      r_tx_done=false;
      if(!r_tx_at(r_chan(r_base,1), r_frame_us+RL_COPY_US)) r_tx_done=true;
      r_tx_state=2;
      break;
    default:                                         /* B done -> next frame on the grid */
      if(!r_tx_done && (int32_t)(now-(r_frame_us+2u*RL_COPY_US))<=0) return;
      r_tx_done=false;
      r_status_due = ((uint8_t)(r_seq % RL_UART_EVERY)==1u);   /* print in this frame's quiet time */
      if((r_seq & (RL_TELE_EVERY-1u))==0u){         /* telemetry frame: listen until the next copy A is loaded */
        r_listen(r_chan(r_base, (uint8_t)((r_seq/RL_TELE_EVERY)&1u)));
        rlink_diag[R_T_ASK]++;
      }
      r_base=(uint8_t)((r_base+1u)%RL_CHANNELS);
      r_frame_us+=RL_FRAME_US;
      while((int32_t)(r_frame_us-RL_TX_LEAD_US-now)<0){ r_frame_us+=RL_FRAME_US; r_base=(uint8_t)((r_base+1u)%RL_CHANNELS); rlink_diag[R_LATE]++; }
      r_tx_state=0;
      break;
  }
}

/* ------------------------------------------------------------ RX */
static void r_next_frame(uint32_t now, bool got)
{
  r_win_frames++;
  if(!got){
    if(r_rx_copy==1u) r_stat(r_cur_ch,true);       /* copy B lost on this channel */
    if(RL_SBUS_PERIOD_US==0u && sbus_send(r_out_us,true,r_in_fs)) rlink_diag[R_SBUS]++;
    rlink_diag[R_FLOST]++; r_win_lost++;
    if(!r_in_dropout){ r_in_dropout=true; r_dropout_start_us=now; }
  } else if(r_in_dropout){
    uint32_t d=(now-r_dropout_start_us)/1000u; if(d>r_worst_ms){ r_worst_ms=d; rlink_diag[R_WORST]=(int32_t)r_worst_ms; }
    r_in_dropout=false;
  }
  r_base=(uint8_t)((r_base+1u)%RL_CHANNELS);
  r_expect_us += RL_FRAME_US; r_rx_copy=0; r_frame_got=false;
  r_rx_base_step(); r_frame_tick(now);
  r_listen(r_chan(r_base,0));
  if(r_win_frames>=RL_LQ_WINDOW){ uint32_t lq=100u-(uint32_t)r_win_lost*100u/r_win_frames; rlink_diag[R_LQ]=(int32_t)lq; r_win_frames=0; r_win_lost=0; }
}

static void r_store_failsafe(void)                /* RX: the current positions become the failsafe preset, saved */
{
  for(uint8_t i=0;i<8;i++) r_fs_us[i]=r_out_us[i];
  r_fs_valid=true;
  rx_record_t r; if(!(rx_store_load(RX_SLOT_RLINK,&r) && r.bound)){ memset(&r,0,sizeof(r)); memcpy(r.hop,r_id,4); r.bound=1; }
  for(uint8_t i=0;i<8;i++) r.fs_us[i]=r_fs_us[i];
  r.fs_valid=1;
  if(rx_store_save(RX_SLOT_RLINK,&r)){ r_saved=r; r_have_saved=true; rlink_diag[R_WORST]=7777; }
#if defined(SL_CATALOG_APP_LOG_PRESENT)
  app_log("RL failsafe stored: %u %u %u %u %u %u %u %u\r\n", r_fs_us[0],r_fs_us[1],r_fs_us[2],r_fs_us[3],r_fs_us[4],r_fs_us[5],r_fs_us[6],r_fs_us[7]);
#endif
}

static void r_apply_failsafe(void)
{
  if(r_in_fs) return;
  r_in_fs=true;
  if(r_fs_valid){ for(uint8_t i=0;i<8;i++) r_out_us[i]=r_fs_us[i]; r_ppm_enable(true); }
  else if(RL_FS_NOPULSES){ r_ppm_enable(false); }
}

static void rlink_rx_action(uint32_t now)
{
  { uint8_t sl = rxs_rx_slot(now, r_rxbind || r_acq);   /* searching: a short look for setup packets every 100 ms */
    if(sl==1u) return;
    if(sl==2u){
      if(r_rxbind){ r_set_sync(RL_BIND_SYNC); r_listen(RL_BIND_CH); }
      else { r_set_sync(r_data_sync); r_listen(r_acq ? r_chan(r_scan_ch,0) : r_chan(r_base,r_rx_copy)); }
    } }
  if(r_rxbind){
    if((now-r_rxbind_arm)>100000u){ r_rxbind_arm=now; r_set_sync(RL_BIND_SYNC); r_listen(RL_BIND_CH); }
    if((int32_t)(now-r_rxbind_deadline)>=0){
      r_rxbind=false;
      if(r_have_saved){ r_derive(r_saved.hop); r_set_sync(r_data_sync); r_last_good_us=now; r_start_acq(now); }
      else { r_rxbind=true; r_rxbind_deadline=now+RL_BIND_TIMEOUT_US; }
    }
    return;
  }
  if(r_acq){
    if((int32_t)(now-r_scan_us)>=0){ r_scan_ch=(uint8_t)((r_scan_ch+1u)%RL_CHANNELS); r_listen(r_chan(r_scan_ch,0)); r_scan_us=now+RL_ACQ_STEP_US; }
    if((now-r_last_good_us) > RL_FAILSAFE_MS*1000u) r_apply_failsafe();
    return;
  }
  /* telemetry reply: load + schedule it, then go back to listening */
  if(r_tele_pending && (int32_t)(now-(r_tele_at-RL_TELE_LEAD_US))>=0){
    r_tele_pending=false; r_tx_done=false;
    if(r_tx_at(r_tele_ch, r_tele_at)){ r_tele_busy=true; rlink_diag[R_T_ASK]++; }
    else r_listen(r_chan(r_base,r_rx_copy));
  }
  if(r_tele_busy && (r_tx_done || (int32_t)(now-(r_tele_at+2000u))>=0)){
    r_tele_busy=false; r_listen(r_chan(r_base,r_rx_copy));
  }
  /* copy A missed so far: move to copy B's channel before B is sent */
  if(r_tele_busy) return;
  if(r_rx_copy==0u && (int32_t)(now-(r_expect_us+RL_COPY_US-RL_LEAD_US))>=0){
    r_stat(r_cur_ch,true);                         /* copy A lost on this channel */
    r_rx_copy=1; r_listen(r_chan(r_base,1));
  }
  uint32_t miss_at = r_expect_us + RL_COPY_US + RL_MISS_PAD_US;
  if((int32_t)(now-miss_at)>=0){
    r_next_frame(now,false);
    if((now-r_last_good_us) > RL_FAILSAFE_MS*1000u){ r_apply_failsafe();
      if((now-r_last_good_us) > 3u*RL_FAILSAFE_MS*1000u) r_start_acq(now); }
  }
}

void rlink_action(void)
{
  uint32_t now=(uint32_t)sl_rail_get_time(r_rail);
  if(r_rx_role) rlink_rx_action(now); else rlink_tx_action(now);
  { sl_rail_sync_word_config_t cur; if(sl_rail_get_sync_words(r_rail,&cur)==SL_RAIL_STATUS_NO_ERROR) rlink_diag[R_SYNC_NOW]=(int32_t)cur.sync_word_0;
    bool bv=true; sl_gpio_get_pin_input(&r_bind_pin,&bv);
    rlink_diag[R_STATE]=(r_rxbind?1:0)|(r_acq?2:0)|(r_have_saved?4:0)|(r_provisional?8:0)|(r_tx_bind?16:0)|(bv?0:32)|(r_lbt_refused?64:0); }
  if(r_rx_role){
    if(RL_SBUS_PERIOD_US!=0u && (int32_t)(now-r_sbus_next_us)>=0){
      r_sbus_next_us = now + RL_SBUS_PERIOD_US;
      if(sbus_send(r_out_us,r_in_dropout,r_in_fs)) rlink_diag[R_SBUS]++;
    }
    sbus_service();
    r_vb_service(now);
#if RL_UART_STATUS && defined(SL_CATALOG_APP_LOG_PRESENT)
    { static uint32_t line_at=0;                      /* linked: ~45 chars = 4 ms after copy A, before the telemetry slot; else every 3 s */
    if(r_rx_line_due || (now-line_at)>=3000000u){
      r_rx_line_due=false; line_at=now;
      long vb=(long)rlink_diag[R_VBAT];
      app_log("RL rx %s q%ld a%ld b%ld lost%ld worst%ld fs%d m%ld v%ld.%02ld\r\n",
              r_rxbind ? "bind" : r_acq ? "search" : "link",
              (long)rlink_diag[R_LQ], (long)rlink_diag[R_RSSI], (long)rlink_diag[R_RSSI_B], (long)rlink_diag[R_FLOST],
              (long)rlink_diag[R_WORST], r_in_fs ? 1 : 0, (long)rlink_diag[R_MAP_NA], vb/1000, (vb%1000)/10);
    } }
#endif
  }
  if(r_rx_role){
    /* hold BIND >0.3 s while linked -> store current positions as failsafe preset */
    bool bv=true; sl_gpio_get_pin_input(&r_bind_pin,&bv); bool bl=!bv;
    if(!bl || r_rxbind || r_acq){ r_btn_since=0; r_btn_done=false; }
    else if(r_btn_since==0){ r_btn_since = now?now:1u; }
    else if(!r_btn_done && (now-r_btn_since)>=RL_FS_PRESS_US){
      r_btn_done=true;
      r_store_failsafe();
    }
    int32_t lq=rlink_diag[R_LQ];
    static uint32_t tb=0; uint32_t period=(lq>=95)?0u:(lq>=70)?400000u:(lq>=40)?150000u:60000u;
    if(r_acq){ if((now-tb)>500000u){ tb=now; sl_gpio_toggle_pin(&r_led_b);} }
    else if(period==0u) sl_gpio_set_pin(&r_led_b);
    else if((now-tb)>period){ tb=now; sl_gpio_toggle_pin(&r_led_b); }
  } else {
    /* ---- alarms: 2 = critical (no telemetry / RX in failsafe), 1 = warning (weak link) */
    uint32_t age = now - r_tele_last_us;
    int32_t lvl = 0;
    rlink_diag[R_INLOST] = r_in_ok ? 0 : 1;
    if(age > RL_ALM_TELE_US || (rlink_diag[R_T_FLAGS] & 1) || !r_in_ok) lvl = 2;
    else if(rlink_diag[R_T_LQ] < RL_ALM_LQ_WARN ||
            ((!r_pc_on || r_pc_req>=RL_TX_POWER_DDBM) && rlink_diag[R_T_RSSIA] < RL_ALM_RSSI_WARN)) lvl = 1;
    if(lvl >= r_alm_level){ if(lvl>0) r_alm_until = now + RL_ALM_HOLD_US; r_alm_level = lvl; }
    else if((int32_t)(now - r_alm_until) >= 0) r_alm_level = lvl;          /* drop only after the hold time */
    rlink_diag[R_ALARM] = r_alm_level;
    if(age <= RL_ALM_TELE_US) r_ever_linked = true;
    r_vb_alarm(now, age <= RL_ALM_TELE_US);
    rlink_diag[R_VB_ALM] = r_vb_lvl;
    int32_t all = r_alm_level > r_vb_lvl ? r_alm_level : r_vb_lvl;
#if RL_ALARM_TONE
    { /* beep patterns (ms within the 1 s period):
       *   weak link (warning)    : two 80 ms beeps every 1 s
       *   RX battery low         : three 80 ms beeps every 1 s (both warnings: alternate each second)
       *   land now               : 150 on / 150 off, continuous - link critical, RX failsafe or RX battery critical
       *   no RX yet since boot   : one 40 ms chirp every 2 s (bench-friendly "no receiver") */
      uint32_t ms = now/1000u, t = ms%1000u; bool on=false;
      if(all==2){ if(r_ever_linked) on = ((ms%300u)<150u); else on = ((ms%2000u)<40u); }
      else if(all==1){
        uint32_t beeps = (r_alm_level==1 && r_vb_lvl==1) ? (((ms/1000u)&1u) ? 3u : 2u) : (r_vb_lvl==1 ? 3u : 2u);
        on = (t < beeps*160u) && ((t%160u) < 80u);
      }
      r_tone(on);
    }
#endif
    if(all) sl_gpio_set_pin(&r_alarm_pin); else sl_gpio_clear_pin(&r_alarm_pin);
    /* LED_A: steady = link fine, 2 Hz blink = warning, flicker per frame = no telemetry */
    if(r_alm_level==2){ static int32_t la=0; if(rlink_diag[R_FTX]!=la){ la=rlink_diag[R_FTX]; sl_gpio_toggle_pin(&r_led_a); } }
    else if(all>=1){ if(((now/250000u)&1u)) sl_gpio_set_pin(&r_led_a); else sl_gpio_clear_pin(&r_led_a); }
    else sl_gpio_set_pin(&r_led_a);
#if RL_UART_STATUS && defined(SL_CATALOG_APP_LOG_PRESENT)
    if(r_status_due){                               /* ~35 chars = ~3 ms at 115200, in the frame's quiet part */
      r_status_due=false;
      static int32_t lbt_prev=0; int32_t lbt_now=rlink_diag[R_LBT_BUSY];
      long vb=(long)rlink_diag[R_VBAT];
      app_log("RL q%ld a%ld b%ld u%ld p%ld m%ld s%ld i%d l%ld v%ld.%02ld\r\n",
              (long)(age>RL_ALM_TELE_US ? 0 : rlink_diag[R_T_LQ]), (long)rlink_diag[R_T_RSSIA], (long)rlink_diag[R_T_RSSIB],
              (long)rlink_diag[R_T_RSSI_UP], (long)(rlink_diag[R_PWR]/10), (long)rlink_diag[R_MAP_NA], (long)all, r_in_ok ? 1 : 0, (long)(lbt_now-lbt_prev), vb/1000, (vb%1000)/10);
      lbt_prev=lbt_now;
    }
#else
    r_status_due=false;
#endif
  }
}

void rlink_rx_process(void)
{
  sl_rail_rx_packet_info_t info;
  sl_rail_rx_packet_handle_t h=sl_rail_get_rx_packet_info(r_rail,SL_RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE,&info);
  if(h==SL_RAIL_RX_PACKET_HANDLE_INVALID) return;
  uint16_t n=info.packet_bytes; if(n>sizeof(r_rx_buf)) n=sizeof(r_rx_buf);
  sl_rail_copy_rx_packet(r_rail,r_rx_buf,&info);
  sl_rail_rx_packet_details_t det; int8_t rssi=-128;
  if(sl_rail_get_rx_packet_details(r_rail,h,&det)==SL_RAIL_STATUS_NO_ERROR) rssi=det.rssi_dbm;
  sl_rail_release_rx_packet(r_rail,h);
  bool crc_ok = (n==RL_FRAME_LEN) && r_crc(r_rx_buf,RL_PAYLOAD)==(uint16_t)((r_rx_buf[RL_PAYLOAD]<<8)|r_rx_buf[RL_PAYLOAD+1u]);
  uint32_t now=(uint32_t)sl_rail_get_time(r_rail);

  if(!r_rx_role){                                         /* TX: only telemetry from our RX matters */
    if(!crc_ok || !(r_rx_buf[0] & RL_HDR_TELE)) return;
    rlink_diag[R_T_GOT]++; r_tele_last_us=now;
    if(r_rx_buf[13]==1u){                                  /* map proposal from the RX */
      uint8_t v=(uint8_t)(r_rx_buf[11]&0x0Fu);
      if(!r_tx_announce && v!=0u && v!=r_map_ver){ r_tx_next_ver=v; r_tx_nA=r_get39(&r_rx_buf[1]); r_tx_nB=r_get39(&r_rx_buf[6]); }
      rlink_diag[R_T_MAPS]++;
      return;
    }
    { uint8_t rv=(uint8_t)(r_rx_buf[12]&0x0Fu);            /* RX's active map must follow ours */
      if(rv!=r_map_ver && !r_tx_announce){ if(++r_tx_mism>=RL_MAP_MISMATCH){ r_map_reset(); rlink_diag[R_MAP_RST]++; } }
      else r_tx_mism=0; }
    rlink_diag[R_T_LQ]    = r_rx_buf[1];
    rlink_diag[R_T_RSSIA] = (int8_t)r_rx_buf[2];
    rlink_diag[R_T_RSSIB] = (int8_t)r_rx_buf[3];
    rlink_diag[R_T_SAVEDB]= (int32_t)(r_rx_buf[4] | (r_rx_buf[5]<<8));
    rlink_diag[R_T_LOST]  = (int32_t)(r_rx_buf[6] | (r_rx_buf[7]<<8));
    rlink_diag[R_T_CRCB]  = (int32_t)(r_rx_buf[8] | (r_rx_buf[9]<<8));
    rlink_diag[R_T_FLAGS] = (int32_t)((r_rx_buf[11]&0x03u) | (r_rx_buf[10]<<8));   /* worst dropout/10 ms in bits 8..15 */
    rlink_diag[R_VBAT]    = (int32_t)(((r_rx_buf[11]>>2)&0x3Fu) | ((uint32_t)(r_rx_buf[12]>>4)<<6)) * 10;
    rlink_diag[R_T_RSSI_UP]= rssi;                        /* how strong the RX is heard at the TX */
    r_pc_update(now, rlink_diag[R_T_LQ], rlink_diag[R_T_RSSIA]);
    return;
  }
  if(rxs_rx_listening()){ rxs_rx_packet(r_rx_buf, n, rssi); return; }   /* the setup slot: only setup packets */
  if(!crc_ok){ rlink_diag[R_CRCB]++; return; }
  if(r_rx_buf[0] & RL_HDR_TELE) return;                   /* another RX's telemetry: not for us */
  rlink_diag[R_CRX]++;

  if(r_rx_buf[0] & 0x01u){                                /* bind advertise */
    rlink_diag[R_BIND_SEEN]++;
    if(r_rxbind){
      uint8_t nid[4]={ r_rx_buf[1],r_rx_buf[2],r_rx_buf[3],r_rx_buf[4] };
      r_derive(nid); r_set_sync(r_data_sync);
      r_rxbind=false; r_provisional=true; r_good=0; r_rxbind_deadline=now+RL_BIND_TIMEOUT_US;
      r_last_good_us=now; r_start_acq(now);
    }
    return;
  }
  if(r_sim_jam && r_cur_ch>=RL_SIM_LO && r_cur_ch<=RL_SIM_HI) return;   /* bench test: act as if jammed */
  if(r_frame_got) return;
  bool tx_in_lost = (r_rx_buf[1] & 0x80u) != 0u;
  bool tx_fs_cmd  = (r_rx_buf[1] & 0x40u) != 0u;         /* the transmitter asks us to store these positions as failsafe */
  uint8_t base=(uint8_t)(r_rx_buf[1] & 0x3Fu);
  if(base>=RL_CHANNELS) { rlink_diag[R_CRCB]++; return; }
  rlink_diag[R_FRX]++; r_last_good_us=now;

  bool was_acq = r_acq;
  if(!was_acq) r_stat(r_cur_ch,false);                    /* the copy we caught arrived fine */
  uint8_t copy = was_acq ? 0u : r_rx_copy;                /* acquisition only listens on A channels */
  if(copy==0u){ rlink_diag[R_GOT_A]++; rlink_diag[R_RSSI]=rssi; } else { rlink_diag[R_SAVED_B]++; rlink_diag[R_RSSI_B]=rssi; }
  uint32_t copy0 = now - (uint32_t)copy*RL_COPY_US;
  r_acq=false;

  if(tx_in_lost){                                          /* TX has no stick input: failsafe now */
    rlink_diag[R_INLOST]++;
    r_apply_failsafe();
  } else {
    r_in_fs=false;
    uint32_t acc=0; uint8_t nb=0, o=3;
    for(uint8_t i=0;i<8;i++){ while(nb<11){ acc|=(uint32_t)r_rx_buf[o++]<<nb; nb+=8; } uint16_t v=(uint16_t)(acc&0x7FFu); acc>>=11; nb-=11; r_out_us[i]=r_from11(v); }
    rlink_diag[R_CH0]=r_out_us[0]; rlink_diag[R_CH1]=r_out_us[1];
    r_last_rx=now;
    r_ppm_enable(true);
    if(tx_fs_cmd){ if(!r_fs_cmd_seen){ r_fs_cmd_seen=true; r_store_failsafe(); } }   /* once per request from the transmitter */
    else r_fs_cmd_seen=false;
  }
  if(++r_rx_line_n>=180u){ r_rx_line_n=0; r_rx_line_due=true; }   /* ~2 s; printed in this frame's quiet part */
  if(RL_SBUS_PERIOD_US==0u && sbus_send(r_out_us,false,r_in_fs)) rlink_diag[R_SBUS]++;
  sl_gpio_toggle_pin(&r_led_a);

  if(r_provisional && ++r_good >= RL_COMMIT_PKTS){
    r_provisional=false;
    rx_record_t r; memset(&r,0,sizeof(r)); memcpy(r.hop,r_id,4); r.bound=1;
    if(r_fs_valid){ r.fs_valid=1; for(uint8_t i=0;i<8;i++) r.fs_us[i]=r_fs_us[i]; }
    if(rx_store_save(RX_SLOT_RLINK,&r)){ r_saved=r; r_have_saved=true; }
  }

  /* account this frame, hop to the next immediately (hop-on-receive) */
  r_win_frames++;
  if(r_in_dropout){ uint32_t d=(now-r_dropout_start_us)/1000u; if(d>r_worst_ms){ r_worst_ms=d; rlink_diag[R_WORST]=(int32_t)r_worst_ms; } r_in_dropout=false; }
  if(r_win_frames>=RL_LQ_WINDOW){ uint32_t lq=100u-(uint32_t)r_win_lost*100u/r_win_frames; rlink_diag[R_LQ]=(int32_t)lq; r_win_frames=0; r_win_lost=0; }
  /* map agreement: TX's active map rules; an announced map equal to our proposal arms the switch */
  { uint8_t fl=r_rx_buf[2], tact=(uint8_t)(fl>>4), tpend=(uint8_t)(fl&0x0Fu);
    if(tact!=r_map_ver){
      if(tact==0u){ r_map_reset(); rlink_diag[R_MAP_RST]++; }
      else if(tact==r_prop_ver){ r_map_armed=false; r_map_activate(r_prop_ver, r_pA, r_pB); }
    }
    if(tpend!=0u && tpend==r_prop_ver && r_prop_ver!=r_map_ver) r_map_armed=true;
  }

  r_base=(uint8_t)((base+1u)%RL_CHANNELS);
  r_expect_us=copy0+RL_FRAME_US;
  r_rx_copy=0; r_frame_got=false;
  if(!was_acq) r_frame_tick(now);

  /* telemetry frame: answer RL_TELE_US after this frame's copy A (channel from the current map) */
  uint8_t fseq=(uint8_t)(r_rx_buf[0]>>2);
  bool send_map = (r_prop_ver!=r_map_ver) && (((fseq/RL_TELE_EVERY)&1u)!=0u);   /* alternate stats / proposal */
  if((fseq & (RL_TELE_EVERY-1u))==0u && send_map){
    memset(r_pkt,0,RL_FRAME_LEN);
    r_pkt[0]=(uint8_t)(RL_HDR_TELE | (fseq<<2));
    r_put39(&r_pkt[1], r_pA); r_put39(&r_pkt[6], r_pB);
    r_pkt[11]=r_prop_ver; r_pkt[12]=r_map_ver; r_pkt[13]=1u;
    uint16_t c=r_crc(r_pkt,RL_PAYLOAD); r_pkt[RL_PAYLOAD]=(uint8_t)(c>>8); r_pkt[RL_PAYLOAD+1u]=(uint8_t)c;
    r_tele_at=copy0+RL_TELE_US; r_tele_ch=r_chan(base,(uint8_t)((fseq/RL_TELE_EVERY)&1u));
    r_tele_pending=true;
  } else if((fseq & (RL_TELE_EVERY-1u))==0u){
    int32_t lq=rlink_diag[R_LQ]; uint32_t w=(uint32_t)rlink_diag[R_WORST]/10u; if(w>255u) w=255u;
    uint16_t sb=(uint16_t)rlink_diag[R_SAVED_B], lo=(uint16_t)rlink_diag[R_FLOST], cb=(uint16_t)rlink_diag[R_CRCB];
    memset(r_pkt,0,RL_FRAME_LEN);
    r_pkt[0]=(uint8_t)(RL_HDR_TELE | (fseq<<2));
    r_pkt[1]=(uint8_t)(lq<0?0:lq>100?100:lq);
    r_pkt[2]=(uint8_t)(int8_t)rlink_diag[R_RSSI];
    r_pkt[3]=(uint8_t)(int8_t)rlink_diag[R_RSSI_B];
    r_pkt[4]=(uint8_t)sb; r_pkt[5]=(uint8_t)(sb>>8);
    r_pkt[6]=(uint8_t)lo; r_pkt[7]=(uint8_t)(lo>>8);
    r_pkt[8]=(uint8_t)cb; r_pkt[9]=(uint8_t)(cb>>8);
    r_pkt[10]=(uint8_t)w;
    { uint16_t v10 = (uint16_t)(r_vb_mv/10u);                      /* battery, 10 mV, 0 = no sensor */
      r_pkt[11]=(uint8_t)((r_in_fs?1u:0u)|(r_provisional?2u:0u)|((v10&0x3Fu)<<2));
      r_pkt[12]=(uint8_t)((r_map_ver&0x0Fu)|((v10>>6)<<4)); }      /* map: lets the TX see that we follow it */
    uint16_t c=r_crc(r_pkt,RL_PAYLOAD); r_pkt[RL_PAYLOAD]=(uint8_t)(c>>8); r_pkt[RL_PAYLOAD+1u]=(uint8_t)c;
    r_tele_at=copy0+RL_TELE_US; r_tele_ch=r_chan(base,(uint8_t)((fseq/RL_TELE_EVERY)&1u));
    r_tele_pending=true;
  }

  r_rx_base_step();                                       /* map switch happens here, at base 0 */
  r_listen(r_chan(r_base,0));
}

uint32_t rlink_tele_age_us(uint32_t now) { return now - r_tele_last_us; }
bool     rlink_ever_linked(void)         { return r_ever_linked; }
