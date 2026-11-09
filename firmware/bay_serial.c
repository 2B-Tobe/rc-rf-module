/* bay_serial.c — see bay_serial.h.  #included into app_process.c.   v1.1: 27-byte frame, protocols 64..255
 *                                                                     v1.5: either polarity (EdgeTX), TX interrupt */
#include "bay_serial.h"
#include "etx_multi.h"   /* etx_translate: the EdgeTX protocol numbers */
#include "em_eusart.h"
#include "em_cmu.h"
#include "em_device.h"
#include "sl_gpio.h"
#include <string.h>

#define BAY_BAUD        100000u
#define BAY_LEN         26u      /* the classic frame (protocols 0..63)                                             */
#define BAY_LEN_V2      27u      /* protocols 64 and up: one more byte                                              */
#define BAY_V2_WAIT_US  200u     /* after the 26th byte: a 27th arrives 110 us later (11 bits at 100000 baud)       */
#define BAY_GAP_US      3000u    /* frames are 3.1 ms long, 22 ms apart: more than 3 ms of silence starts a new frame */
#define BAY_GAP_ETX_US  1000u    /* EdgeTX: 3 to 4 ms frames every 7 ms, sent without pauses: 1 ms of silence is enough */
#define BAY_IRQ_PRIO    4u       /* below TIMER0 (PPM capture, 2); the 16-byte RX FIFO covers 1.9 ms of latency   */
#define BAY_RX_PORT     SL_GPIO_PORT_A
#define BAY_RX_PIN      6u
#define BAY_TX_PORT     SL_GPIO_PORT_B   /* telemetry out: the PPM input pin, idle in encoder mode */
#define BAY_TX_PIN      1u
#define BAY_TELEM_LEN   16u

volatile int32_t bay_diag[BAY_DIAG_N];

static sl_rail_handle_t b_rail;
static void (*b_cb)(const bay_frame_t *f);
static uint8_t  b_buf[BAY_LEN_V2];
static uint8_t  b_n;             /* bytes of the frame in progress                                 */
static bool     b_done;          /* frame complete (or refused): ignore bytes until the next gap   */
static bool     b_err;           /* a parity / framing / overrun error in the frame in progress    */
static uint32_t b_last_us;       /* last byte                                                      */
static uint32_t b_seq;
static bay_frame_t b_last;       /* latest good frame (written in the ISR)                        */
static bool     b_inv;           /* RX inverted (EdgeTX)                                            */
static bool     b_txinv;         /* telemetry inverted                                              */
static uint32_t b_prev_ok_us;    /* the previous good frame                                         */
static uint8_t  b_fast;          /* good frames in a row under BAY_FAST_US apart (EdgeTX: 7 ms)      */
static uint8_t  b_settle;        /* good frames still held back after a silence                    */
#define BAY_FAST_US    12000u    /* EdgeTX every 7 ms (a lost frame: 14), the encoder every 22 ms    */
#define BAY_SLOW_US    15000u
#define BAY_BURST_US   60000u    /* longer than this between good frames: a new sender, maybe        */

/* Encoder: v = (ticks - 2000) * 82 / 100 + 204 with ticks in 0.5 us, so us = 1000 + (v - 204) * 25 / 41. Rounded. */
static uint16_t bay_us(uint16_t v)
{
  int32_t d = (int32_t)v - 204;
  int32_t q = (d >= 0) ? (d * 50 + 41) / 82 : -((-d * 50 + 41) / 82);
  return (uint16_t)(1000 + q);
}

void bay_decode(const uint8_t *b, uint8_t n, bool etx, bay_frame_t *f)
{
  f->proto    = (uint8_t)((b[1] & 0x1Fu) | ((b[0] & 0x01u) ? 0u : 0x20u));
  f->failsafe = (b[0] & 0x02u) != 0u;                            /* 0x57 / 0x56: the Multiprotocol failsafe frame */
  if (n >= BAY_LEN_V2) f->proto |= (uint8_t)(b[26] & 0xC0u);     /* V2 frame: protocol bits 6..7 */
  f->range    = (b[1] & 0x20u) != 0u;
  f->autobind = (b[1] & 0x40u) != 0u;
  f->bind     = (b[1] & 0x80u) != 0u;
  f->rxnum    = (uint8_t)(b[2] & 0x0Fu);
  f->sub      = (uint8_t)((b[2] >> 4) & 0x07u);
  f->lowpower = (b[2] & 0x80u) != 0u;
  f->option   = (int8_t)b[3];
  f->edgetx   = etx;
  f->wproto   = f->proto;
  f->wsub     = f->sub;
  f->scan     = false;
  f->scan_idx = 0;
  f->tel_inv  = false;
  if (etx) {                                                     /* EdgeTX: byte 26 as in Multiprotocol */
    uint8_t b26 = (n >= BAY_LEN_V2) ? b[26] : 0u;
    f->telem    = (b26 & 0x02u) == 0u;                           /* bit 1: the model has telemetry off */
    f->tel_inv  = (b26 & 0x08u) != 0u;
    f->rxnum    = (uint8_t)(f->rxnum | (b26 & 0x30u));           /* RX number 0..63 */
    f->rxsetup  = false;
    f->rxlayout = 0;
    if (f->wproto == 0u) { f->scan = true; f->scan_idx = b[3]; }
    etx_translate(f);                                            /* Multiprotocol numbers -> this module's (etx_multi.c) */
  } else {
    f->telem    = (n >= BAY_LEN_V2) && (b[26] & 0x04u) != 0u;    /* ... and bit 2: telemetry wanted on the PPM pin */
    f->rxsetup  = (n >= BAY_LEN_V2) && (b[26] & 0x08u) != 0u;    /* ... bit 3: RxSetup, layout in bits 0..1 */
    f->rxlayout = f->rxsetup ? (uint8_t)(b[26] & 0x03u) : 0u;
  }
  uint32_t acc = 0; uint8_t nb = 0, k = 4;
  for (uint8_t i = 0; i < 16u; i++) {
    while (nb < 11u) { acc |= (uint32_t)b[k++] << nb; nb = (uint8_t)(nb + 8u); }
    f->us[i] = bay_us((uint16_t)(acc & 0x7FFu));
    acc >>= 11; nb = (uint8_t)(nb - 11u);
  }
}

void EUSART1_RX_IRQHandler(void)
{
  uint32_t fl = EUSART1->IF;
  EUSART1->IF_CLR = fl;                                        /* clear first: a byte arriving later sets it again */
  bool err = (fl & (EUSART_IF_PERR | EUSART_IF_FERR | EUSART_IF_RXOF)) != 0u;
  if (fl & EUSART_IF_RXOF) bay_diag[BAY_OVF]++;
  if (err) b_err = true;
  uint32_t now = (uint32_t)sl_rail_get_time(b_rail);
  while (EUSART1->STATUS & EUSART_STATUS_RXFL) {
    uint8_t c = (uint8_t)EUSART1->RXDATA;
    bay_diag[BAY_BYTES]++;
    if ((uint32_t)(now - b_last_us) > ((b_inv || b_fast >= 2u) ? BAY_GAP_ETX_US : BAY_GAP_US)) {   /* silence before this byte: a new frame */
      if (b_n != 0u && !b_done) bay_diag[BAY_SHORT]++;
      b_n = 0; b_done = false; b_err = err;                       /* an error in this batch may be the new frame's */
    }
    b_last_us = now;
    if (b_done) { bay_diag[BAY_EXTRA]++; continue; }
    if (b_n == 0u && (c & 0xFCu) != 0x54u) {                    /* not 0x55 / 0x54 (nor a failsafe frame 0x57 / 0x56) */
      bay_diag[BAY_HDR]++; bay_diag[BAY_LASTHDR] = c; b_done = true; continue;
    }
    b_buf[b_n++] = c;
    if (b_n < BAY_LEN) continue;
    if (b_n == BAY_LEN) {                                          /* 26 bytes: complete, unless a 27th is on its way */
      uint32_t t0 = (uint32_t)sl_rail_get_time(b_rail);
      while (!(EUSART1->STATUS & EUSART_STATUS_RXFL)
             && ((uint32_t)sl_rail_get_time(b_rail) - t0) < BAY_V2_WAIT_US) { }
      if (EUSART1->STATUS & EUSART_STATUS_RXFL) {
        uint32_t f2 = EUSART1->IF;                                 /* the 27th byte's flags (an error there counts) */
        EUSART1->IF_CLR = f2;
        if (f2 & (EUSART_IF_PERR | EUSART_IF_FERR | EUSART_IF_RXOF)) b_err = true;
        now = (uint32_t)sl_rail_get_time(b_rail);
        continue;                                                  /* the loop reads it as byte 27 */
      }
    }
    b_done = true;
    if (b_err) { bay_diag[BAY_ERR]++; continue; }
    bay_diag[BAY_GOOD]++;
    uint32_t dt = now - b_prev_ok_us;
    b_prev_ok_us = now;
    if (dt > BAY_BURST_US) { b_fast = 0; b_settle = 2; }          /* after a silence: the rate first */
    else if (dt < BAY_FAST_US) { if (b_fast < 3u) b_fast++; }
    else if (dt > BAY_SLOW_US) b_fast = 0;
    if (b_settle) { b_settle--; continue; }
    bay_decode(b_buf, b_n, b_inv || b_fast >= 2u, &b_last);
    b_last.t_us = now;
    b_last.seq  = ++b_seq;
    bay_diag[BAY_OK]++;
    if (b_cb) b_cb(&b_last);
  }
}

void bay_init(sl_rail_handle_t rh, void (*on_frame)(const bay_frame_t *f))
{
  b_rail = rh; b_cb = on_frame;
  b_n = 0; b_done = true; b_err = false; b_seq = 0;
  b_inv = false; b_txinv = false;                                 /* normal (encoder) first; bay_set_inverted flips it */
  b_fast = 0; b_settle = 2; b_prev_ok_us = 0;                     /* the first good frame starts a burst */
  b_last_us = (uint32_t)sl_rail_get_time(rh);                    /* a frame already under way is skipped */
  memset((void *)bay_diag, 0, sizeof(bay_diag));
  memset(&b_last, 0, sizeof(b_last));

  sl_gpio_t rx = { .port = BAY_RX_PORT, .pin = BAY_RX_PIN };
  sl_gpio_set_pin_mode(&rx, SL_GPIO_MODE_INPUT_PULL, 1);        /* pull-up: idle high with nothing connected */
  /* EUSART0 (the log UART and its command line) keeps listening on PA06 until the frames are detected, so that
   * "cfg" can be typed into a transmitter waiting for its encoder; app_process.c disables it after bay_detect. */

  CMU_ClockEnable(cmuClock_EUSART1, true);
  EUSART_UartInit_TypeDef init = EUSART_UART_INIT_DEFAULT_HF;
  init.baudrate = BAY_BAUD;
  init.databits = eusartDataBits8;
  init.parity   = eusartEvenParity;
  init.stopbits = eusartStopbits2;
  init.enable   = eusartEnableRx;
  EUSART_UartInitHf(EUSART1, &init);
  GPIO->EUSARTROUTE[EUSART_NUM(EUSART1)].RXROUTE =
      ((uint32_t)BAY_RX_PORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
    | ((uint32_t)BAY_RX_PIN  << _GPIO_EUSART_RXROUTE_PIN_SHIFT);   /* an input needs no ROUTEEN bit */

  EUSART_IntClear(EUSART1, _EUSART_IF_MASK);
  EUSART_IntEnable(EUSART1, EUSART_IEN_RXFL);
  NVIC_ClearPendingIRQ(EUSART1_RX_IRQn);
  NVIC_SetPriority(EUSART1_RX_IRQn, BAY_IRQ_PRIO);
  NVIC_EnableIRQ(EUSART1_RX_IRQn);
  NVIC_ClearPendingIRQ(EUSART1_TX_IRQn);
  NVIC_SetPriority(EUSART1_TX_IRQn, BAY_IRQ_PRIO);
  NVIC_EnableIRQ(EUSART1_TX_IRQn);
}

/* ---- telemetry back to the encoder / the radio ---- */
static bool    tx_on;
static uint8_t tx_seq;
static uint8_t tx_buf[BAY_TXBUF];
static volatile uint8_t tx_n, tx_i;                              /* bytes queued, bytes handed to the FIFO */

bool bay_inverted(void) { return b_inv; }

/* CFG0 (RXINV / TXINV) can only be written with the EUSART disabled; the receiver starts over afterwards. */
static void b_reconfig(void)
{
  NVIC_DisableIRQ(EUSART1_RX_IRQn);
  NVIC_DisableIRQ(EUSART1_TX_IRQn);
  EUSART_Enable(EUSART1, eusartDisable);
  uint32_t c = EUSART1->CFG0 & ~(EUSART_CFG0_RXINV | EUSART_CFG0_TXINV);
  if (b_inv)   c |= EUSART_CFG0_RXINV;
  if (b_txinv) c |= EUSART_CFG0_TXINV;
  EUSART1->CFG0 = c;
  EUSART_Enable(EUSART1, tx_on ? eusartEnable : eusartEnableRx);
  tx_n = tx_i = 0;
  b_n = 0; b_done = true; b_err = false;
  b_fast = 0; b_settle = 2;
  b_last_us = (uint32_t)sl_rail_get_time(b_rail);                /* a frame under way is skipped */
  EUSART_IntClear(EUSART1, _EUSART_IF_MASK);
  EUSART_IntDisable(EUSART1, EUSART_IEN_TXFL);
  NVIC_ClearPendingIRQ(EUSART1_RX_IRQn);
  NVIC_ClearPendingIRQ(EUSART1_TX_IRQn);
  NVIC_EnableIRQ(EUSART1_RX_IRQn);
  NVIC_EnableIRQ(EUSART1_TX_IRQn);
}

void bay_set_inverted(bool rx_inv)
{
  if (rx_inv == b_inv && rx_inv == b_txinv) return;
  b_inv = rx_inv; b_txinv = rx_inv;
  b_reconfig();
}

void bay_tx_polarity(bool tx_inv)
{
  if (tx_inv == b_txinv) return;
  while (tx_on && bay_tx_busy()) { }                             /* let the last frame out first */
  b_txinv = tx_inv;
  b_reconfig();
  if (tx_on) {                                                   /* the idle level changed with it */
    sl_gpio_t tx = { .port = BAY_TX_PORT, .pin = BAY_TX_PIN };
    if (b_txinv) sl_gpio_clear_pin(&tx); else sl_gpio_set_pin(&tx);
  }
}

void EUSART1_TX_IRQHandler(void)
{
  EUSART1->IF_CLR = EUSART_IF_TXFL;
  while (tx_i < tx_n && (EUSART1->STATUS & EUSART_STATUS_TXFL)) EUSART1->TXDATA = tx_buf[tx_i++];
  if (tx_i >= tx_n) EUSART_IntDisable(EUSART1, EUSART_IEN_TXFL);
}

bool bay_tx_busy(void)
{
  return tx_i < tx_n || !(EUSART1->STATUS & EUSART_STATUS_TXIDLE);
}

bool bay_send_raw(const uint8_t *p, uint8_t n)
{
  if (!tx_on || n == 0u || n > BAY_TXBUF) return false;
  if (bay_tx_busy()) return false;
  NVIC_DisableIRQ(EUSART1_TX_IRQn);
  memcpy(tx_buf, p, n);
  tx_i = 0; tx_n = n;
  EUSART_IntClear(EUSART1, EUSART_IF_TXFL);
  EUSART_IntEnable(EUSART1, EUSART_IEN_TXFL);                    /* the interrupt fills the FIFO as it empties */
  NVIC_SetPendingIRQ(EUSART1_TX_IRQn);                            /* ... starting now */
  NVIC_EnableIRQ(EUSART1_TX_IRQn);
  return true;
}

uint8_t bay_crc8(const uint8_t *p, uint8_t n)
{
  uint8_t crc = 0;
  while (n--) {
    crc ^= *p++;
    for (uint8_t b = 0; b < 8u; b++) crc = (uint8_t)((crc & 0x80u) ? ((crc << 1) ^ 0x31u) : (unsigned)(crc << 1));
  }
  return crc;
}

bool bay_tx_on(void) { return tx_on; }

void bay_tx_begin(void)
{
  if (tx_on) return;
  sl_gpio_t tx = { .port = BAY_TX_PORT, .pin = BAY_TX_PIN };
  sl_gpio_set_pin_mode(&tx, SL_GPIO_MODE_PUSH_PULL, b_txinv ? 0 : 1);   /* the idle level before the route takes the pin */
  GPIO->EUSARTROUTE[EUSART_NUM(EUSART1)].TXROUTE =
      ((uint32_t)BAY_TX_PORT << _GPIO_EUSART_TXROUTE_PORT_SHIFT)
    | ((uint32_t)BAY_TX_PIN  << _GPIO_EUSART_TXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[EUSART_NUM(EUSART1)].ROUTEEN |= GPIO_EUSART_ROUTEEN_TXPEN;
  EUSART_Enable(EUSART1, eusartEnable);                          /* RX stays as it was, TX joins it */
  tx_on = true;
}

void bay_tx_end(void)
{
  if (!tx_on) return;
  while (bay_tx_busy()) { }                                      /* a frame on its way goes out (4 ms at most) */
  GPIO->EUSARTROUTE[EUSART_NUM(EUSART1)].ROUTEEN &= ~GPIO_EUSART_ROUTEEN_TXPEN;
  sl_gpio_t tx = { .port = BAY_TX_PORT, .pin = BAY_TX_PIN };
  sl_gpio_set_pin_mode(&tx, SL_GPIO_MODE_INPUT_PULL, b_txinv ? 0 : 1);   /* pulled to the idle level (encoder: up, as inputs_init left it) */
  tx_on = false;
}

bool bay_send_telemetry(const bay_telem_t *t)
{
  if (!tx_on) return false;
  if (bay_tx_busy()) return false;                               /* the last frame is still going out */
  uint8_t b[BAY_TELEM_LEN];
  b[0]  = 0xA5u;
  b[1]  = (uint8_t)((t->link ? 0x01u : 0u) | (t->bound ? 0x02u : 0u) | (t->binding ? 0x04u : 0u) | (t->range ? 0x08u : 0u)
                  | (t->vbat_sensor ? 0x10u : 0u) | (t->uk_link ? 0x20u : 0u) | (t->uk_active ? 0x40u : 0u) | (t->failsafe ? 0x80u : 0u));
  b[2]  = t->proto;
  b[3]  = (uint8_t)((t->sub & 0x0Fu) | ((t->link_alarm & 3u) << 4) | ((t->vbat_alarm & 3u) << 6));
  b[4]  = (uint8_t)t->power_dbm;
  b[5]  = (uint8_t)t->rssi_rx;
  b[6]  = (uint8_t)(t->rssi_tx_valid ? t->rssi_tx : BAY_NO_RSSI);
  b[7]  = t->lq;
  b[8]  = (uint8_t)(t->vbat_10mv & 0xFFu);
  b[9]  = (uint8_t)(t->vbat_10mv >> 8);
  b[10] = t->lost;
  b[11] = BAY_TELEM_VERSION;
  b[12] = (uint8_t)((t->rssi_rx_dbm ? 0x01u : 0u) | (t->rssi_tx_valid ? 0x02u : 0u) | (t->unsupported ? 0x04u : 0u));
  b[13] = t->uk_hop;
  b[14] = tx_seq++;
  b[15] = bay_crc8(b, BAY_TELEM_LEN - 1u);
  return bay_send_raw(b, BAY_TELEM_LEN);
}

bool bay_get(bay_frame_t *f)
{
  NVIC_DisableIRQ(EUSART1_RX_IRQn);
  *f = b_last;
  NVIC_EnableIRQ(EUSART1_RX_IRQn);
  return f->seq != 0u;
}
