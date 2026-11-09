/* sbus_out.c — see sbus_out.h.  #included into app_process.c.   v1.1: on EUSART1, the log keeps EUSART0. */
#include "sbus_out.h"
#include "servo_out.h"
#include "em_eusart.h"
#include "em_cmu.h"
#include "em_device.h"
#include "sl_gpio.h"
#include <string.h>

volatile int32_t sbus_frames;

#if SBUS_ENABLE
static uint8_t  s_buf[25];
static uint8_t  s_left;                          /* bytes still to push, 0 = idle */
static bool     s_ready;
static sl_gpio_t s_pin = { .port = SBUS_PORT, .pin = SBUS_PIN };

void sbus_init(void)
{
  sl_gpio_set_pin_mode(&s_pin, SL_GPIO_MODE_PUSH_PULL, 0);
  EUSART_UartInit_TypeDef init = EUSART_UART_INIT_DEFAULT_HF;
  EUSART_AdvancedInit_TypeDef adv = EUSART_ADVANCED_INIT_DEFAULT;
  adv.invertIO          = eusartInvertTxEnable;  /* SBUS is inverted: idle low */
  init.baudrate         = 100000;
  init.databits         = eusartDataBits8;
  init.parity           = eusartEvenParity;
  init.stopbits         = eusartStopbits2;
  init.advancedSettings = &adv;
  init.enable           = eusartEnableTx;
  CMU_ClockEnable(cmuClock_EUSART1, true);
  EUSART_UartInitHf(EUSART1, &init);
  GPIO->EUSARTROUTE[EUSART_NUM(EUSART1)].TXROUTE =
      ((uint32_t)s_pin.port << _GPIO_EUSART_TXROUTE_PORT_SHIFT)
    | ((uint32_t)s_pin.pin  << _GPIO_EUSART_TXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[EUSART_NUM(EUSART1)].ROUTEEN = GPIO_EUSART_ROUTEEN_TXPEN;
  s_left = 0; s_ready = false; sbus_frames = 0;
}

static uint16_t sbus_val(uint16_t us)            /* 1000..2000 us -> 172..1811 */
{
  if (us < 880u)  us = 880u;
  if (us > 2120u) us = 2120u;
  int32_t v = 172 + (((int32_t)us - 1000) * 1639 + 500) / 1000;
  if (v < 0)    v = 0;
  if (v > 2047) v = 2047;
  return (uint16_t)v;
}

bool sbus_send(const uint16_t us[8], bool frame_lost, bool failsafe)
{
  if (s_left) return false;                      /* previous frame still going out */
  uint16_t ch[16];
  const volatile uint16_t *ov = servo_out_overridden() ? servo_out_current() : NULL;   /* dual: the 459 MHz data */
  if (ov != NULL) { frame_lost = false; failsafe = false; }
  for (uint8_t i = 0; i < 8; i++)  ch[i] = sbus_val(ov ? ov[i] : us[i]);
  for (uint8_t i = 8; i < 16; i++) ch[i] = sbus_val(1500u);
  memset(s_buf, 0, sizeof(s_buf));
  s_buf[0] = 0x0F;
  uint32_t acc = 0; uint8_t nb = 0, o = 1;
  for (uint8_t i = 0; i < 16; i++) {
    acc |= (uint32_t)(ch[i] & 0x7FFu) << nb; nb += 11;
    while (nb >= 8) { s_buf[o++] = (uint8_t)(acc & 0xFFu); acc >>= 8; nb -= 8; }
  }
  s_buf[23] = (uint8_t)((frame_lost ? 0x04u : 0u) | (failsafe ? 0x08u : 0u));
  s_buf[24] = 0x00;
  s_left = 25; s_ready = true; sbus_frames++;
  return true;
}

bool sbus_idle(void) { return s_left == 0; }
void sbus_set_pin(uint8_t port, uint8_t pin) { s_pin.port = (sl_gpio_port_t)port; s_pin.pin = pin; }

void sbus_service(void)
{
  if (!s_ready) return;
  while (s_left && (EUSART1->STATUS & EUSART_STATUS_TXFL)) {
    EUSART1->TXDATA = s_buf[25u - s_left];
    s_left--;
  }
}
#else
void sbus_init(void) { }
void sbus_set_pin(uint8_t port, uint8_t pin) { (void)port; (void)pin; }
bool sbus_send(const uint16_t us[8], bool fl, bool fs) { (void)us; (void)fl; (void)fs; return true; }
void sbus_service(void) { }
bool sbus_idle(void) { return true; }
#endif
