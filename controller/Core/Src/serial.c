/* serial.c - see serial.h.  STM32F103, register level. */
#include "serial.h"
#include "stm32f1xx.h"

#define BAUD        115200u
#define APB2_HZ     72000000u          /* USART1 */
#define APB1_HZ     36000000u          /* USART2 */
#define TIMER_HZ    72000000u          /* TIM2 (APB1 timers run at 2 x APB1) */

/* ------------------------------------------------------------------ receive buffer, filled from interrupts */
#define RX_LEN 512u
static volatile uint8_t  rx_buf[RX_LEN];
static volatile uint16_t rx_head, rx_tail;
static volatile uint32_t rx_frames, rx_errors;

static void rx_put(uint8_t c)
{
  uint16_t next = (uint16_t)((rx_head + 1u) % RX_LEN);
  if (next != rx_tail) { rx_buf[rx_head] = c; rx_head = next; }
}

int serial_read(void)
{
  if (rx_head == rx_tail) return -1;
  uint8_t c = rx_buf[rx_tail];
  rx_tail = (uint16_t)((rx_tail + 1u) % RX_LEN);
  return c;
}

uint32_t serial_frames(void) { return rx_frames; }
uint32_t serial_errors(void) { return rx_errors; }

/* ------------------------------------------------------------------ transmit: USART1 on PA9 */
static void tx_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  USART1->BRR = APB2_HZ / BAUD;                                    /* 625: integer, 115200 exactly */
  USART1->CR2 = 0;
  USART1->CR3 = 0;
  USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
}

void serial_send(const char *text)
{
  for (; *text; text++) {
    uint32_t guard = 0;
    while (!(USART1->SR & USART_SR_TXE) && ++guard < 100000u) { }
    USART1->DR = (uint8_t)*text;
  }
}

/* ------------------------------------------------------------------ receive, three variants */
#if LINK_RX_PIN == 8
/* Software receiver on PA8. EXTI line 8 on both edges; the first edge of a byte (from idle to active) starts
   TIM2, which fires 1.25 bit times later and then once per bit; each time the line is sampled. 1.25 rather than
   1.5: through a slow stage the start edge is seen late, this keeps the samples away from the bit ends. */
#define BIT_TICKS   (TIMER_HZ / BAUD)                               /* 625 */
static volatile uint8_t  rx_invert;                                /* 1 = idle is low */
static volatile uint8_t  rx_bit, rx_byte;

static inline uint8_t line_level(void) { return (uint8_t)(((GPIOA->IDR >> 8) & 1u) ^ rx_invert); }

void EXTI9_5_IRQHandler(void)
{
  if (EXTI->PR & EXTI_PR_PR8) {
    EXTI->PR = EXTI_PR_PR8;
    if (line_level() == 0u) {                                      /* start bit */
      EXTI->IMR &= ~EXTI_IMR_MR8;
      TIM2->CR1 &= ~TIM_CR1_CEN;
      TIM2->CNT = 0;
      TIM2->ARR = (BIT_TICKS * 5u) / 4u - 1u;
      TIM2->SR = 0;
      TIM2->CR1 |= TIM_CR1_CEN;
      rx_bit = 0; rx_byte = 0; rx_frames++;
    }
  }
}

void TIM2_IRQHandler(void)
{
  TIM2->SR = 0;
  uint8_t level = line_level();
  if (rx_bit < 8u) {
    if (level) rx_byte |= (uint8_t)(1u << rx_bit);
    rx_bit++;
    TIM2->ARR = BIT_TICKS - 1u;
  } else {                                                         /* stop bit */
    if (level) rx_put(rx_byte); else rx_errors++;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    EXTI->PR = EXTI_PR_PR8;                                        /* edges inside the byte were data, not starts */
    EXTI->IMR |= EXTI_IMR_MR8;
  }
}

static void rx_init(void)
{
  GPIOA->CRH = (GPIOA->CRH & ~(0xFu << 0)) | (0x8u << 0);         /* PA8 input with pull-up */
  GPIOA->BSRR = 1u << 8;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->PSC = 0;
  TIM2->CR1 = TIM_CR1_URS;
  TIM2->DIER = TIM_DIER_UIE;
  NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);
  AFIO->EXTICR[2] &= ~AFIO_EXTICR3_EXTI8;                          /* EXTI8 <- port A */
  EXTI->RTSR |= EXTI_RTSR_TR8;
  EXTI->FTSR |= EXTI_FTSR_TR8;
  EXTI->PR = EXTI_PR_PR8;
  EXTI->IMR |= EXTI_IMR_MR8;
  NVIC_SetPriority(EXTI9_5_IRQn, 0);
  NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void serial_listen(void)
{
  rx_invert = (uint8_t)(((GPIOA->IDR >> 8) & 1u) ^ 1u);          /* whatever the line rests at now is idle */
  TIM2->CR1 &= ~TIM_CR1_CEN;
  rx_tail = rx_head;
  rx_frames = 0; rx_errors = 0;
  EXTI->PR = EXTI_PR_PR8;
  EXTI->IMR |= EXTI_IMR_MR8;
}

bool serial_inverted(void) { return rx_invert != 0u; }

#elif LINK_RX_PIN == 10
void USART1_IRQHandler(void)
{
  uint32_t sr = USART1->SR;
  uint8_t  c  = (uint8_t)USART1->DR;                               /* reading DR clears the flags */
  if ((sr & USART_SR_RXNE) && !(sr & (USART_SR_FE | USART_SR_NE))) { rx_put(c); rx_frames++; }
  else if (sr & USART_SR_FE) rx_errors++;                          /* a pressed DEC key reads as a break */
}

static void rx_init(void)
{
  USART1->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;                 /* PA10 is already an input with pull-up (the key) */
  NVIC_SetPriority(USART1_IRQn, 1);
  NVIC_EnableIRQ(USART1_IRQn);
}

void serial_listen(void) { rx_tail = rx_head; rx_frames = 0; rx_errors = 0; }
bool serial_inverted(void) { return false; }

#else  /* LINK_RX_PIN == 3 */
void USART2_IRQHandler(void)
{
  uint32_t sr = USART2->SR;
  uint8_t  c  = (uint8_t)USART2->DR;
  if (sr & USART_SR_RXNE) { rx_put(c); rx_frames++; }
}

static void rx_init(void)
{
  GPIOA->CRL = (GPIOA->CRL & ~(0xFu << 12)) | (0x8u << 12);       /* PA3 input with pull-up */
  GPIOA->BSRR = 1u << 3;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  USART2->BRR = (APB1_HZ + BAUD / 2u) / BAUD;                     /* 36 MHz / 115200 = 312.5: mantissa 19, fraction 9/16 */
  USART2->CR2 = 0;
  USART2->CR3 = 0;
  USART2->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
  NVIC_SetPriority(USART2_IRQn, 1);
  NVIC_EnableIRQ(USART2_IRQn);
}

void serial_listen(void) { rx_tail = rx_head; rx_frames = 0; rx_errors = 0; }
bool serial_inverted(void) { return false; }
#endif

void serial_init(void)
{
  tx_init();
  rx_init();
}
