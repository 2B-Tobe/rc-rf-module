/* board.c - see board.h.  STM32F103C8, register level. */
#include "board.h"
#include "stm32f1xx.h"

/* ------------------------------------------------------------------ time */
static volatile uint32_t ms_count;

void SysTick_Handler(void) { ms_count++; }

uint32_t board_ms(void) { return ms_count; }

void board_delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000u);
  while ((DWT->CYCCNT - start) < ticks) { }
}

void board_delay_ms(uint32_t ms)
{
  uint32_t start = ms_count;
  while ((ms_count - start) < ms) { }
}

/* ------------------------------------------------------------------ clock: HSE 8 MHz * 9 = 72 MHz */
static void clock_init(void)
{
  RCC->CR |= RCC_CR_HSEON;
  while (!(RCC->CR & RCC_CR_HSERDY)) { }

  FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;             /* two wait states above 48 MHz */

  RCC->CFGR = RCC_CFGR_PLLSRC                                       /* PLL from HSE */
            | RCC_CFGR_PLLMULL9                                     /* x9 */
            | RCC_CFGR_PPRE1_DIV2                                   /* APB1 36 MHz (APB1 timers x2 = 72 MHz) */
            | RCC_CFGR_ADCPRE_DIV6;
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY)) { }
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }
  SystemCoreClock = 72000000u;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;                  /* cycle counter for the microsecond delay */
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  SysTick_Config(SystemCoreClock / 1000u);                         /* 1 ms tick */
}

/* ------------------------------------------------------------------ pins */
/* STM32F1 GPIO: 4 bits per pin in CRL (pins 0-7) / CRH (pins 8-15): MODE[1:0] then CNF[1:0].
   0x8 = input with pull (pull direction from ODR), 0x2 = push-pull output 2 MHz, 0xB = alternate push-pull 50 MHz. */
static void pin_cfg(GPIO_TypeDef *port, uint8_t pin, uint8_t cfg)
{
  volatile uint32_t *reg = (pin < 8u) ? &port->CRL : &port->CRH;
  uint8_t shift = (uint8_t)((pin & 7u) * 4u);
  *reg = (*reg & ~(0xFu << shift)) | ((uint32_t)cfg << shift);
}

static void pin_pullup(GPIO_TypeDef *port, uint8_t pin) { port->BSRR = 1u << pin; }

static void gpio_init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
  /* PB3, PB4 (and PA15) are JTAG pins at reset: release them, keep SWD for programming and debugging */
  AFIO->MAPR = (AFIO->MAPR & ~AFIO_MAPR_SWJ_CFG) | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

  /* keys: inputs with pull-up */
  pin_cfg(GPIOB, 4, 0x8);  pin_pullup(GPIOB, 4);                  /* INC   */
  pin_cfg(GPIOA, 10, 0x8); pin_pullup(GPIOA, 10);                 /* DEC   */
  pin_cfg(GPIOB, 11, 0x8); pin_pullup(GPIOB, 11);                 /* BACK  */
  pin_cfg(GPIOB, 3, 0x8);  pin_pullup(GPIOB, 3);                  /* ENTER */
  pin_cfg(GPIOB, 8, 0x8);  pin_pullup(GPIOB, 8);                  /* AUX   */

  /* display: SPI2 SCK PB13 and MOSI PB15 as alternate push-pull, CD PB14 and RST PB12 as outputs, backlight PB2 */
  pin_cfg(GPIOB, 13, 0xB);
  pin_cfg(GPIOB, 15, 0xB);
  pin_cfg(GPIOB, 14, 0x2);
  pin_cfg(GPIOB, 12, 0x2);
  pin_cfg(GPIOB, 2, 0x2);

  /* link: USART1 TX PA9 as alternate push-pull (the receive pin is set up in serial.c) */
  pin_cfg(GPIOA, 9, 0xB);
}

uint8_t board_keys(void)
{
  uint32_t a = GPIOA->IDR, b = GPIOB->IDR;
  uint8_t k = 0;
  if (!(b & (1u << 4)))  k |= KEY_INC;
  if (!(a & (1u << 10))) k |= KEY_DEC;
  if (!(b & (1u << 11))) k |= KEY_BACK;
  if (!(b & (1u << 3)))  k |= KEY_ENTER;
  if (!(b & (1u << 8)))  k |= KEY_AUX;
  return k;
}

void board_backlight(bool on)
{
  GPIOB->BSRR = on ? (1u << 2) : ((1u << 2) << 16);
}

/* ------------------------------------------------------------------ display SPI */
static void spi_init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
  /* master, mode 0, 8 bit, MSB first, software slave management, fPCLK1 / 8 = 4.5 MHz
     (the UC1701 wants a clock period of at least 100 ns) */
  SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_SSM | SPI_CR1_SSI;
  SPI2->CR1 |= SPI_CR1_SPE;
}

void board_spi_write(uint8_t b)
{
  while (!(SPI2->SR & SPI_SR_TXE)) { }
  *(volatile uint8_t *)&SPI2->DR = b;
  while (!(SPI2->SR & SPI_SR_RXNE)) { }                            /* in master mode a byte is received as it is sent: */
  (void)SPI2->DR;                                                  /* RXNE = the last clock has gone out, CD may change */
}

void board_disp_cd(bool data)    { GPIOB->BSRR = data ? (1u << 14) : ((1u << 14) << 16); }
void board_disp_reset(bool high) { GPIOB->BSRR = high ? (1u << 12) : ((1u << 12) << 16); }

/* ------------------------------------------------------------------ */
void board_init(void)
{
  clock_init();
  gpio_init();
  spi_init();
}
