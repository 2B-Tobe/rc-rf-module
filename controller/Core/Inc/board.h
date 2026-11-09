/*
 * board.h - the controller board: STM32F103C8 (72 MHz from an 8 MHz crystal), a 128x64 UC1701 display on SPI2,
 * four menu keys and one extra key, and the serial link to the RF module.
 *
 *   Display   SPI2: SCK PB13, MOSI PB15, CD (A0) PB14, RST PB12, no CS; backlight PB2
 *   Keys      INC PB4, DEC PA10, BACK PB11, ENTER PB3, AUX PB8 (all to GND, internal pull-ups; JTAG disabled, SWD kept)
 *   Link      TX: USART1 PA9, 115200 8N1  -> module J3 RX
 *             RX: see LINK_RX_PIN in serial.h <- module J3 TX
 *
 * Everything is register level (CMSIS device header only), no HAL.
 */
#ifndef BOARD_H
#define BOARD_H
#include <stdint.h>
#include <stdbool.h>

#define KEY_INC   0x01u
#define KEY_DEC   0x02u
#define KEY_BACK  0x04u
#define KEY_ENTER 0x08u
#define KEY_AUX   0x10u

void     board_init(void);            /* clocks, SysTick (1 ms), pins */
uint32_t board_ms(void);              /* milliseconds since start */
void     board_delay_ms(uint32_t ms);
void     board_delay_us(uint32_t us);
uint8_t  board_keys(void);            /* KEY_* bits, 1 = pressed (raw, not debounced) */
void     board_backlight(bool on);

/* display lines, used by display.c */
void     board_spi_write(uint8_t b);
void     board_disp_cd(bool data);    /* false = command, true = data */
void     board_disp_reset(bool high);
#endif
