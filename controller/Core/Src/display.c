/* display.c - see display.h.  u8glib on SPI2; the communication procedure below is the one u8glib calls for
   every byte and control line (see u8g_InitComFn in the library). */
#include "display.h"
#include "board.h"
#include "u8g.h"
#include <string.h>

static u8g_t u8g;

/* the three delay hooks u8glib expects from a bare-metal Arm target */
void u8g_Delay(uint16_t val)  { board_delay_ms(val); }
void u8g_MicroDelay(void)     { board_delay_us(1); }
void u8g_10MicroDelay(void)   { board_delay_us(10); }

static uint8_t com_spi2(u8g_t *u, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
  (void)u;
  switch (msg) {
    case U8G_COM_MSG_INIT:
      board_disp_reset(true);
      break;
    case U8G_COM_MSG_RESET:
      board_disp_reset(arg_val != 0u);
      break;
    case U8G_COM_MSG_ADDRESS:                                      /* 0 = command, 1 = data */
      board_disp_cd(arg_val != 0u);
      break;
    case U8G_COM_MSG_WRITE_BYTE:
      board_spi_write(arg_val);
      break;
    case U8G_COM_MSG_WRITE_SEQ:
    case U8G_COM_MSG_WRITE_SEQ_P: {
      const uint8_t *p = (const uint8_t *)arg_ptr;
      while (arg_val--) board_spi_write(*p++);
      break;
    }
    default:                                                       /* no chip select line, nothing to stop */
      break;
  }
  return 1;
}

void display_init(void)
{
  u8g_InitComFn(&u8g, &u8g_dev_uc1701_mini12864_2x_hw_spi, com_spi2);
  u8g_SetFont(&u8g, u8g_font_7x14);
  u8g_SetContrast(&u8g, 180);
}

void display_begin(void) { u8g_FirstPage(&u8g); }
bool display_end(void)   { return u8g_NextPage(&u8g) != 0u; }

static u8g_uint_t col_x(uint8_t col) { return (u8g_uint_t)(col * 7u); }
static u8g_uint_t row_y(uint8_t row) { return (u8g_uint_t)(12u + row * 12u); }

void draw_text(uint8_t col, uint8_t row, const char *s)
{
  u8g_DrawStr(&u8g, col_x(col), row_y(row), s);
}

void draw_text_reversed(uint8_t col, uint8_t row, const char *s)
{
  u8g_uint_t w = (u8g_uint_t)(7u * strlen(s) + 2u);
  u8g_DrawBox(&u8g, (u8g_uint_t)(col_x(col) - 1u), (u8g_uint_t)(row_y(row) - 11u), w, 14);
  u8g_SetDefaultBackgroundColor(&u8g);
  u8g_DrawStr(&u8g, col_x(col), row_y(row), s);
  u8g_SetDefaultForegroundColor(&u8g);
}

void draw_cursor(uint8_t col, uint8_t row)
{
  u8g_DrawBox(&u8g, col_x(col), (u8g_uint_t)(row_y(row) - 10u), 5, 12);
}
