/*
 * display.h - the 128x64 display through u8glib (UC1701 "mini12864", two-page buffer), 7x14 fixed font:
 * 18 columns x 5 rows. Rows are 0..4 (text baselines at 12, 24, 36, 48, 60), columns 0..17.
 */
#ifndef DISPLAY_H
#define DISPLAY_H
#include <stdint.h>
#include <stdbool.h>

#define DISP_COLS 18
#define DISP_ROWS 5

void display_init(void);
void display_begin(void);                                  /* start a frame: call draw_* between begin and end */
bool display_end(void);                                    /* true while more passes are needed: loop begin..end */
void draw_text(uint8_t col, uint8_t row, const char *s);
void draw_text_reversed(uint8_t col, uint8_t row, const char *s);   /* white on black: the value being edited */
void draw_cursor(uint8_t col, uint8_t row);                /* a bar in front of a row */
#endif
