/*
 * display.h - ST7565P LCD Display Driver using u8g2 library
 * Supports SPI communication with PA15 (CS), PA17 (DC), PA06 (RST)
 */

#ifndef DISPLAY_H_INCLUDED
#define DISPLAY_H_INCLUDED

#include <stdint.h>
#include <stddef.h>

void display_init(void);
void display_clear(void);
void display_draw_str(uint8_t x, uint8_t y, const char *str);
void display_send_buffer(void);
void display_printf(uint8_t x, uint8_t y, const char *fmt, ...);

#endif /* DISPLAY_H_INCLUDED */