/*
 * display.c - ST7565P LCD Display Driver using u8g2 library
 * SPI-based driver with u8g2 integration for 128x64 display
 */

#include "display.h"
#include "asf.h"
#include "u8g2.h"
#include "u8x8.h"
#include <stdio.h>
#include <stdarg.h>

/* Pin definitions for EXT1 header */
#define DISP_CS_PIN    PIN_PA15   /* EXT1_PIN_10 */
#define DISP_DC_PIN    PIN_PA17   /* EXT1_PIN_12 */
#define DISP_RST_PIN   PIN_PA06   /* EXT1_PIN_3  */

/* SPI module reference */
#define DISPLAY_SPI_MODULE SERCOM5

/* Global u8g2 struct */
static u8g2_t u8g2;
static struct spi_module spi_master;

/* Forward declarations for u8g2 callbacks */
static uint8_t u8x8_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
static uint8_t u8x8_byte_hw_spi_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

/* Initialize SERCOM5 as SPI master (blocking mode) */
static void display_spi_init(void)
{
    struct spi_config config;
    struct system_pinmux_config pinmux_config;
    
    /* Configure MOSI: PB02 (SERCOM5 Pad 0) */
    system_pinmux_get_config_defaults(&pinmux_config);
    pinmux_config.mux_position = PINMUX_PB02D_SERCOM5_PAD0;
    system_pinmux_pin_set_config(PIN_PB02, &pinmux_config);
    
    /* Configure SCK: PB23 (SERCOM5 Pad 3) */
    system_pinmux_get_config_defaults(&pinmux_config);
    pinmux_config.mux_position = PINMUX_PB23D_SERCOM5_PAD3;
    system_pinmux_pin_set_config(PIN_PB23, &pinmux_config);
    
    /* Configure CS/DC/RST as GPIO outputs */
    system_pinmux_get_config_defaults(&pinmux_config);
    pinmux_config.mux_position = SYSTEM_PINMUX_GPIO;
    pinmux_config.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
    system_pinmux_pin_set_config(DISP_CS_PIN, &pinmux_config);
    system_pinmux_pin_set_config(DISP_DC_PIN, &pinmux_config);
    system_pinmux_pin_set_config(DISP_RST_PIN, &pinmux_config);
    
    /* Set initial states */
    port_pin_set_output_level(DISP_CS_PIN, true);   /* CS inactive (high) */
    port_pin_set_output_level(DISP_DC_PIN, false);  /* DC low (command mode) */
    port_pin_set_output_level(DISP_RST_PIN, true);  /* RST high */
    
    /* Configure SPI master */
    spi_get_config_defaults(&config);
    config.mode = SPI_MODE_MASTER;
    config.data_order = SPI_DATA_ORDER_MSB;
    config.transfer_mode = SPI_TRANSFER_MODE_0;
    config.mux_setting = SPI_SIGNAL_MUX_SETTING_D;
    config.character_size = SPI_CHARACTER_SIZE_8BIT;
    config.mode_specific.master.baudrate = 4000000; /* 4 MHz */
    config.pinmux_pad0 = PINMUX_PB02D_SERCOM5_PAD0; /* MOSI */
    config.pinmux_pad1 = PINMUX_UNUSED;              /* MISO not used in output mode */
    config.pinmux_pad2 = PINMUX_UNUSED;              /* Not used */
    config.pinmux_pad3 = PINMUX_PB23D_SERCOM5_PAD3; /* SCK */
    
    spi_init(&spi_master, (Sercom *)DISPLAY_SPI_MODULE, &config);
    spi_enable(&spi_master);
}

/* GPIO & delay callback for u8x8 */
static uint8_t u8x8_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    (void)u8x8;
    (void)arg_ptr;
    
    switch(msg) {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        display_spi_init();
        break;
    case U8X8_MSG_DELAY_MILLI:
        delay_ms(arg_int);
        break;
    case U8X8_MSG_DELAY_10MICRO:
        delay_us(10 * arg_int);
        break;
    case U8X8_MSG_DELAY_100NANO:
        /* Minimal delay - skip */
        break;
    case U8X8_MSG_GPIO_D0:
    case U8X8_MSG_GPIO_D1:
    case U8X8_MSG_GPIO_D2:
    case U8X8_MSG_GPIO_D3:
    case U8X8_MSG_GPIO_D4:
    case U8X8_MSG_GPIO_D5:
    case U8X8_MSG_GPIO_D6:
    case U8X8_MSG_GPIO_D7:
    case U8X8_MSG_GPIO_E:
        /* Not used in SPI mode */
        break;
    case U8X8_MSG_GPIO_CS:
        port_pin_set_output_level(DISP_CS_PIN, arg_int);
        break;
    case U8X8_MSG_GPIO_DC:
        port_pin_set_output_level(DISP_DC_PIN, arg_int);
        break;
    case U8X8_MSG_GPIO_RESET:
        port_pin_set_output_level(DISP_RST_PIN, arg_int);
        break;
    default:
        return 0;
    }
    return 1;
}

/* SPI byte transfer callback for u8x8 - blocking write */
static uint8_t u8x8_byte_hw_spi_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    (void)u8x8;
    
    switch(msg) {
    case U8X8_MSG_BYTE_INIT:
        /* SPI already initialized in GPIO callback */
        break;
    case U8X8_MSG_BYTE_SEND:
        {
            uint8_t *data = (uint8_t *)arg_ptr;
            /* Blocking SPI write for all bytes */
            spi_write_buffer_wait(&spi_master, data, (uint16_t)arg_int);
        }
        break;
    case U8X8_MSG_BYTE_SET_DC:
        port_pin_set_output_level(DISP_DC_PIN, arg_int);
        break;
    case U8X8_MSG_BYTE_START_TRANSFER:
        port_pin_set_output_level(DISP_CS_PIN, 0);
        break;
    case U8X8_MSG_BYTE_END_TRANSFER:
        port_pin_set_output_level(DISP_CS_PIN, 1);
        break;
    default:
        return 0;
    }
    return 1;
}

/* Public API: Initialize display */
void display_init(void)
{
    /* Setup u8g2 with ST7565 variant for NHD C12864 display (128x64 LCD) */
    u8g2_Setup_st7565_nhd_c12864_1(&u8g2, U8G2_R0, u8x8_byte_hw_spi_cb, u8x8_gpio_and_delay_cb);
    
    /* Initialize display */
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);    /* Wake up display */
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);  /* Use a simple font */
    
    /* Clear and show welcome message */
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawStr(&u8g2, 0, 10, "Display Ready");
    u8g2_SendBuffer(&u8g2);
}

/* Public API: Clear display buffer */
void display_clear(void)
{
    u8g2_ClearBuffer(&u8g2);
}

/* Public API: Draw string at position */
void display_draw_str(uint8_t x, uint8_t y, const char *str)
{
    if(str) {
        u8g2_DrawStr(&u8g2, x, y, str);
    }
}

/* Public API: Send buffer to display */
void display_send_buffer(void)
{
    u8g2_SendBuffer(&u8g2);
}

/* Public API: Printf-style draw at position */
void display_printf(uint8_t x, uint8_t y, const char *fmt, ...)
{
    static char buf[32];
    va_list args;
    
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    
    u8g2_DrawStr(&u8g2, x, y, buf);
}
