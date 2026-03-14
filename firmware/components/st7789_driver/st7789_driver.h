#ifndef ST7789_DRIVER_H
#define ST7789_DRIVER_H

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

/* ST7789 常见分辨率 */
#define ST7789_WIDTH   240
#define ST7789_HEIGHT  240 // 如果是 240x320 请改为 320

/* 引脚定义（保持原样） */
#define PIN_NUM_MOSI  18
#define PIN_NUM_SCLK  8
#define PIN_NUM_CS    16
#define PIN_NUM_DC    17
#define PIN_NUM_RST   21
#define PIN_NUM_BL    15

/* 初始化函数返回 panel 句柄，供 LVGL 使用 */
esp_lcd_panel_handle_t st7789_init(void);

#endif