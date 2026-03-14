#include "lvgl.h"
#include "st7789_driver.h"

static esp_lcd_panel_handle_t panel_handle = NULL;

/* 显存缓冲区：ST7789 分辨率大，建议增加缓存行数 */
#define DRAW_BUF_LINES 40
static lv_color_t buf1[ST7789_WIDTH * DRAW_BUF_LINES];
static lv_color_t buf2[ST7789_WIDTH * DRAW_BUF_LINES];
static lv_disp_draw_buf_t draw_buf;

/* 修改后的 flush 回调，调用 esp_lcd 接口 */
static void st7789_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    // 使用 esp_lcd 的高效刷屏接口
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    
    // 告知 LVGL 刷新完成 (如果是 DMA 异步传输，通常在 esp_lcd 的 on_color_trans_done 回调里调这个)
    // 这里简单处理，同步等待或利用 esp_lcd 内部排队
    lv_disp_flush_ready(disp_drv);
}

void lv_port_disp_init(void)
{
    // 初始化硬件并获取句柄
    panel_handle = st7789_init();

    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, ST7789_WIDTH * DRAW_BUF_LINES);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = ST7789_WIDTH;
    disp_drv.ver_res = ST7789_HEIGHT;
    disp_drv.flush_cb = st7789_flush_cb;
    disp_drv.draw_buf = &draw_buf;

    lv_disp_drv_register(&disp_drv);
}