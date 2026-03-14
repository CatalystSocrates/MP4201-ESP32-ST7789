#include "st7789_driver.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "st7789_drv";

esp_lcd_panel_handle_t st7789_init(void)
{
    // 1. 初始化背光
    gpio_config_t bk_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_BL),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&bk_conf);
    gpio_set_level(PIN_NUM_BL, 1);

    // 2. 初始化 SPI 总线
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ST7789_WIDTH * 80 * sizeof(uint16_t), // 这里的 80 是缓存行数
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 3. 初始化 Panel IO (SPI 句柄)
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = 40 * 1000 * 1000, // 40MHz
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

    // 4. 初始化 ST7789 Panel 驱动
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    // 5. 配置屏幕
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);

    //交换 XY 轴 (将竖屏变为横屏的关键)
    esp_lcd_panel_swap_xy(panel_handle, true);
    esp_lcd_panel_mirror(panel_handle, false, true); 

    // 根据实际安装方向调整，ST7789 通常需要反色
    esp_lcd_panel_invert_color(panel_handle, true); 
    
    // 设置坐标偏移（如果是 240x240 在某些驱动下需要设置）
    esp_lcd_panel_set_gap(panel_handle, 80, 0);

    esp_lcd_panel_disp_on_off(panel_handle, true);

    ESP_LOGI(TAG, "ST7789 (esp_lcd) initialized");
    return panel_handle;
}