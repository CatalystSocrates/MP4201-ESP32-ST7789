#include "IIC_driver.h"
#include "esp_log.h"

static const char *TAG = "IIC_DRV";

esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed");
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
        return err;
    }

    ESP_LOGI(TAG, "I2C initialized successfully (SDA:%d, SCL:%d)", SDA_PIN, SCL_PIN);
    return ESP_OK;
}