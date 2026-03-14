#include "MCP4725.h"
#include "esp_log.h"

static const char *TAG = "MCP4725";

esp_err_t MCP4725_Init(uint8_t addr) {
    // 探测设备是否在线：发送一个空命令
    uint8_t probe_data = 0;
    return i2c_master_write_to_device(MCP4725_I2C_PORT, addr, &probe_data, 0, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t MCP4725_SetVoltage(uint8_t addr, uint16_t voltage, bool save_to_eeprom) {
    if (voltage > 4095) voltage = 4095;

    uint8_t data[3];
    data[0] = save_to_eeprom ? MCP4725_MODE_EEPROM : MCP4725_MODE_WRITE;
    data[1] = (voltage >> 4) & 0xFF;     // 高8位
    data[2] = (voltage << 4) & 0xF0;     // 低4位

    esp_err_t err = i2c_master_write_to_device(MCP4725_I2C_PORT, addr, data, 3, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Write to 0x%02X failed: %s", addr, esp_err_to_name(err));
    }
    return err;
}

void MP4201_SET_MODE_DAC(int mode_idx) {
    uint16_t dac_val[] = {DAC_VAL_MODE_PFM_FSS, DAC_VAL_MODE_PFM_nFSS, DAC_VAL_MODE_FCCM_FSS, DAC_VAL_MODE_FCCM_nFSS};
    if (mode_idx > 3) mode_idx = 3;
    MCP4725_SetVoltage(MCP4725_ADDR_MODE, dac_val[mode_idx], true);
    ESP_LOGI(TAG, "Mode DAC set to: %d", dac_val[mode_idx]);
}

void MP4201_SET_FREQ_DAC(int freq_idx) {
    uint16_t dac_vals[] = {DAC_VAL_FREQ_200K, DAC_VAL_FREQ_400K, DAC_VAL_FREQ_600K, DAC_VAL_FREQ_1MZ};
    if (freq_idx > 3) freq_idx = 3;
    MCP4725_SetVoltage(MCP4725_ADDR_FREQ, dac_vals[freq_idx], true);
    ESP_LOGI(TAG, "Freq DAC set to: %d", dac_vals[freq_idx]);
}