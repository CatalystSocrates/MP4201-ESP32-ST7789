#ifndef __MCP4725_H
#define __MCP4725_H

#include "driver/i2c.h"

// ESP-IDF 使用 7 位地址
#define MCP4725_ADDR_MODE 0x60  // A0引脚接地 -> 控制 MODE
#define MCP4725_ADDR_FREQ 0x61  // A0引脚接高 -> 控制 FREQ

// 对应 IIC_driver.c 中的定义
#define MCP4725_I2C_PORT  I2C_NUM_0 
#define I2C_TIMEOUT_MS    100

// 操作模式
#define MCP4725_MODE_WRITE  0x40  // 写入DAC寄存器(不保存)
#define MCP4725_MODE_EEPROM 0x60  // 写入DAC并保存到EEPROM

// 根据您的电路需求定义 DAC 输出值 (12位: 0-4095)
// 假设：PFM需要低电压，FCCM需要高电压；频率随电压升高而升高
typedef enum {
    DAC_VAL_MODE_PFM_FSS  = 1365,   // 约 0.6V
    DAC_VAL_MODE_PFM_nFSS  = 0,   // 约 0.6V
    DAC_VAL_MODE_FCCM_FSS = 4096,  // 约 2.4V
    DAC_VAL_MODE_FCCM_nFSS = 2730,  // 约 2.4V
} MCP4725_MP4201_MODE_t;

typedef enum {
    DAC_VAL_FREQ_200K = 0,
    DAC_VAL_FREQ_400K = 1365,
    DAC_VAL_FREQ_600K = 2730,
    DAC_VAL_FREQ_1MZ  = 4096,
} MCP4725_MP4201_FREQ_t;

esp_err_t MCP4725_Init(uint8_t addr);
esp_err_t MCP4725_SetVoltage(uint8_t addr, uint16_t voltage, bool save_to_eeprom);

// 适配业务逻辑的接口
void MP4201_SET_MODE_DAC(int mode_idx);
void MP4201_SET_FREQ_DAC(int freq_idx);

#endif