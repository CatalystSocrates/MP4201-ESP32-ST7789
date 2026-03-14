#ifndef IIC_DRIVER_H
#define IIC_DRIVER_H

#include "driver/i2c.h"
#include "esp_err.h"

/* 引脚配置（按你之前提到的变量名定义） */
#define SDA_PIN             7  // 替换为你的实际引脚
#define SCL_PIN             6  // 替换为你的实际引脚
#define I2C_MASTER_PORT     I2C_NUM_0
#define I2C_FREQ_HZ         400000

/* 初始化函数 */
esp_err_t i2c_master_init(void);

#endif