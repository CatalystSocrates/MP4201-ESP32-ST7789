/********************************************************************************
* @author: LTS
* @date: 26-1-24 下午10:44
* @version: 1.0
* @description: 
********************************************************************************/
#include "MP4201Middleware.h"
#include "esp_log.h"

void MP4201_Delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

#ifdef HARDWARE_I2C
    void MP4201_SendByte(const uint8_t addr, uint8_t command)
    {
        i2c_master_write_to_device(MP4201_I2C_PORT, addr, &command, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    }

    void MP4201_WriteByte(const uint8_t addr, uint8_t command, uint8_t data)
    {
        uint8_t buf[2] = {command, data};
        i2c_master_write_to_device(MP4201_I2C_PORT, addr, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    }

    void MP4201_WriteWord(const uint8_t addr, uint8_t command, uint16_t data)
    {
        uint8_t buf[3];
        buf[0] = command;
        buf[1] = (uint8_t)data;         // Low Byte
        buf[2] = (uint8_t)(data >> 8);  // High Byte
        i2c_master_write_to_device(MP4201_I2C_PORT, addr, buf, 3, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    }

    void MP4201_ReadByte(const uint8_t addr, uint8_t command, uint8_t *data)
    {
        i2c_master_write_read_device(MP4201_I2C_PORT, addr, &command, 1, data, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    }

    void MP4201_ReadWord(const uint8_t addr, uint8_t command, uint16_t *data)
    {
        uint8_t buf[2] = {0};
        esp_err_t err = i2c_master_write_read_device(MP4201_I2C_PORT, addr, &command, 1, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        *data = (uint16_t)((buf[1] << 8) | buf[0]);
        if (err != ESP_OK) {
            ESP_LOGE("MP4201_I2C", "Read Error: 0x%X", err);
        }
    }
#endif

#ifdef SOFTWARE_I2C
    /**
     * @brief 向 MP4201 发送一个字节
     * @param addr MP4201 I2C 地址
     * @param command
     */
    void MP4201_SendByte(const uint8_t addr, uint8_t command)
    {
        bool Is_no_ACK = 0;

        IIC_Start();
        IIC_Send(addr << 1 | 0x01); // write addr
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;

        IIC_Send(command);
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;
        IIC_Stop();
    }

    /**
     * @brief 向 MP4201 寄存器写入一个字节
     * @param addr MP4201 I2C地址
     * @param command MP4201 从设备指令
     * @param data 写入 MP4201 寄存器的8位数据
     */
    void MP4201_WriteByte(const uint8_t addr, uint8_t command, uint8_t data)
    {
        bool Is_no_ACK = 0;

        IIC_Start();
        IIC_Send(addr << 1 | 0x01); // write addr
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;

        IIC_Send(command);
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;

        IIC_Send(data);
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;
        IIC_Stop();
    }

    /**
     * 向 MP4201 寄存器写入一个字
     * @param addr MP4201 I2C地址
     * @param command MP4201 从设备指令
     * @param data 写入 MP4201 寄存器的16位数据
     */
    void MP4201_WriteWord(const uint8_t addr, uint8_t command, uint16_t data)
    {
        bool Is_no_ACK = 0;
        uint8_t data_8bit[2] = {0x00};
        data_8bit[0] = (uint8_t)data;
        data_8bit[1] = data >> 8;

        IIC_Start();
        IIC_Send(addr << 1 | 0x01); // write addr
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;

        IIC_Send(command);
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;

        IIC_Send(data_8bit[0]);
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;

        IIC_Send(data_8bit[1]);
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;
        IIC_Stop();
    }

    /**
     * 向 MP4201 寄存器读取一个字节
     * @param addr MP4201 I2C地址
     * @param command MP4201 从设备指令
     * @param data 读取 MP4201 寄存器的8位数据
     */
    void MP4201_ReadByte(const uint8_t addr, uint8_t command, uint8_t *data)
    {
        bool Is_no_ACK = 0;

        IIC_Start();
        IIC_Send(addr << 1 | 0x01); // write addr
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;

        IIC_Send(command);
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;

        IIC_Send(addr << 1 | 0x00); // read addr
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;

        *data = IIC_Read();
        IIC_Stop();
    }

    /**
     * 向 MP4201 寄存器读取一个字
     * @param addr MP4201 I2C地址
     * @param command MP4201 从设备指令
     * @param data 读取 MP4201 寄存器的16位数据
     */
    void MP4201_ReadWord(const uint8_t addr, uint8_t command, uint16_t *data)
    {
        uint8_t data_8bit[2] = {0x00};
        bool Is_no_ACK = 0;

        IIC_Start();
        IIC_Send(addr << 1 | 0x01); // write addr
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;

        IIC_Send(command);
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;

        IIC_Send(addr << 1 | 0x00); // read addr
        Is_no_ACK = IIC_Wait_Ack();
        if (Is_no_ACK)
            return;
        data_8bit[1] = IIC_Read();
        IIC_Ack(0);

        data_8bit[0] = IIC_Read();
        IIC_Ack(0);
        IIC_Stop();

        *data = (uint16_t)(data_8bit[1] << 8 | data_8bit[0]);
    }
#endif
