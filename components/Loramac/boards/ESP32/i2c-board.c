#include "i2c-board.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"

#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

void I2cMcuInit(I2c_t *obj, I2cId_t i2cId, PinNames scl, PinNames sda)
{
    obj->I2cId = i2cId;

    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = sda,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_io_num       = scl,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000};
    i2c_param_config(i2cId, &conf);
    i2c_driver_install(
        i2cId, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void I2cMcuDeInit(I2c_t *obj)
{
    i2c_driver_delete(obj->I2cId);
}

void I2cMcuFormat(
    I2c_t         *obj,
    I2cMode        mode,
    I2cDutyCycle   dutyCycle,
    bool           I2cAckEnable,
    I2cAckAddrMode AckAddrMode,
    uint32_t       I2cFrequency)
{
    i2c_config_t i2c_config;
    i2c_config.master.clk_speed = I2cFrequency;
    i2c_param_config(obj->I2cId, &i2c_config);
}

LmnStatus_t
I2cMcuWriteBuffer(I2c_t *obj, uint8_t deviceAddr, uint8_t *buffer, uint16_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buffer, size, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(obj->I2cId, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret == ESP_OK ? LMN_STATUS_OK : LMN_STATUS_ERROR;
}

LmnStatus_t
I2cMcuReadBuffer(I2c_t *obj, uint8_t deviceAddr, uint8_t *buffer, uint16_t size)
{
    if (size == 0) {
        return LMN_STATUS_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_READ, true);
    if (size > 1) {
        i2c_master_read(cmd, buffer, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buffer + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(obj->I2cId, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret == ESP_OK ? LMN_STATUS_OK : LMN_STATUS_ERROR;
}

LmnStatus_t I2cMcuWriteMemBuffer(
    I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (addr >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, addr & 0xFF, true);
    i2c_master_write(cmd, buffer, size, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(obj->I2cId, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret == ESP_OK ? LMN_STATUS_OK : LMN_STATUS_ERROR;
}

LmnStatus_t I2cMcuReadMemBuffer(
    I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size)
{
    if (size == 0) {
        return LMN_STATUS_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, (addr >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, addr & 0xFF, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_READ, true);
    if (size > 1) {
        i2c_master_read(cmd, buffer, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buffer + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(obj->I2cId, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret == ESP_OK ? LMN_STATUS_OK : LMN_STATUS_ERROR;
}
