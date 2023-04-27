#include "eeprom-board.h"
#include "utilities.h"

LmnStatus_t EepromMcuWriteBuffer(uint16_t addr, uint8_t *buffer, uint16_t size)
{
    LmnStatus_t status = LMN_STATUS_ERROR;

    return status;
}

LmnStatus_t EepromMcuReadBuffer(uint16_t addr, uint8_t *buffer, uint16_t size)
{
    return LMN_STATUS_ERROR;
}

void EepromMcuSetDeviceAddr(uint8_t addr)
{
    while (1) {
    }
}

LmnStatus_t EepromMcuGetDeviceAddr(void)
{
    while (1) {
    }
    //    return 0;
}
