#include "driver/gpio.h"
#include "driver/uart.h"


#include "board.h"
#include "freertos/FreeRTOS.h"
#include "uart-board.h"



void UartMcuInit(Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx)
{
    obj->UartId = uartId;

    // UART configuration
    uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(obj->UartId, &uart_config);

    // UART GPIO initialization
    uart_set_pin(obj->UartId, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Driver installation
    uart_driver_install(obj->UartId, 256, 0, 0, NULL, 0);
}

void UartMcuConfig(
    Uart_t      *obj,
    UartMode_t   mode,
    uint32_t     baudrate,
    WordLength_t wordLength,
    StopBits_t   stopBits,
    Parity_t     parity,
    FlowCtrl_t   flowCtrl)
{
    // Change the baudrate
    uart_set_baudrate(obj->UartId, baudrate);
}

void UartMcuDeInit(Uart_t *obj)
{
    uart_driver_delete(obj->UartId);
}

uint8_t UartMcuPutChar(Uart_t *obj, uint8_t data)
{
    if (uart_write_bytes(obj->UartId, (const char *)&data, 1) == 1) {
        return 0;   // OK
    }
    return 1;   // Busy
}

uint8_t UartMcuGetChar(Uart_t *obj, uint8_t *data)
{
    if (uart_read_bytes(obj->UartId, data, 1, pdMS_TO_TICKS(20)) == 1) {
        return 0;   // OK
    }
    return 1;   // Busy
}

uint8_t UartMcuPutBuffer(Uart_t *obj, uint8_t *buffer, uint16_t size)
{
    if (uart_write_bytes(obj->UartId, (const char *)buffer, size) == size) {
        return 0;   // OK
    }
    return 1;   // Error
}

uint8_t
UartMcuGetBuffer(Uart_t *obj, uint8_t *buffer, uint16_t size, uint16_t *nbReadBytes)
{
    *nbReadBytes = uart_read_bytes(obj->UartId, buffer, size, pdMS_TO_TICKS(20));
    if (*nbReadBytes == 0) {
        return 1;   // Empty
    }
    return 0;   // OK
}
