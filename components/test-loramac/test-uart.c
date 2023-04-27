#include <stdio.h>

#include "test-uart.h"
#include "uart.h"
#include "utilities.h"
#include "board-config.h"
#include "uart-board.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Uart_t Uart1 = {
    .IsInitialized = false,
};


void test_uart(){
    int i = 0;
    UartInit(&Uart1, UART_1, UART_TX, UART_RX);
    printf("Uart iniciou %d\n", i++);
    UartConfig(&Uart1, RX_TX, /*921600*/115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL);

    printf("Uart configurado %d\n", i++);
      


    vTaskDelay(portMAX_DELAY);
}
