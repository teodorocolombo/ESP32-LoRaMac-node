#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board.h"
#include "test-rtc.h"
#include "test-uart.h"
#include "test-spi.h"
#include "test-sx1276.h"
#include "ping-pong.h"


void app_main(void)
{
    printf("Init MCU Test\n");

   //xTaskCreate(test_rtc, "taskss", 3 * 1024, NULL, 3, NULL);
   //xTaskCreate(test_uart, "taskss", 3 * 1024, NULL, 3, NULL); 
   //xTaskCreate(test_spi, "taskss", 3 * 1024, NULL, 3, NULL); 
   //xTaskCreate(test_sx1276, "taskss", 3 * 1024, NULL, 3, NULL); 

   xTaskCreate(start, "taskss", 3 * 1024, NULL, 3, NULL); 

    vTaskDelay(portMAX_DELAY);
}
