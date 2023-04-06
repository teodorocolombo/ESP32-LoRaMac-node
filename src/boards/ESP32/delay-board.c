#include "delay-board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void DelayMsMcu(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}