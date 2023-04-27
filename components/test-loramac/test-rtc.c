#include "rtc-board.h"
#include "test-rtc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


void test_rtc(void){
    int i = 0;

    esp_timer_init();
    printf("inicia o timer [%d]\n", i++);
    RtcInit();
    printf("inicia o rtc [%d]\n", i++);

    for (int j=0; j<100; j++) {
        if(j==10){
            printf("----------------\n");
            printf("delay de 10000 ms\n");
            RtcDelayMs(1000);
            printf("--------------\n");
            printf("delay de 100 ms\n");
            RtcSetAlarm(100);
            printf("----------------\n");

        }
            printf(
            "ms = %d | us = %d | ticks = %d |xticks = %d | s = %d\n  ",
            (int)RtcGetTimerValue(),
            (int)esp_timer_get_time(),
            (int)RtcMs2Tick(RtcGetTimerValue()),
            (int)xTaskGetTickCount(),
            (int)RtcGetCalendarTime(RtcGetTimerValue()));
        vTaskDelay(10);

    

}
    vTaskDelay(portMAX_DELAY);
}