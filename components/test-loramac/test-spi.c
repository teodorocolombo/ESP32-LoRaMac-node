#include "spi.h"
#include "spi-board.h"
#include "sx1276.h"
#include "sx1276-board.h"
#include "board-config.h"
#include "gpio.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "test-spi.h"

void test_spi(){
      
    SpiInit(&SX1276.Spi, SPI_2, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, RADIO_NSS);
    ESP_LOGI("spi", "Spi iniciado");

    
    //int res = SpiInOut(&SX1276.Spi, 10);


    SpiDeInit(&SX1276.Spi);

    ESP_LOGI("spi", "Spi Deletado");
    vTaskDelay(portMAX_DELAY);

}