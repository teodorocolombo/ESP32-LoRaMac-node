#include "sx1276-board.h"
/*

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "test-sx1276.h"

typedef enum {
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
} States_t;



static RadioEvents_t RadioEvents;

void test_sx1276(){

    RadioEvents.TxDone    = OnTxDone;
    RadioEvents.RxDone    = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError   = OnRxError;
    
    SX1276IoInit();
    ESP_LOGI("sx1276", "iniciou");

    SX1276Init(&RadioEvents);



    vTaskDelay(portMAX_DELAY);

}

void OnTxDone(void)
{
    Radio.Sleep();
    State = TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    Radio.Sleep();
    BufferSize = size;
    memcpy(Buffer, payload, BufferSize);
    RssiValue = rssi;
    SnrValue  = snr;
    State     = RX;
}

void OnTxTimeout(void)
{
    Radio.Sleep();
    State = TX_TIMEOUT;
}

void OnRxTimeout(void)
{
    Radio.Sleep();
    State = RX_TIMEOUT;
}

void OnRxError(void)
{
    Radio.Sleep();
    State = RX_ERROR;
}
*/