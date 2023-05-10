#include "test-ping.h"
#include "board.h"
#include "delay.h"
#include "gpio.h"
#include "radio.h"
#include "timer.h"
#include <stdio.h>
#include <string.h>

#include "sx1276-board.h"
#include "sx1276.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define RF_FREQUENCY  925200000
#define RX_TIMEOUT_VALUE 1000

#define BUFFER_SIZE 64   // Define the payload size here

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t  Buffer[BUFFER_SIZE];
#define TX_OUTPUT_POWER 14   // dBm

#define LORA_BANDWIDTH 0
                                     /*// [0: 125 kHz, \ 
                                      //  1: 250 kHz, \
                                      //  2: 500 kHz, \
                                      //  3: Reserved]
                                      */
#define LORA_SPREADING_FACTOR 7   // [SF7..SF12]
#define LORA_CODINGRATE 1
                                /*// [1: 4/5, \
                                 //  2: 4/6, \
                                 //  3: 4/7, \
                                 //  4: 4/8]
                                 */
#define LORA_PREAMBLE_LENGTH 8   // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 5    // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false



static RadioEvents_t RadioEvents;

void OnTxDone();
void OnTxTimeout();

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout();


void func(){
    BoardInitMcu();

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;


    Radio.Init(&RadioEvents);

    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        true,
        0,
        0,
        LORA_IQ_INVERSION_ON,
        3000);

    Radio.SetRxConfig(
        MODEM_LORA,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODINGRATE,
        0,
        LORA_PREAMBLE_LENGTH,
        LORA_SYMBOL_TIMEOUT,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        0,
        true,
        false,
        0,
        LORA_IQ_INVERSION_ON,
        true);

    Radio.SetMaxPayloadLength(MODEM_LORA, BUFFER_SIZE);
    Radio.SetPublicNetwork(true);
    
    printf("Rssi %d", SX1276ReadRssi(MODEM_LORA));

    Radio.SetChannel(RF_FREQUENCY);
    Radio.Send(Buffer,BufferSize);
    Radio.Rx(5000);

    vTaskDelay(portMAX_DELAY);
}


void OnTxDone(){
    printf("Transmissão feita\n");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr){
    printf("Recepção feita\n Buffer: %s", payload);
    Radio.Send(Buffer,BufferSize);
}

void OnTxTimeout(){
    printf("timeout da transmissão :c\n");

}

void OnRxTimeout(){
    printf("timeout da recepção\n");
    Radio.Rx(5000);

}
