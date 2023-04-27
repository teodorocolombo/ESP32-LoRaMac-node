#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "spi.h"
#include "driver/spi_master.h"


#include "esp_log.h"

spi_device_handle_t SpiHandle;

void SpiInit(
    Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss)
{
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .miso_io_num     = miso,
        .mosi_io_num     = mosi,
        .sclk_io_num     = sclk,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 64};

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,   // Default clock speed: 1MHz
        .mode           = 0,         // Default SPI mode: 0
        .spics_io_num   = nss,
        .queue_size     = 1,
      //yy  .flags          = SPI_DEVICE_HALFDUPLEX 
        };



    ret = spi_bus_initialize(spiId, &buscfg, 1);   // DMA channel 1
    assert(ret == ESP_OK);

    ret = spi_bus_add_device(spiId, &devcfg, &SpiHandle);
    assert(ret == ESP_OK);

    obj->SpiId    = spiId;
    obj->Mosi.pin = mosi;
    obj->Miso.pin = miso;
    obj->Sclk.pin = sclk;
    obj->Nss.pin  = nss;
}

void SpiDeInit(Spi_t *obj)
{
    spi_bus_remove_device(SpiHandle);
    spi_bus_free(obj->SpiId);

    gpio_reset_pin(obj->Mosi.pin);
    gpio_reset_pin(obj->Miso.pin);
    gpio_reset_pin(obj->Sclk.pin);
    gpio_reset_pin(obj->Nss.pin);
}

void SpiFormat(Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave)
{}

void SpiFrequency(Spi_t *obj, uint32_t hz)
{}

uint16_t SpiInOut(Spi_t *obj, uint16_t outData)
{
    uint16_t rx_buffer = 0;
    spi_transaction_t t = {
        .length    = 16,   // Transaction length is 16 bit
        .tx_buffer = &outData,
        .rx_buffer = &rx_buffer
        };


    esp_err_t ret = spi_device_polling_transmit(SpiHandle, &t);
    
    assert(ret == ESP_OK);


    return rx_buffer;
}
