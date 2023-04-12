#include "spi-board.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

typedef struct {
    spi_host_device_t   host;
    spi_device_handle_t handle;
} SpiDevice_t;

static SpiDevice_t spiDevice;

void SpiInit(
    Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss)
{
    spiDevice.host = spiId == SPI_1 ? SPI2_HOST : SPI3_HOST;

    spi_bus_config_t bus_config = {
        .miso_io_num     = miso,
        .mosi_io_num     = mosi,
        .sclk_io_num     = sclk,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 0};

    esp_err_t ret = spi_bus_initialize(spiDevice.host, &bus_config, 1);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "spi_bus_initialize() failed: %d", ret);
        return;
    }

    spi_device_interface_config_t device_config = {
        .clock_speed_hz = 1000000,   // 1 MHz
        .mode           = 0,
        .spics_io_num   = nss,
        .queue_size     = 1};

    ret = spi_bus_add_device(spiDevice.host, &device_config, &spiDevice.handle);
    if (ret != ESP_OK) {
        ESP_LOGE("SPI", "spi_bus_add_device() failed: %d", ret);
        return;
    }
}
