
/* loramac includes */

#include "board.h"
#include "adc.h"
#include "delay.h"
#include "gpio.h"
#include "gps.h"
#include "i2c.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"
#include "utilities.h"
#include "board-config.h"
#include "sx1276-board.h"
#include "rtc-board.h"

/* esp includes */
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_system.h"

/* std includes */
#include <string.h>

// /*
//  * MCU objects
//  */
Gpio_t Led1;
Uart_t Uart1;
I2c_t  I2c;

// /*!
//  * Flag to indicate if the MCU is Initialized
//  */
static bool McuInitialized = false;

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

void BoardCriticalSectionBegin(uint32_t *mask)
{
    taskENTER_CRITICAL(&spinlock);
}

void BoardCriticalSectionEnd(uint32_t *mask)
{
    taskEXIT_CRITICAL(&spinlock);
}

void BoardInitPeriph(void)
{
   // GpioInit(&Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
}

void BoardInitMcu(void)
{

    esp_timer_init();

    RtcInit();
  
    UartInit(&Uart1, UART_1, UART_TX, UART_RX);
    UartConfig(&Uart1, RX_TX, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY,
              NO_FLOW_CTRL);

    SpiInit(&SX1276.Spi, SPI_2, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, RADIO_NSS);
    SX1276IoInit();
 //   I2cInit(&I2c, I2C_1, I2C_SCL, I2C_SDA);

    McuInitialized = true;
   // SX1276IoDbgInit();
  //  SX1276IoTcxoInit();
}

void BoardResetMcu(void)
{
    esp_restart();
}

void BoardDeInitMcu(void)
{
    SpiDeInit(&SX1276.Spi);
}

uint32_t BoardGetRandomSeed(void)
{
    return esp_random();
}

void BoardGetUniqueId(uint8_t *id)
{
    uint8_t chip_id[8];
    esp_efuse_mac_get_default(&chip_id);

    memcpy(id, &chip_id, 8);
}

uint8_t BoardGetBatteryLevel(void)
{
    return 0;   //  Battery level [0: node is connected to an external power
///    source ...
}

uint8_t GetBoardPowerSource(void)
{
    return USB_POWER;
}

void BoardLowPowerHandler(void)
{
    // Implement low power handling for ESP32
}

#if !defined(__CC_ARM)

// /*
//  * Function to be used by stdout for printf etc
//  */
int _write(int fd, const void *buf, size_t count)
{
    while (UartPutBuffer(&Uart1, (uint8_t *)buf, (uint16_t)count) != 0) {};
    return count;
}

// /*
//  * Function to be used by stdin for scanf etc
//  */
int _read(int fd, const void *buf, size_t count)
{
    size_t bytesRead = 0;
    while (UartGetBuffer(&Uart1, (uint8_t *)buf, count, (uint16_t
    *)&bytesRead) != 0) {
    };
    // Echo back the character
    while (UartPutBuffer(&Uart1, (uint8_t *)buf, (uint16_t)bytesRead) != 0) {
    };
    return bytesRead;
}
#else

    #include <stdio.h>

// Keil compiler
int fputc(int c, FILE *stream)
{
    while (UartPutChar(&Uart1, (uint8_t)c) != 0)
        ;
    return c;
}

int fgetc(FILE *stream)
{
    uint8_t c = 0;
    while (UartGetChar(&Uart1, &c) != 0)
    ;
    // Echo back the character
    while (UartPutChar(&Uart1, c) != 0)
    ;
    return (int)c;
}

#endif

#ifdef USE_FULL_ASSERT

    #include <stdio.h>

// /*
//  * Function Name  : assert_failed
//  * Description    : Reports the name of the source file and the source line
//  number
//  *                  where the assert_param error has occurred.
//  * Input          : - file: pointer to the source file name
//  *                  - line: assert_param error line source number
//  * Output         : None
//  * Return         : None
//  */
void assert_failed(uint8_t file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line
    number,
ex: printf("Wrong parameters value: file %s on line %u\n", file, line) */

    printf("Wrong parameters value: file %s on line %u\n", (const char
    *)file, line);
    /* Infinite loop */
    while (1) {
    }
}
#endif