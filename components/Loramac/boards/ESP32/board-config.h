/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 *
 * \author    Marten Lootsma(TWTG) on behalf of Microchip/Atmel (c)2017
 */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME 1

/*!
 * Board MCU pins definitions
 */

#define RADIO_RESET GPIO_NUM_15
#define TCXO_PWR_PIN NC
#define RF_SWITCH_PIN NC

#define RADIO_MOSI GPIO_NUM_19
#define RADIO_MISO GPIO_NUM_23
#define RADIO_SCLK GPIO_NUM_18
#define RADIO_NSS GPIO_NUM_5

#define RADIO_DIO_0 GPIO_NUM_12
#define RADIO_DIO_1 GPIO_NUM_22
#define RADIO_DIO_2 GPIO_NUM_21
#define RADIO_DIO_3 NC

#define LED_1 NC

#define UART_TX GPIO_NUM_17
#define UART_RX GPIO_NUM_16
#define I2C_SDA NC
#define I2C_SCL NC

// Debug pins definition.
#define RADIO_DBG_PIN_TX NC
#define RADIO_DBG_PIN_RX NC

#ifdef __cplusplus
}
#endif

#endif   // __BOARD_CONFIG_H__
