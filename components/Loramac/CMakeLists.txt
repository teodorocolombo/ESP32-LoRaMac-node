##
##   ______                              _
##  / _____)             _              | |
## ( (____  _____ ____ _| |_ _____  ____| |__
##  \____ \| ___ |    (_   _) ___ |/ ___)  _ \
##  _____) ) ____| | | || |_| ____( (___| | | |
## (______/|_____)_|_|_| \__)_____)\____)_| |_|
## (C)2013-2017 Semtech
##  ___ _____ _   ___ _  _____ ___  ___  ___ ___
## / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
## \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
## |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
## embedded.connectivity.solutions.==============
##
## License:  Revised BSD License, see LICENSE.TXT file included in the project
## Authors:  Johannes Bruder (STACKFORCE), Miguel Luis (Semtech)
##
cmake_minimum_required(VERSION 3.6)

#---------------------------------------------------------------------------------------
# Options
#---------------------------------------------------------------------------------------

# # Allow switching of target platform
# set(BOARD_LIST NAMote72 NucleoL073 NucleoL152 NucleoL476 SAMR34 SKiM880B SKiM980A SKiM881AXL B-L072Z-LRWAN1)
# set(BOARD ESP32 CACHE STRING "Default target platform is ESP32")
# set_property(CACHE BOARD PROPERTY STRINGS ${BOARD_LIST})

# # Allow switching of MBED shields
# set(MBED_RADIO_SHIELD_LIST SX1272MB2DAS SX1276MB1LAS SX1276MB1MAS SX1261MBXBAS SX1262MBXCAS SX1262MBXDAS )
# set(MBED_RADIO_SHIELD SX1262MBXCAS CACHE STRING "Default MBED radio shield is SX1261MBXBAS")
# set_property(CACHE MBED_RADIO_SHIELD PROPERTY STRINGS ${MBED_RADIO_SHIELD_LIST})

# # Allow switching of secure-elements
# set(SECURE_ELEMENT_LIST SOFT_SE LR1110_SE ATECC608A_TNGLORA_SE)
# set(SECURE_ELEMENT SOFT_SE CACHE STRING "Default secure element is SOFT_SE")
# set_property(CACHE SECURE_ELEMENT PROPERTY STRINGS ${SECURE_ELEMENT_LIST})

# # Allow switching of Applications
# set(APPLICATION_LIST LoRaMac ping-pong rx-sensi tx-cw )
# set(APPLICATION LoRaMac CACHE STRING "Default Application is LoRaMac")
# set_property(CACHE APPLICATION PROPERTY STRINGS ${APPLICATION_LIST})

# # Switch for USB-Uart support, enable it for some Applications who needs it.
# option(USE_USB_CDC "Use USB-Uart" OFF)

# # Switch for debugger support.
# option(USE_DEBUGGER "Use Debugger" OFF)

# # Switch for Class B support of LoRaMac.
# option(CLASSB_ENABLED "Class B support of LoRaMac" OFF)

#---------------------------------------------------------------------------------------
# Target Boards
#---------------------------------------------------------------------------------------


#if(BOARD STREQUAL ESP32)
    # Configure toolchain for SAMR34

    # Build platform specific board implementation
    #add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards/ESP32)

    # Configure radio
    # set(RADIO sx1276 CACHE INTERNAL "Radio sx1276 selected")
#endif()


#---------------------------------------------------------------------------------------
# General Components
#---------------------------------------------------------------------------------------

#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/boards)

#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/radio)

#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/system)

#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/mac)

#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/peripherals)

#---------------------------------------------------------------------------------------
# Applications
#---------------------------------------------------------------------------------------s
list(APPEND srcs

    #board/esp
    "boards/ESP32/board.c"
    "boards/ESP32/delay-board.c"
    "boards/ESP32/eeprom-board.c"
    "boards/ESP32/gpio-board.c"
    "boards/ESP32/rtc-board.c"
    "boards/ESP32/spi-board.c"
    "boards/ESP32/sx1276-board.c"
    "boards/ESP32/uart-board.c"
    "boards/ESP32/i2c-board.c"
    "boards/mcu/utilities.c"

    # #mac
    "mac/LoRaMacAdr.c"
    "mac/LoRaMac.c"
    "mac/LoRaMacClassB.c"
    "mac/LoRaMacCommands.c"
    "mac/LoRaMacConfirmQueue.c"
    "mac/LoRaMacCrypto.c"
    "mac/LoRaMacParser.c"
    "mac/LoRaMacSerializer.c"

    "mac/region/RegionAU915.c"
    "mac/region/RegionCommon.c"

    # #peripherals

    "peripherals/gpio-ioe.c"
    "peripherals/mag3110.c"
    "peripherals/mma8451.c"
    "peripherals/mpl3115.c"
    "peripherals/pam7q.c"
    "peripherals/sx1509.c"
    "peripherals/sx9500.c"

    "peripherals/soft-se/aes.c"
    "peripherals/soft-se/cmac.c"
    "peripherals/soft-se/soft-se.c"
    "peripherals/soft-se/soft-se-hal.c"

    # #radio
     "radio/sx1276/sx1276.c"

    # #system
    "system/adc.c"
    "system/delay.c"
    "system/fifo.c"
    "system/gpio.c"
    "system/gps.c"
    "system/i2c.c"
    "system/nvmm.c"
    "system/systime.c"
    "system/timer.c"
    "system/uart.c"


)

list(APPEND libs  
    "boards"
    "boards/ESP32"
    "mac"
    "mac/region"
    "peripherals"
    "radio"
    "radio/sx1276"
    "system"
)


idf_component_register( 
                       SRCS    
                       "${srcs}"     
                       INCLUDE_DIRS 
                       "${libs}"

                       REQUIRES
                       driver
                       esp_timer
)
