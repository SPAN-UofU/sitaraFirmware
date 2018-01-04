/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#ifndef SITARA_H
#define SITARA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

// LEDs definitions for SITARA based on PCA10056
#define LEDS_NUMBER    3

#define LED_1          NRF_GPIO_PIN_MAP(0,4)
#define LED_2          NRF_GPIO_PIN_MAP(0,6)
#define LED_3          NRF_GPIO_PIN_MAP(0,8)

#define LEDS_ACTIVE_STATE 0

#define LEDS_LIST { LED_1, LED_2, LED_3 }

#define LEDS_INV_MASK  LEDS_MASK

#define BSP_LED_0      4
#define BSP_LED_1      6
#define BSP_LED_2      8

#define BUTTONS_NUMBER 1

#define BUTTON_1       NRF_GPIO_PIN_MAP(1,9)
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

// #define RX_PIN_NUMBER  8
// #define TX_PIN_NUMBER  6
// #define CTS_PIN_NUMBER 7
// #define RTS_PIN_NUMBER 5
// #define HWFC           true

// #define BSP_QSPI_SCK_PIN   19
// #define BSP_QSPI_CSN_PIN   17
// #define BSP_QSPI_IO0_PIN   20
// #define BSP_QSPI_IO1_PIN   21
// #define BSP_QSPI_IO2_PIN   22
// #define BSP_QSPI_IO3_PIN   23


// serialization APPLICATION board - temp. setup for running serialized MEMU tests
// #define SER_APP_RX_PIN              NRF_GPIO_PIN_MAP(1,13)    // UART RX pin number.
// #define SER_APP_TX_PIN              NRF_GPIO_PIN_MAP(1,14)    // UART TX pin number.
// #define SER_APP_CTS_PIN             NRF_GPIO_PIN_MAP(0,2)     // UART Clear To Send pin number.
// #define SER_APP_RTS_PIN             NRF_GPIO_PIN_MAP(1,15)    // UART Request To Send pin number.

// #define SER_APP_SPIM0_SCK_PIN       NRF_GPIO_PIN_MAP(0,27)     // SPI clock GPIO pin number.
// #define SER_APP_SPIM0_MOSI_PIN      NRF_GPIO_PIN_MAP(0,2)      // SPI Master Out Slave In GPIO pin number
// #define SER_APP_SPIM0_MISO_PIN      NRF_GPIO_PIN_MAP(0,26)     // SPI Master In Slave Out GPIO pin number
// #define SER_APP_SPIM0_SS_PIN        NRF_GPIO_PIN_MAP(1,13)     // SPI Slave Select GPIO pin number
// #define SER_APP_SPIM0_RDY_PIN       NRF_GPIO_PIN_MAP(1,15)     // SPI READY GPIO pin number
// #define SER_APP_SPIM0_REQ_PIN       NRF_GPIO_PIN_MAP(1,14)     // SPI REQUEST GPIO pin number

// serialization CONNECTIVITY board
// #define SER_CON_RX_PIN              NRF_GPIO_PIN_MAP(1,14)    // UART RX pin number.
// #define SER_CON_TX_PIN              NRF_GPIO_PIN_MAP(1,13)    // UART TX pin number.
// #define SER_CON_CTS_PIN             NRF_GPIO_PIN_MAP(1,15)    // UART Clear To Send pin number. Not used if HWFC is set to false.
// #define SER_CON_RTS_PIN             NRF_GPIO_PIN_MAP(0,2)     // UART Request To Send pin number. Not used if HWFC is set to false.


// #define SER_CON_SPIS_SCK_PIN        NRF_GPIO_PIN_MAP(0,27)    // SPI SCK signal.
// #define SER_CON_SPIS_MOSI_PIN       NRF_GPIO_PIN_MAP(0,2)     // SPI MOSI signal.
// #define SER_CON_SPIS_MISO_PIN       NRF_GPIO_PIN_MAP(0,26)    // SPI MISO signal.
// #define SER_CON_SPIS_CSN_PIN        NRF_GPIO_PIN_MAP(1,13)    // SPI CSN signal.
// #define SER_CON_SPIS_RDY_PIN        NRF_GPIO_PIN_MAP(1,15)    // SPI READY GPIO pin number.
// #define SER_CON_SPIS_REQ_PIN        NRF_GPIO_PIN_MAP(1,14)    // SPI REQUEST GPIO pin number.

// #define SER_CONN_CHIP_RESET_PIN     NRF_GPIO_PIN_MAP(1,1)    // Pin used to reset connectivity chip

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#ifdef __cplusplus
}
#endif

#endif // SITARA_H
