 /*
 * CC1200 CW Test Application
 *
 * Copyright (C) 2015 University of Utah
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Written by:
 * Anh Luong <luong@eng.utah.edu>
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <linux/types.h>
#include <time.h>

/* Defines and register set */
#include "cc1200-const.h"
#include "cc1200-rf-cfg.h"
#include "cc1200-802154g-868mhz-2gfsk-50kbps-cw.h"
#include "cc1200.h"

#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"

/******************************************************************************
 * Local Macro Declarations                                                    * 
 ******************************************************************************/
#define ARR_SIZE(a) (sizeof(a) / sizeof((a)[0]))

// #define SPI_PATH 	"/dev/spidev2.0"

#define MAGN_VALID_PIN NRF_GPIO_PIN_MAP(1,15)

/*---------------------------------------------------------------------------*/
/* RF configuration */
/*---------------------------------------------------------------------------*/
/* Import the rf configuration set by CC1200_RF_CFG */

extern const cc1200_rf_cfg_t CC1200_RF_CFG;
#define CC1200_RF_CFG cc1200_802154g_868mhz_2gfsk_50kbps_cw
const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(0);

// static uint8_t tx_msg[] = {0x18, 0, 0, 'T', 'I', 'C', 'C', '1', '2', '0', '0', 'A', 'L'};
//static uint8_t rx_msg[ARR_SIZE(tx_msg)] = {0, };
//static const uint8_t m_length = sizeof(tx_msg);        /**< Transfer length. */

uint8_t spimsg[10] = {0};
uint8_t sample = false;
int count = 0;

void magn_vadid_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	// Disable 
	nrf_drv_gpiote_in_event_disable(MAGN_VALID_PIN);
	sample = true;
	count++;
	// Clear
	nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_IN_0);
	// Enable
	nrf_drv_gpiote_in_event_enable(MAGN_VALID_PIN, true);
}

void timer_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
        	nrf_drv_gpiote_in_event_disable(MAGN_VALID_PIN);
            break;
        default:
            //Do nothing.
            break;
    }
}

/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t magn_valid_pin_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    magn_valid_pin_config.pull = NRF_GPIO_PIN_NOPULL;

    err_code = nrf_drv_gpiote_in_init(MAGN_VALID_PIN, &magn_valid_pin_config, magn_vadid_handler);
    APP_ERROR_CHECK(err_code);
}

/// MAIN ///
int main(void){

    uint32_t time_ms = 1000; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
    //Configure all leds on board.
    bsp_board_leds_init();

	cc1200_init(); // Initialize cc1200
	gpio_init();

	// Write registers to radio
	cc1200_write_reg_settings(CC1200_RF_CFG.register_settings, CC1200_RF_CFG.size_of_register_settings);

	// Clear
	nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_IN_0);

	// Enable interrupt
	nrf_drv_gpiote_in_event_enable(MAGN_VALID_PIN, true);

	// static uint8_t status;

    //Configure all leds on board.
    bsp_board_leds_init();


	#ifdef RX_MODE

		// uint8_t rxbytes;

		NRF_LOG_INFO("RX Mode!\r\n");

		// RX
		cc1200_cmd_strobe(CC1200_SRX);
	    //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
	    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
	    err_code = nrf_drv_timer_init(&TIMER_LED, &timer_cfg, timer_handler);
	    APP_ERROR_CHECK(err_code);
	    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_LED, time_ms);
	    nrf_drv_timer_extended_compare(
	         &TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
	    nrf_drv_timer_enable(&TIMER_LED);
	
		while(sample)
		{	

			sample = false;
			cc1200_burst_read_register(CC1200_MAGN2, spimsg, 5);
			spimsg[6]++;
			NRF_LOG_HEXDUMP_INFO(spimsg, 6);
			NRF_LOG_FLUSH();
		}
		NRF_LOG_INFO("count is %d",count);
        NRF_LOG_FLUSH();
	#else 
		NRF_LOG_INFO("TX Mode!\n");
		// TX
		cc1200_cmd_strobe(CC1200_STX);
		while(1);
	#endif
	return 0;
}
