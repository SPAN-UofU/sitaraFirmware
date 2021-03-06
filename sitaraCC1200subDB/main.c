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
#include "cc1200-802154g-434mhz-2gfsk-50kbps-cw.h"
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
#define CC1200_RF_CFG cc1200_802154g_434mhz_2gfsk_50kbps_cw

// static uint8_t tx_msg[] = {0x18, 0, 0, 'T', 'I', 'C', 'C', '1', '2', '0', '0', 'A', 'L'};
//static uint8_t rx_msg[ARR_SIZE(tx_msg)] = {0, };
//static const uint8_t m_length = sizeof(tx_msg);        /**< Transfer length. */

uint8_t spimsg[10] = {0};

uint8_t sample = false;

void magn_vadid_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	// Disable 
	nrf_drv_gpiote_in_event_disable(MAGN_VALID_PIN);
    
	// Handle
    // NRF_LOG_INFO("MAGN_VALID!\r\n");
	// uint8_t spimsg[5] = {0};
	//cc1200_burst_read_register(CC1200_MAGN2, spimsg, 5);
	// cc1200_read_register(CC1200_MAGN2, &test);
	// NRF_LOG_INFO("MAGN_VALID! %d\r\n", test++);
	// spimsg[0] = 0xDE;
	// spimsg[1] = 0xAD; 
	// spimsg[2] = 0xBE; 
	// spimsg[3] = 0xEF; 
	// spimsg[4] = test++;
	// // NRF_LOG_HEXDUMP_INFO(spimsg, 5);
	// NRF_LOG_FLUSH();
	sample = true;

	// Clear
	nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_IN_0);

	// Enable
	nrf_drv_gpiote_in_event_enable(MAGN_VALID_PIN, true);
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

	// bsp_board_leds_init();
	cc1200_init(); // Initialize cc1200
	gpio_init();

	// Write registers to radio
	cc1200_write_reg_settings(CC1200_RF_CFG.register_settings, CC1200_RF_CFG.size_of_register_settings);

	// Clear
	nrf_gpiote_event_clear(NRF_GPIOTE_EVENTS_IN_0);

	// Enable interrupt
	nrf_drv_gpiote_in_event_enable(MAGN_VALID_PIN, true);

	// static uint8_t status;

	#ifdef RX_MODE

		// uint8_t rxbytes;

		NRF_LOG_INFO("RX Mode!\r\n");

		// RX
		cc1200_cmd_strobe(CC1200_SRX);
	
		while(sample)
		{	
			// magn2 = spimsg[0];
			// magn1 = spimsg[1];
			// magn0 = spimsg[2];
			// ang1 = spimsg[3];
			// ang0 = spimsg[4];
			sample = false;
			cc1200_burst_read_register(CC1200_MAGN2, spimsg, 5);
			NRF_LOG_HEXDUMP_INFO(spimsg, 5);
			NRF_LOG_FLUSH();
		}

		// while(1)
		// {		
		// 	uint8_t rx_msg[ARRAY_SIZE(tx_msg)] = {0, };
		// 	cc1200_read_register(CC1200_MARC_STATUS1, &status);
		// 	// NRF_LOG_INFO(" status is %02x\r\n", status)
		// 	if(status == CC1200_MARC_STATUS1_RX_SUCCEED)
		// 	{	
		// 		cc1200_read_register(CC1200_NUM_RXBYTES, &rxbytes);
		// 		if (rxbytes != 0)
		// 		{
		// 			cc1200_read_register(CC1200_MARCSTATE, &status);
		// 			if ((status & 0x1F) == CC1200_MARC_STATE_RX_FIFO_ERR)
		// 			{
		// 				cc1200_cmd_strobe(CC1200_SFRX);
		// 			}
		// 			else
		// 			{
		// 				// NRF_LOG_INFO("In else");
		// 				int rx_fifo_bytes = rxbytes - 2;
		// 				cc1200_read_rxfifo(rx_msg, rx_fifo_bytes);

		// 				NRF_LOG_INFO("MSG Received! DATA: ");
		// 				NRF_LOG_HEXDUMP_INFO(tx_msg, sizeof(tx_msg));
						
		// 				// int i;
		// 				// for (i = 0; i < rx_fifo_bytes; i++)
						
		// 				NRF_LOG_INFO(" bytes: %d\r\n", rxbytes);

		// 				cc1200_cmd_strobe(CC1200_SFRX);
		// 			}
		// 		}

		// 		cc1200_cmd_strobe(CC1200_SRX);
		// 	}
		// 	nrf_delay_ms(1000);
		// }
	

	#else 
		NRF_LOG_INFO("TX Mode!\n");

		// int i;
		// for (i = 0; i < 3; i++)
		// while(1)
		{
			// tx_msg[1] = sizeof(tx_msg);
			// tx_msg[2]++;

			// // int i;
			// // for (i = 0; i < sizeof(tx_msg); i++)
			// //NRF_LOG_HEXDUMP_INFO(tx_msg, sizeof(tx_msg));

			// NRF_LOG_INFO(" size: %d\r\n", sizeof(tx_msg));

			// // Write data into FIFO
			// cc1200_write_txfifo(tx_msg, sizeof(tx_msg));

			// // Check status
			// cc1200_get_status(&status);
			// if ((status & 0xF0) == CC1200_STATUS_BYTE_TX_FIFO_ERR) {
			// 	NRF_LOG_INFO("cc1200 tx fifo error\r\n");
			// 	cc1200_cmd_strobe(CC1200_SFTX);
			// 	continue;
			// }

			// TX
			cc1200_cmd_strobe(CC1200_STX);
			while(1);
			
			// // Check if TX completed
			// cc1200_read_register(CC1200_MARC_STATUS1, &status);
			// while(status != CC1200_MARC_STATUS1_TX_SUCCEED)
			// {
			// 	cc1200_read_register(CC1200_MARC_STATUS1, &status);
			// 	NRF_LOG_INFO("status is %02x\r\n", status);
			// 	nrf_delay_ms(1000);
			// };

			// NRF_LOG_INFO("MSG SENT!\r\n");

			// cc1200_cmd_strobe(CC1200_SFTX);

			// nrf_delay_ms(1000);
		}
	#endif
	return 0;
}
