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
#include "cc1200-802154g-434mhz-2gfsk-50kbps.h"
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

/******************************************************************************
 * Local Macro Declarations                                                    * 
 ******************************************************************************/
#define ARR_SIZE(a) (sizeof(a) / sizeof((a)[0]))

// #define SPI_PATH 	"/dev/spidev2.0"


/*---------------------------------------------------------------------------*/
/* RF configuration */
/*---------------------------------------------------------------------------*/
/* Import the rf configuration set by CC1200_RF_CFG */

extern const cc1200_rf_cfg_t CC1200_RF_CFG;
#define CC1200_RF_CFG cc1200_802154g_434mhz_2gfsk_50kbps


uint8_t tx_msg[] = {0x18, 0, 0, 'T', 'I', 'C', 'C', '1', '2', '0', '0', 'A', 'L'};
// static uint8_t rx_msg[ARR_SIZE(tx_msg)] = {0, };
//static const uint8_t m_length = sizeof(tx_msg);        /**< Transfer length. */

void print_info()
{
	uint8_t partnum = 0;
	uint8_t partver = 0;

	// Get Chip Info
	cc1200_read_register(CC1200_PARTNUMBER, &partnum);
	cc1200_read_register(CC1200_PARTVERSION, &partver);

	NRF_LOG_INFO("CC1200 Chip Number: 0x%x Chip Version: 0x%x\r\n", partnum, partver);
	NRF_LOG_FLUSH();

	return;
}

/// MAIN ///
int main(void){

	cc1200_init(); // Initialize cc1200

	// Write registers to radio	
	cc1200_write_reg_settings(CC1200_RF_CFG.register_settings, CC1200_RF_CFG.size_of_register_settings);

	uint8_t status;

	#ifdef RX_MODE

		uint8_t rxbytes;

		NRF_LOG_INFO("RX Mode!\r\n");

		// RX
		cc1200_cmd_strobe(CC1200_SRX);
	
		while(1)
		{
			
			uint8_t rx_msg[ARRAY_SIZE(tx_msg)] = {0, };
			cc1200_read_register(CC1200_MARC_STATUS1, &status);
			// NRF_LOG_INFO(" status is %02x\r\n", status)
			if(status == CC1200_MARC_STATUS1_RX_SUCCEED)
			{	
				cc1200_read_register(CC1200_NUM_RXBYTES, &rxbytes);
				if (rxbytes != 0)
				{
					cc1200_read_register(CC1200_MARCSTATE, &status);
					if ((status & 0x1F) == CC1200_MARC_STATE_RX_FIFO_ERR)
					{
						cc1200_cmd_strobe(CC1200_SFRX);
					}
					else
					{
						// NRF_LOG_INFO("In else");
						int rx_fifo_bytes = rxbytes - 2;
						cc1200_read_rxfifo(rx_msg, rx_fifo_bytes);

						NRF_LOG_INFO("MSG Received! DATA: ");
						if(rx_msg != 0)
						{
							NRF_LOG_HEXDUMP_INFO(rx_msg, rx_fifo_bytes+1);
						}

						//int i;
						//for (i = 0; i < rx_fifo_bytes; i++)
						
						NRF_LOG_INFO(" bytes: %d\r\n", rx_fifo_bytes);

						cc1200_cmd_strobe(CC1200_SFRX);
					}
				}

				cc1200_cmd_strobe(CC1200_SRX);
			}
			//nrf_delay_ms(1000);
		}
	

	#else 
		NRF_LOG_INFO("TX Mode!\n");

		// int i;
		// for (i = 0; i < 3; i++)
		while(1)
		{
			tx_msg[1] = sizeof(tx_msg);
			//tx_msg[2]++;

			// int i;
			// for (i = 0; i < sizeof(tx_msg); i++)
			NRF_LOG_HEXDUMP_INFO(tx_msg, sizeof(tx_msg));

			//NRF_LOG_INFO(" size: %d\r\n", sizeof(tx_msg));

			// print_info();

			// cc1200_get_status(&status);

			// NRF_LOG_INFO(" status: %d\r\n", status);

			// Write data into FIFO
			cc1200_write_txfifo(tx_msg, sizeof(tx_msg));

			// // Check status
			cc1200_get_status(&status);
			if ((status & 0xF0) == CC1200_STATUS_BYTE_TX_FIFO_ERR) {
				NRF_LOG_INFO("cc1200 tx fifo error\r\n");
				cc1200_cmd_strobe(CC1200_SFTX);
				continue;
			}

			//print_info();

			// TX
			cc1200_cmd_strobe(CC1200_STX);

			// Check if TX completed
			cc1200_read_register(CC1200_MARC_STATUS1, &status);
			while(status != CC1200_MARC_STATUS1_TX_SUCCEED)
			{
				cc1200_read_register(CC1200_MARC_STATUS1, &status);
				//NRF_LOG_INFO("status is %02x\r\n", status);
				//nrf_delay_ms(1000);
			};

			NRF_LOG_INFO("MSG SENT!\r\n");

			cc1200_cmd_strobe(CC1200_SFTX);

			//nrf_delay_ms(1000);
		}
	#endif
	return 0;
}
