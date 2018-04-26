 /*
 * CC1200 Lib
 *
 * Copyright (C) 2016 University of Utah
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

#include "nrf_drv_spi.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "ble_nus.h"

/* Defines and register set */
#include "cc1200.h"
#include "cc1200-const.h"
#include "cc1200-rf-cfg.h"
#include "nrf_delay.h"
#include "boards.h"
/******************************************************************************
 * Local Macro Declarations                                                    * 
 ******************************************************************************/
// static ble_nus_t f_nus;                                      /**< Structure to identify the Nordic UART Service. */

// SPI for NRF
#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
#define ARR_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

uint8_t tx_buf[128] = {0};
uint8_t rx_buf[128] = {0};
static volatile bool spi_xfer_done;

// static int ble_data_queue(uint8_t *final_msg,uint8_t*spimsg)
// {
//     total_mag[index] = ((spimsg[0]<<16)&0xFF0000) + ((spimsg[1]<<8)&0x00FF00) + (spimsg[2]&0x0000FF);
//     final_msg[index+4] = spimsg[2];
//     final_msg[index+12] = spimsg[4];
//     average_mag +=  total_mag[index];
//     index++;
//     if(index == 8)
//     {
//     	temp = index;
//         index = 0;
//         average_mag = average_mag/ARR_SIZE(total_mag);
//         for(int k=0; k<8; k++)
//         {
//             if(CHECK_BIT(average_mag+(total_mag[k]&0x000000FF),17))
//             {
//                 average_mag |= 1<<(k+18);
//                 //NRF_LOG_INFO("check\r\n");
//             }                   
//         }
//         //NRF_LOG_INFO("avg %x\r\n",average_mag);
//         //NRF_LOG_FLUSH();
//         final_msg[0] = (average_mag >> 24) & 0xff;  // overflow 7 bits(lower 7), reserved
//         final_msg[1] = (average_mag >> 16) & 0xff; // average magnitude 1 bit(bit 0), overflow 7 bits
//         final_msg[2] = (average_mag >> 8) & 0xff; // average magnitude
//         final_msg[3] = (average_mag) & 0xff; // average magnitude
//         average_mag = 0;
//         memset(total_mag,0,sizeof(total_mag));
// 		NRF_LOG_HEXDUMP_INFO(final_msg, 20);
// 		// err_code = ble_nus_string_send(&f_nus,final_msg,20);
// 		return temp;
// 		//nrf_delay_ms(2000);
//     }
//     return index;        
// }

void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void *p_context)
{
    spi_xfer_done = true;
}

int cc1200_cmd_strobe(uint8_t cmd)
{	
	uint8_t tx_len = 0;
	uint8_t rx_len = 0;
	memset(tx_buf, 0, sizeof(tx_buf));
	memset(rx_buf, 0, sizeof(rx_buf));

	tx_buf[tx_len++] = cmd;
	rx_len = tx_len+1;

	//NRF_LOG_HEXDUMP_INFO(tx_buf, strlen((const char *)tx_buf));
	
	spi_xfer_done = false; 
	uint8_t status = nrf_drv_spi_transfer(&spi, tx_buf, tx_len, rx_buf, rx_len); // SPI TRANSFER
	APP_ERROR_CHECK(status);
	while(!spi_xfer_done){} // wait
	
	NRF_LOG_FLUSH();
	
	return 0;
}

int
cc1200_get_status(uint8_t *status)
{
	uint8_t tx_len = 0;
	uint8_t rx_len = 0;
	memset(tx_buf, 0, sizeof(tx_buf));
	memset(rx_buf, 0, sizeof(rx_buf));
	
	tx_buf[tx_len++] = CC1200_SNOP;
	rx_len = tx_len+1;

	spi_xfer_done = false;
	uint8_t ret = nrf_drv_spi_transfer(&spi, tx_buf, tx_len, rx_buf, rx_len);
	APP_ERROR_CHECK(ret);
	while(!spi_xfer_done){} // wait
	//NRF_LOG_HEXDUMP_INFO(rx_buf,rx_len);
	//NRF_LOG_INFO("status bit is %x \r\n", rx_buf[rx_len-1]);
	NRF_LOG_FLUSH();

	if(ret < 0)
		return ret;
	else
		*status = rx_buf[rx_len-1];
		//nrf_delay_ms(1000); // remove later
	return *status;
}

int cc1200_write_register(uint16_t reg, uint8_t value)
{	
	uint8_t tx_len = 0;
	uint8_t rx_len = 0;
	memset(tx_buf, 0, sizeof(tx_buf));
	memset(rx_buf, 0, sizeof(rx_buf));

	if (!CC1200_IS_EXTENDED_ADDR(reg)) {
		tx_buf[tx_len++] = CC1200_WRITE_BIT | reg;
		tx_buf[tx_len++] = value;
	} 
	// Extended Address
	else {
		tx_buf[tx_len++] = CC1200_WRITE_BIT | CC1200_EXT_REG_MASK;
		tx_buf[tx_len++] = CC1200_UNEXTEND_ADDR(reg);
		tx_buf[tx_len++] = value;
	}

	rx_len = tx_len;

	spi_xfer_done = false;
	uint8_t ret = nrf_drv_spi_transfer(&spi, tx_buf, tx_len, rx_buf, rx_len);
	APP_ERROR_CHECK(ret);
	while(!spi_xfer_done){} // wait

	NRF_LOG_FLUSH();

	return ret;
}

void cc1200_write_reg_settings(const registerSetting_t *reg_settings,
		uint16_t sizeof_reg_settings)
{
	int i = sizeof_reg_settings / sizeof(registerSetting_t);

	if(reg_settings != NULL) {
		while(i--) {
			cc1200_write_register(reg_settings->addr,
					reg_settings->val);
			reg_settings++;
		}
	}
}

int cc1200_read_register(uint16_t reg, uint8_t *data)
{
	uint8_t tx_len = 0;
	uint8_t rx_len = 0;
	memset(tx_buf, 0, sizeof(tx_buf));
	memset(rx_buf, 0, sizeof(rx_buf));

	// Reg
	if (!CC1200_IS_EXTENDED_ADDR(reg)) {
		tx_buf[tx_len++] = CC1200_READ_BIT | reg;
	} 
	// Extended Address
	else {
		tx_buf[tx_len++] = CC1200_READ_BIT | CC1200_EXT_REG_MASK;
		tx_buf[tx_len++] = CC1200_UNEXTEND_ADDR(reg);
	}

	tx_buf[tx_len++] = CC1200_SNOP;

	//NRF_LOG_INFO("Transfering:");
	//NRF_LOG_HEXDUMP_INFO(tx_buf, tx_len);

	rx_len = tx_len;

	spi_xfer_done = false;
	uint8_t ret = nrf_drv_spi_transfer(&spi, tx_buf, tx_len, rx_buf, rx_len);
	APP_ERROR_CHECK(ret);
	while(!spi_xfer_done){} // wait
	
	// NRF_LOG_INFO("Received");
	//NRF_LOG_HEXDUMP_INFO(rx_buf, rx_len);

	NRF_LOG_FLUSH();

	if(ret < 0)
		return ret;
	else
		*data = rx_buf[rx_len-1];
	return ret;
}
int cc1200_burst_read_register(uint16_t reg, uint8_t *data, uint8_t read_bytes)
{
	uint8_t tx_len = 0;
	uint8_t rx_len = 0;
	memset(tx_buf, 0, sizeof(tx_buf));
	memset(rx_buf, 0, sizeof(rx_buf));

	// Reg
	if (!CC1200_IS_EXTENDED_ADDR(reg)) {
		tx_buf[tx_len++] = CC1200_READ_BIT | CC1200_BURST_BIT | reg;
	} 
	// Extended Address
	else {
		tx_buf[tx_len++] = CC1200_READ_BIT | CC1200_BURST_BIT | CC1200_EXT_REG_MASK;
		tx_buf[tx_len++] = CC1200_UNEXTEND_ADDR(reg);
	}

	tx_len += read_bytes;
	rx_len = tx_len;
	
	spi_xfer_done = false;
	uint8_t ret = nrf_drv_spi_transfer(&spi, tx_buf, tx_len, rx_buf, rx_len);
	APP_ERROR_CHECK(ret);
	while(!spi_xfer_done){} // wait
	
	// NRF_LOG_INFO("Received");
	// NRF_LOG_HEXDUMP_INFO(rx_buf, rx_len);

	// NRF_LOG_FLUSH();

	if(ret < 0)
		return ret;
	else
	{
		int j;
		for (j = 2; j < rx_len; j++)
		{
			data[j-2] = rx_buf[j];
		}
	}

	// send the SPI message (all of the above fields, inc. buffers)
	return ret;
}


int cc1200_write_txfifo(uint8_t *data, uint8_t len)
{
	uint8_t tx_len = 0;
	uint8_t rx_len = 0;
	memset(tx_buf, 0, sizeof(tx_buf));
	memset(rx_buf, 0, sizeof(rx_buf));

	// Reg
	tx_buf[tx_len++] = CC1200_FIFO | CC1200_WRITE_BIT | CC1200_BURST_BIT;
	tx_len += len;

	int j;
	for (j = 0; j < len; j++)
	{
		tx_buf[j+1] = data[j];
	}
	
	NRF_LOG_HEXDUMP_INFO(tx_buf, tx_len);
	//NRF_LOG_INFO("size %d\r\n", tx_len);
	rx_len = tx_len;

	spi_xfer_done = false;
	uint8_t ret = nrf_drv_spi_transfer(&spi, tx_buf, tx_len, rx_buf, rx_len);
	APP_ERROR_CHECK(ret);
	while(!spi_xfer_done){}

	NRF_LOG_FLUSH();

	return ret;
}

int cc1200_read_rxfifo(uint8_t *data, uint8_t len)
{
	uint8_t tx_len = 0;
	uint8_t rx_len = 0;
	memset(tx_buf, 0, sizeof(tx_buf));
	memset(rx_buf, 0, sizeof(rx_buf));

	tx_buf[tx_len++] = CC1200_FIFO | CC1200_READ_BIT | CC1200_BURST_BIT;
	tx_len += len;
	rx_len = tx_len;

	spi_xfer_done = false;
	uint8_t ret = nrf_drv_spi_transfer(&spi, tx_buf, tx_len, rx_buf, rx_len);
	APP_ERROR_CHECK(ret);
	while(!spi_xfer_done){}

	NRF_LOG_FLUSH();

	if(ret < 0)
		return ret;
	else
	{
		int j;
		for (j = 0; j < rx_len; j++)
		{
			data[j] = rx_buf[j];
		}
	}
	return ret;
}


int cc1200_init(void)
{      
	
	uint8_t partnum = 0;
	uint8_t partver = 0;

	// Reset
	uint8_t reset_p = NRF_GPIO_PIN_MAP(0,29);
	nrf_gpio_cfg_output(reset_p);
	nrf_gpio_pin_write(reset_p, 1);

	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_INFO("SPI example\r\n");
    
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    //spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

	// Reset Radio
	cc1200_cmd_strobe(CC1200_SRES);

	// Get Chip Info
	cc1200_read_register(CC1200_PARTNUMBER, &partnum);
	cc1200_read_register(CC1200_PARTVERSION, &partver);
	
	NRF_LOG_INFO("CC1200 Chip Number: 0x%x Chip Version: 0x%x\r\n", partnum, partver);
	
	// Get BURST Chip Info
	uint8_t spimsg[10] = {0};
	cc1200_burst_read_register(CC1200_PARTNUMBER, spimsg, 2);
	NRF_LOG_HEXDUMP_INFO(spimsg, 2);
	NRF_LOG_INFO("CC1200 Chip Number: 0x%x Chip Version: 0x%x\r\n", spimsg[0], spimsg[1]);


	NRF_LOG_FLUSH();
	
	return 0;
}
