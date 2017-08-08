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
/* Defines and register set */
#include "cc1200.h"
#include "cc1200-const.h"
#include "cc1200-rf-cfg.h"
//#include "nrf_delay.h"
/******************************************************************************
 * Local Macro Declarations                                                    * 
 ******************************************************************************/
// SPI for NRF
#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
 
uint8_t tx_buf[128];
uint8_t rx_buf[128];
volatile bool spi_xfer_done;


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
	NRF_LOG_HEXDUMP_INFO(rx_buf,rx_len);
	NRF_LOG_INFO(" status bit is %x", rx_buf[0]);
	NRF_LOG_FLUSH();

	if(ret < 0)
		return ret;
	else
		*status = rx_buf[0];
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
	NRF_LOG_HEXDUMP_INFO(rx_buf, rx_len);

	NRF_LOG_FLUSH();

	if(ret < 0)
		return ret;
	else
		*data = rx_buf[rx_len-1];
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
	NRF_LOG_INFO("size %d\r\n", tx_len);
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

