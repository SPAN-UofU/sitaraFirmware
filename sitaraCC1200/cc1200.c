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

/******************************************************************************
 * Local Macro Declarations                                                    * 
 ******************************************************************************/

// SPI for NRF
#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
 
static uint8_t buf[1024];
//static uint8_t tx_buf[1024];
static uint8_t rx_buf[1024];
static volatile bool spi_xfer_done;

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.\r\n");
    if (rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received: \r\n");
        NRF_LOG_HEXDUMP_INFO(rx_buf, strlen((const char *)rx_buf));
    }
}

int cc1200_cmd_strobe(uint8_t cmd)
{	
	/*struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	buf[transfer.len++] = cmd;

	// send the SPI message (all of the above fields, inc. buffers)
	return ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);*/

	static uint8_t len = 0;

	buf[len++] = cmd;

	NRF_LOG_HEXDUMP_INFO(buf, strlen((const char *)buf));
	spi_xfer_done = false;
	uint8_t status = nrf_drv_spi_transfer(&spi, buf, len, buf, len);
	 
	APP_ERROR_CHECK(status); // SPI TRANSFER
	while(!spi_xfer_done){} // wait
	NRF_LOG_FLUSH();

	return status;	
}

int
cc1200_get_status(uint8_t *status)
{
	
	/*struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};*/

	static uint8_t len = 0;

	buf[len++] = CC1200_SNOP;

	spi_xfer_done = false;
	uint8_t ret = nrf_drv_spi_transfer(&spi, buf, len, buf, len);
	APP_ERROR_CHECK(ret);
	while(!spi_xfer_done){} // wait
	NRF_LOG_FLUSH();
	// send the SPI message (all of the above fields, inc. buffers)
	//ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	
	if(ret < 0)
		return ret;
	else
		*status = buf[0];
	return ret;
}

int cc1200_write_register(uint16_t reg, uint8_t value)
{
	
	/*struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};*/

	// Reg
	static uint8_t len = 0;
	
	if (!CC1200_IS_EXTENDED_ADDR(reg)) {
		buf[len++] = CC1200_WRITE_BIT | reg;
		buf[len++] = value;
	} 
	// Extended Address
	else {
		buf[len++] = CC1200_WRITE_BIT | CC1200_EXT_REG_MASK;
		buf[len++] = CC1200_UNEXTEND_ADDR(reg);
		buf[len++] = value;
	}

	// send the SPI message (all of the above fields, inc. buffers)
	//ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);

	spi_xfer_done = false;
	uint8_t ret = nrf_drv_spi_transfer(&spi, buf, len, buf, len);
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
	
	/*struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};*/
	static uint8_t len = 0;

	memset(rx_buf, 0, len);

	// Reg
	if (!CC1200_IS_EXTENDED_ADDR(reg)) {
		buf[len++] = CC1200_READ_BIT | reg;
	} 
	// Extended Address
	else {
		buf[len++] = CC1200_READ_BIT | CC1200_EXT_REG_MASK;
		buf[len++] = CC1200_UNEXTEND_ADDR(reg);
	}

	buf[len++] = CC1200_SNOP;

	//ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	//while(!spi_xfer_done){}
	spi_xfer_done = false;
	
	NRF_LOG_INFO("Transfering: ");
	NRF_LOG_HEXDUMP_INFO(buf, len);

	uint8_t ret = nrf_drv_spi_transfer(&spi, buf, len, rx_buf, len);
	APP_ERROR_CHECK(ret);
	while(!spi_xfer_done){} // wait
	
	NRF_LOG_INFO("Received: ");
	NRF_LOG_HEXDUMP_INFO(rx_buf, len);

	NRF_LOG_FLUSH();

	if(ret < 0)
		return ret;
	else
		*data = rx_buf[len-1];

	// send the SPI message (all of the above fields, inc. buffers)
	return ret;
}

int cc1200_write_txfifo(uint8_t *data, uint8_t len)
{
	
	/*struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};*/
	static uint8_t length = 0;
	
	// Reg
	buf[length++] = CC1200_FIFO | CC1200_WRITE_BIT | CC1200_BURST_BIT;
	length += len;

	int j;
	for (j = 0; j < len; j++)
	{
		buf[j+1] = data[j];
	}

	// send the SPI message (all of the above fields, inc. buffers)
	//ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	spi_xfer_done = false;
	uint8_t ret = nrf_drv_spi_transfer(&spi, buf, length, buf, length);
	APP_ERROR_CHECK(ret);
	while(!spi_xfer_done){}
	NRF_LOG_FLUSH();

	return ret;
}

int cc1200_read_rxfifo(uint8_t *data, uint8_t len)
{
	
	/*struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};*/

	static uint8_t length = 0;
	
	buf[length++] = CC1200_FIFO | CC1200_READ_BIT | CC1200_BURST_BIT;
	length += len;

	//ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	spi_xfer_done = false;
	uint8_t ret = nrf_drv_spi_transfer(&spi, buf, length, buf, length);
	APP_ERROR_CHECK(ret);
	while(!spi_xfer_done){}
	NRF_LOG_FLUSH();

	if(ret < 0)
		return ret;
	else
	{
		int j;
		for (j = 0; j < length; j++)
		{
			data[j] = buf[j+1];
		}
	}

	// send the SPI message (all of the above fields, inc. buffers)
	return ret;
}

int cc1200_init(void)
{      
	
	uint8_t partnum = 0;
	uint8_t partver = 0;

	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_INFO("SPI example\r\n");

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

	/*// The following calls set up the CC1200 SPI bus properties
	if((fd = open(spi_path, O_RDWR))<0){
		perror("SPI Error: Can't open device.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_MODE, &mode)==-1){
		perror("SPI: Can't set SPI mode.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_MODE, &mode)==-1){
		perror("SPI: Can't get SPI mode.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)==-1){
		perror("SPI: Can't set bits per word.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits)==-1){
		perror("SPI: Can't get bits per word.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't set max speed HZ");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't get max speed HZ.");
		return -1;
	}
	
	// Check that the properties have been set
	printf("SPI Mode is: %d\n", mode);
	printf("SPI Bits is: %d\n", bits);
	printf("SPI Speed is: %d\n", speed);
	*/
	// Reset Radio
	cc1200_cmd_strobe(CC1200_SRES);

	// Get Chip Info
	cc1200_read_register(CC1200_PARTNUMBER, &partnum);
	cc1200_read_register(CC1200_PARTVERSION, &partver);
	NRF_LOG_INFO("CC1200 Chip Number: 0x%x Chip Version: 0x%x\r\n", partnum, partver);
	NRF_LOG_FLUSH();

	return 0;
}
