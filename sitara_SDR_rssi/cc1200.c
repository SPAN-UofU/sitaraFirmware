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
 * Originally Written by:
 * Anh Luong <luong@eng.utah.edu>
 *
 * Modified by:
 * Phillip Smith
 */

#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_spi.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_gpiote.h"
/* Defines and register set */
#include "cc1200.h"
#include "cc1200-const.h"
#include "cc1200-rf-cfg.h"

#include "boards.h"
/******************************************************************************
 * Local Macro Declarations                                                    * 
 ******************************************************************************/
#define GPIO2_INT_PIN NRF_GPIO_PIN_MAP(1,15)
#define GPIO3_INT_PIN NRF_GPIO_PIN_MAP(0,2)

// SPI for NRF
#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);

uint8_t tx_buf[128] = {0};
// in rx_buf's the last 5 bytes are intended to be used for storing clock and count information, not samples
uint8_t rx_buf[1005] = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";//{0};
uint8_t rx_buf2[1005] = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";//{0};
uint8_t rx_buf3[1005] = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";//{0};


uint32_t PPI_paused_counter;


#ifdef DEBUG
uint32_t test_buf[10] = {0}; // used to store test timer captures
uint8_t test_counter = 0; // used to index test_buf
uint8_t sample_counter = 1; // used to track number of buffers
#endif //DEBUG

static volatile bool spi_xfer_done;

// PPI channel setup for automatic spi transfer
nrf_ppi_channel_t ppi_channel_spi, ppi_channel_counter, ppi_channel_stop;
nrf_ppi_channel_group_t ppi_group_0;
// timer instance for use with ppi channel and spi count
#define PPI_SPI_RX_SAMPLE_LIMIT 81 //BLE_NUS_MAX_DATA_LEN/3 // compare condition for spi_counter -switch buffers after receiving this many samples
#define WAIT_TIME 1200000 // *1/16MHz, timer compare for PPI SPI with delay, this will determine sample rate
#define COUNTER_INSTANCE 1 // spi count timer index, don't use 0 or it will piss off softdevice
#define TIMER_INSTANCE 2 // spi timer index,
const nrf_drv_timer_t spi_counter = NRF_DRV_TIMER_INSTANCE(COUNTER_INSTANCE);
const nrf_drv_timer_t spi_timer = NRF_DRV_TIMER_INSTANCE(TIMER_INSTANCE);

typedef enum {PPI_WAITING, PPI_ACTIVE_BUFFER1, PPI_ACTIVE_BUFFER2, PPI_ACTIVE_BUFFER3} PPI_status_t;
typedef enum {PPI_BUSY, // currently receiving samples over PPI and loading into buffer
			  BLE_BUSY, // currently processing samples in buffer to prepare for TX
		      PPI_READY, // ready to receive new samples over PPI, BLE done using
		      BLE_READY}buffer_status_t; // ready to process for BLE, PPI already filled buffer
PPI_status_t PPI_status;
buffer_status_t rx_buf1_status, rx_buf2_status, rx_buf3_status;


void spi_counter_event_handler(nrf_timer_event_t event_type, void * p_context)
{
	switch(PPI_status)
	{
		case PPI_ACTIVE_BUFFER1: // PPI just finished rx_buf (buffer1)
			rx_buf1_status = BLE_READY;
			if(rx_buf2_status == PPI_READY)
			{
				NRF_SPIM0->RXD.PTR = (uint32_t)&rx_buf2; // switch to SPI to receive in buffer 2
				PPI_status = PPI_ACTIVE_BUFFER2;
				ret_code_t err_code = nrf_drv_ppi_group_enable(NRF_PPI_CHANNEL_GROUP0); // turn PPI/SPI back on!
#ifdef DEBUG
				nrf_drv_timer_capture(&spi_timer, NRF_TIMER_CC_CHANNEL1); // used to measure timing; starts when PPI SPI is enabled
				rx_buf[1001] = sample_counter%10; // used for testing
				rx_buf[1002] = (spi_timer.p_reg->CC[1]) >> 24;
				rx_buf[1003] = (spi_timer.p_reg->CC[1]) >> 16;
				rx_buf[1004] = (spi_timer.p_reg->CC[1]) >> 8;
				rx_buf[1005] = spi_timer.p_reg->CC[1];

#endif //DEBUG
				APP_ERROR_CHECK(err_code);

			}
			else
			{
				PPI_status = PPI_WAITING;
				//NRF_LOG_WARNING("PPI waiting on available buffer\n");
				PPI_paused_counter++;
			}
			break;

		case PPI_ACTIVE_BUFFER2: // PPI just finished rx_buf2 (buffer2)
			rx_buf2_status = BLE_READY;
			if(rx_buf3_status == PPI_READY)
			{
				NRF_SPIM0->RXD.PTR = (uint32_t)&rx_buf3; // switch to buffer 3
				PPI_status = PPI_ACTIVE_BUFFER3;
				ret_code_t err_code = nrf_drv_ppi_group_enable(NRF_PPI_CHANNEL_GROUP0); // turn PPI/SPI back on!
#ifdef DEBUG
				nrf_drv_timer_capture(&spi_timer, NRF_TIMER_CC_CHANNEL1);
				rx_buf2[1001] = sample_counter%10;
				rx_buf[1002] = (spi_timer.p_reg->CC[1]) >> 24;
				rx_buf[1003] = (spi_timer.p_reg->CC[1]) >> 16;
				rx_buf[1004] = (spi_timer.p_reg->CC[1]) >> 8;
				rx_buf[1005] = spi_timer.p_reg->CC[1];
#endif //DEBUG
				APP_ERROR_CHECK(err_code);

			}
			else
			{
				PPI_status = PPI_WAITING;
				//NRF_LOG_WARNING("PPI waiting on available buffer\n");
				PPI_paused_counter++;
			}
			break;

		case PPI_ACTIVE_BUFFER3: // PPI just finished rx_buf3 (buffer3)
			rx_buf3_status = BLE_READY;
			if(rx_buf1_status == PPI_READY)
			{
				NRF_SPIM0->RXD.PTR = (uint32_t)&rx_buf; // switch to buffer 1
				PPI_status = PPI_ACTIVE_BUFFER1;
				ret_code_t err_code = nrf_drv_ppi_group_enable(NRF_PPI_CHANNEL_GROUP0); // turn PPI/SPI back on!
#ifdef DEBUG
				nrf_drv_timer_capture(&spi_timer, NRF_TIMER_CC_CHANNEL1); // used to test timing
				rx_buf3[1001] = sample_counter%10; // used for testing
				rx_buf[1002] = (spi_timer.p_reg->CC[1]) >> 24;
				rx_buf[1003] = (spi_timer.p_reg->CC[1]) >> 16;
				rx_buf[1004] = (spi_timer.p_reg->CC[1]) >> 8;
				rx_buf[1005] = spi_timer.p_reg->CC[1];
#endif //DEBUG
				APP_ERROR_CHECK(err_code);

			}
			else
			{
				PPI_status = PPI_WAITING;
				//NRF_LOG_WARNING("PPI waiting on available buffer\n");
				PPI_paused_counter++;
			}
			break;
		case PPI_WAITING:
			NRF_LOG_ERROR("PPI_WAITING status encountered in event handler.\n"); // this shouldn't happen
			break;
		default:
			NRF_LOG_ERROR("INVALID PPI STATUS: 0x%X - something has gone terribly wrong.\n", PPI_status);
			break;
	}


#ifdef DEBUG	// used for testing-- measuring timing
	nrf_drv_timer_capture(&spi_timer, NRF_TIMER_CC_CHANNEL2); // used to test timing
	test_buf[test_counter++] = spi_timer.p_reg->CC[1];
	test_buf[test_counter++] = spi_timer.p_reg->CC[2];
	test_counter %=10; //don't want to exceed index..r
	sample_counter++;
#endif //DEBUG
}

void spi_timer_event_handler(nrf_timer_event_t event_type, void * p_context)
{
	//not used
}


void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void *p_context)
{
	spi_xfer_done = true;
	NRF_LOG_DEBUG("Transfer completed.\r\n");
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

int cc1200_get_status(uint8_t *status)
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
	nrf_drv_spi_xfer_desc_t xfer = NRF_DRV_SPI_SINGLE_XFER(tx_buf, tx_len, rx_buf, rx_len); //NRF_DRV_SPI_XFER_RX(rx_buf,5);
	uint32_t flags = 0 /*NRF_DRV_SPI_FLAG_HOLD_XFER 	|
			     NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER	|
			     NRF_DRV_SPI_FLAG_RX_POSTINC*/;
		ret_code_t ret = nrf_drv_spi_xfer(&spi, &xfer, flags);
	//uint8_t ret = nrf_drv_spi_transfer(&spi, tx_buf, tx_len, rx_buf, rx_len);
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

int cc1200_read_rx_buf(uint8_t *data)
{
	int j;
	for (j = 0; j < 5; j++)
	{
		data[j] = rx_buf[j];
	}
	return 0;
}

/* ------------------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------------------------------------------------- */

int cc1200_ppi_spi_enable(uint16_t reg, uint8_t rx_len, uint8_t num_samples, uint8_t int_pin)
{
	ret_code_t err_code;
	uint8_t tx_len = 0;
	memset(tx_buf, 0, sizeof(tx_buf));
	memset(rx_buf, 'X', sizeof(rx_buf));
	memset(rx_buf2, 'X', sizeof(rx_buf2));

	// Reg
	if (!CC1200_IS_EXTENDED_ADDR(reg)) {
		tx_buf[tx_len++] = CC1200_READ_BIT | CC1200_BURST_BIT | reg;
	}
	// Extended Address
	else {
		tx_buf[tx_len++] = CC1200_READ_BIT | CC1200_BURST_BIT | CC1200_EXT_REG_MASK;
		tx_buf[tx_len++] = CC1200_UNEXTEND_ADDR(reg);
	}

	/*-----------------------------------------*/
	/*-----------------------------------------*/
	rx_buf1_status = PPI_READY;
	rx_buf2_status = PPI_READY;
	rx_buf3_status = PPI_READY;

	nrf_drv_gpiote_in_event_disable(int_pin); //disable interrupts for a bit
	// initialize control for SPI SS (CSn) pin
	nrf_drv_gpiote_out_config_t CSn_config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(true); // initialize high (SPI disabled)
	err_code = nrf_drv_gpiote_out_init(SPI_SS_PIN, &CSn_config);
	nrf_drv_gpiote_out_task_enable(SPI_SS_PIN);

	// TIMER for testing now
	nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG; //16 MHz default
	timer_cfg.mode = NRF_TIMER_MODE_TIMER;
	timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
	err_code = nrf_drv_timer_init(&spi_timer, &timer_cfg, spi_timer_event_handler);
	APP_ERROR_CHECK(err_code);
	// set up timer compare and trigger an event when spi_counter reaches WAIT_TIME
	//nrf_drv_timer_compare(&spi_timer, NRF_TIMER_CC_CHANNEL0, WAIT_TIME, false);
	nrf_drv_timer_extended_compare(&spi_timer, NRF_TIMER_CC_CHANNEL0, WAIT_TIME,NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);


	NRF_LOG_DEBUG("timer initialized and compare interrupt enabled\n");

	// SPI counter for use in PPI
	timer_cfg.mode = NRF_TIMER_MODE_COUNTER;
	err_code = nrf_drv_timer_init(&spi_counter, &timer_cfg, spi_counter_event_handler);
	APP_ERROR_CHECK(err_code);
	// set up timer compare and trigger an event when spi_counter reaches PPI_SPI_RX_BUF
	//nrf_drv_timer_compare(&spi_counter, NRF_TIMER_CC_CHANNEL1, PPI_SPI_RX_BUF, true);
	nrf_drv_timer_extended_compare(&spi_counter, NRF_TIMER_CC_CHANNEL0, num_samples, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
	NRF_LOG_DEBUG("timer initialized and compare interrupt enabled\n");

	// set up ppi channels and such
	err_code = nrf_drv_ppi_init();
	APP_ERROR_CHECK(err_code);
	// allocate ppi channels
	err_code = nrf_drv_ppi_channel_alloc(&ppi_channel_spi);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_ppi_channel_alloc(&ppi_channel_counter);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_ppi_channel_alloc(&ppi_channel_stop);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_ppi_group_alloc(&ppi_group_0);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_ppi_channel_include_in_group(ppi_channel_spi,NRF_PPI_CHANNEL_GROUP0);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_ppi_channel_include_in_group(ppi_channel_counter,NRF_PPI_CHANNEL_GROUP0);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_ppi_channel_include_in_group(ppi_channel_stop,NRF_PPI_CHANNEL_GROUP0);
	APP_ERROR_CHECK(err_code);

	nrf_drv_spi_xfer_desc_t xfer = NRF_DRV_SPI_SINGLE_XFER(tx_buf, tx_len,rx_buf, rx_len); 
	uint32_t flags = NRF_DRV_SPI_FLAG_HOLD_XFER         	|
		NRF_DRV_SPI_FLAG_NO_XFER_EVT_HANDLER	|
							  NRF_DRV_SPI_FLAG_RX_POSTINC;
		ret_code_t ret = nrf_drv_spi_xfer(&spi, &xfer, flags);
	NRF_LOG_DEBUG("spi xfer set with return code: 0x%x\n",ret);
	if (ret == NRF_SUCCESS)
	{
		// need to set up ppi to trigger transfer here, this should be when sample is ready..
		err_code = nrf_drv_ppi_channel_assign(ppi_channel_spi,
				nrf_drv_gpiote_in_event_addr_get(int_pin), //input/trigger event address
				nrf_drv_spi_start_task_get(&spi)); // spi transfer start task address
		APP_ERROR_CHECK(err_code);
		nrf_drv_ppi_channel_fork_assign(ppi_channel_spi, nrf_drv_gpiote_out_task_addr_get(SPI_SS_PIN)); // toggle CSN to low

		err_code = nrf_drv_ppi_channel_assign(ppi_channel_counter,
				nrf_drv_spi_end_event_get(&spi), //input/trigger event address
				nrf_drv_timer_task_address_get(&spi_counter, NRF_TIMER_TASK_COUNT)); // increment spi_counter
		APP_ERROR_CHECK(err_code);
		nrf_drv_ppi_channel_fork_assign(ppi_channel_counter, nrf_drv_gpiote_out_task_addr_get(SPI_SS_PIN)); // toggle CSN to high


		err_code = nrf_drv_ppi_channel_assign(ppi_channel_stop,
				nrf_drv_timer_compare_event_address_get(&spi_counter,NRF_TIMER_CC_CHANNEL0), // stops after counter reaches set compare
				nrf_drv_ppi_task_addr_group_disable_get(NRF_PPI_CHANNEL_GROUP0));
		APP_ERROR_CHECK(err_code);
		nrf_drv_ppi_channel_fork_assign(ppi_channel_stop, nrf_drv_timer_capture_get(&spi_timer,NRF_TIMER_CC_CHANNEL2)); //used to test timing..
		NRF_LOG_DEBUG("ppi channels defined\n");

		ret_code_t err_code = nrf_drv_ppi_group_enable(NRF_PPI_CHANNEL_GROUP0); // turn three PPI channels on
		APP_ERROR_CHECK(err_code);

		NRF_LOG_DEBUG("ppi channels enabled!\n");

	}
	else
		return 1;

	rx_buf1_status = PPI_BUSY; // we configure PPI to start on rx_buf (i.e., buffer1)
	PPI_status = PPI_ACTIVE_BUFFER1; // we configure PPI to start on rx_buf (i.e., buffer1)


	nrf_drv_timer_enable(&spi_counter);
	nrf_drv_timer_enable(&spi_timer); //this should start everything.. // this isn't used.

	nrf_drv_gpiote_in_event_enable(int_pin, false); // enable event but not interrupt, this begins sampling

	NRF_LOG_DEBUG("timers and  interrupt enabled!\n");

	return 0;

}

int cc1200_ppi_spi_disable(void)
{
	nrf_drv_ppi_uninit();
	nrf_drv_gpiote_out_uninit(SPI_SS_PIN);
	nrf_drv_timer_uninit(&spi_counter);
	nrf_drv_timer_uninit(&spi_timer);
	nrf_drv_gpiote_in_event_disable(GPIO2_INT_PIN); //disable interrupts for a bit
	nrf_drv_gpiote_in_event_disable(GPIO3_INT_PIN); //disable interrupts for a bit

	// setting SPI_SS_PIN for gpiote breaks other SPI transfers, so a temporary fix is to reset all SPI stuff
	nrf_drv_spi_uninit(&spi);
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin   = SPI_SS_PIN;
	spi_config.miso_pin = SPI_MISO_PIN;
	spi_config.mosi_pin = SPI_MOSI_PIN;
	spi_config.sck_pin  = SPI_SCK_PIN;
	//spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
	APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));


	return 0;
}

uint32_t cc1200_get_rx_buffer() // returns pointer to the correct rx_buffer that's ready to process
{
	uint32_t data;

	if(rx_buf1_status == BLE_READY)
	{
		data = (uint32_t)&rx_buf;   // now BLE processing buffer1
		rx_buf1_status = BLE_BUSY;
		return data;
	}
	else if(rx_buf2_status == BLE_READY)
	{
		data = (uint32_t)&rx_buf2;
		rx_buf2_status = BLE_BUSY;
		return data;
	}
	else if(rx_buf3_status == BLE_READY)
	{
		data = (uint32_t)&rx_buf3;
		rx_buf3_status = BLE_BUSY;
		return data;
	}
	//else if (PPI_status == PPI_WAITING)
	//	NRF_LOG_ERROR("ERROR: PPI WAITING but BLE not processing!\n");
	return 0; // we assume this means PPI is still processing and we need to wait

	/*
	if (NRF_SPIM0->RXD.PTR < (uint32_t)&rx_buf2)
		data = (uint32_t)&rx_buf2;
	else
		data = (uint32_t)&rx_buf;

	return data;
	*/
}

int cc1200_rx_buffer_processed()
{

	if(rx_buf1_status == BLE_BUSY)
	{
		rx_buf1_status = PPI_READY;
		return 0;
	}
	if(rx_buf2_status == BLE_BUSY)
	{
		rx_buf2_status = PPI_READY;
		return 0;
	}
	if(rx_buf3_status == BLE_BUSY)
	{
		rx_buf3_status = PPI_READY;
		return 0;
	}


	NRF_LOG_ERROR("BLE BUSY was never assigned during BLE processing!\n");
	return 1;
}

int inline cc1200_resume_PPI() // this needs to be fast.
{
	if(PPI_status != PPI_WAITING) // put this first to get of function quickly if not needed
		return 0;
	if(rx_buf1_status == PPI_READY)
	{
		NRF_SPIM0->RXD.PTR = (uint32_t)&rx_buf; // switch to buffer 1
		PPI_status = PPI_ACTIVE_BUFFER1;
		ret_code_t err_code = nrf_drv_ppi_group_enable(NRF_PPI_CHANNEL_GROUP0); // turn PPI/SPI back on!
		nrf_drv_timer_capture(&spi_timer, NRF_TIMER_CC_CHANNEL1); // used to determine gap that PPI was paused
		APP_ERROR_CHECK(err_code);
#ifdef DEBUG
		rx_buf[500] = sample_counter%10; // used for testing
#endif
		return spi_timer.p_reg->CC[1]-spi_timer.p_reg->CC[2];
	}
	else if(rx_buf2_status == PPI_READY)
	{
		NRF_SPIM0->RXD.PTR = (uint32_t)&rx_buf2; // switch to buffer 2
		PPI_status = PPI_ACTIVE_BUFFER2;
		ret_code_t err_code = nrf_drv_ppi_group_enable(NRF_PPI_CHANNEL_GROUP0); // turn PPI/SPI back on!
		nrf_drv_timer_capture(&spi_timer, NRF_TIMER_CC_CHANNEL1); // used to determine gap that PPI was paused
		APP_ERROR_CHECK(err_code);
#ifdef DEBUG
		rx_buf2[500] = sample_counter%10; // used for testing
#endif
		return spi_timer.p_reg->CC[1]-spi_timer.p_reg->CC[2];
	}
	else if(rx_buf3_status == PPI_READY)
	{
		NRF_SPIM0->RXD.PTR = (uint32_t)&rx_buf3; // switch to buffer 3
		PPI_status = PPI_ACTIVE_BUFFER3;
		ret_code_t err_code = nrf_drv_ppi_group_enable(NRF_PPI_CHANNEL_GROUP0); // turn PPI/SPI back on!
		nrf_drv_timer_capture(&spi_timer, NRF_TIMER_CC_CHANNEL1); // used to determine gap that PPI was paused
		APP_ERROR_CHECK(err_code);
#ifdef DEBUG
		rx_buf3[500] = sample_counter%10; // used for testing
#endif
		return spi_timer.p_reg->CC[1]-spi_timer.p_reg->CC[2];
	}
	return 0;
}

int cc1200_get_PPI_paused_count()
{
	int tmp;

	tmp = PPI_paused_counter;
	PPI_paused_counter = 0;
	return tmp;
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

	//NRF_LOG_INFO("CC1200 Chip Number: 0x%x Chip Version: 0x%x\r\n", partnum, partver);

	// Get BURST Chip Info
	uint8_t spimsg[10] = {0};
	cc1200_burst_read_register(CC1200_PARTNUMBER, spimsg, 2);
	//NRF_LOG_HEXDUMP_INFO(spimsg, 2);
	NRF_LOG_INFO("CC1200 Chip Number: 0x%x Chip Version: 0x%x\r\n", spimsg[0], spimsg[1]);


	NRF_LOG_FLUSH();

	return 0;
}
