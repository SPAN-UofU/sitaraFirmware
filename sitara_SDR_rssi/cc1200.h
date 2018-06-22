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

#include <stdint.h>
#include <unistd.h>
#include "cc1200-rf-cfg.h"

//extern uint8_t rx_buf[1000];
int cc1200_init(void);
int cc1200_cmd_strobe(uint8_t cmd);
int cc1200_get_status(uint8_t *status);
int cc1200_write_register(uint16_t reg, uint8_t value);
void cc1200_write_reg_settings(const registerSetting_t *reg_settings,
		uint16_t sizeof_reg_settings);
int cc1200_read_register(uint16_t reg, uint8_t *data);
int cc1200_write_txfifo(uint8_t *data, uint8_t len);
int cc1200_read_rxfifo(uint8_t *data, uint8_t len);
int cc1200_burst_read_register(uint16_t reg, uint8_t *data, uint8_t read_bytes);
int cc1200_read_rx_buf(uint8_t *data);
int cc1200_ppi_spi_enable(uint16_t reg,uint8_t rx_len, uint8_t num_samples, uint8_t int_pin); // initializes and enables ppi_spi transfer
int cc1200_ppi_spi_disable(void); // disables and uninitializes ppi_spi transfer
uint32_t cc1200_get_rx_buffer(void); // returns pointer to the correct rx_buffer that's ready to process
int cc1200_rx_buffer_processed(void); // change buffer status after BLE processing
int cc1200_resume_PPI(void); // checks to see if PPI was waiting on BLE and re-enables if so
int cc1200_get_PPI_paused_count(void); // returns number of times PPI was paused and resets counter

