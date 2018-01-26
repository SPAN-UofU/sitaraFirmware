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
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <time.h>

/* Defines and register set */
#include "cc1200-const.h"
#include "cc1200-rf-cfg.h"
//#include "cc1200-802154g-434mhz-2gfsk-50kbps-cw.h"
#include "cc1200-802154g-868mhz-2gfsk-50kbps-cw.h"

// Standard header files
#include <sys/mman.h>
#include <errno.h>
#include <sys/time.h>
#include <pruss/prussdrv.h>
#include <pruss/pruss_intc_mapping.h>
#include "SPI_bin.h"

/******************************************************************************
 * Local Macro Declarations                                                    * 
 ******************************************************************************/
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define PRU_NUM 	0
#define OFFSET_SHAREDRAM 0		//equivalent with 0x00002000

#define PRUSS0_SHARED_DATARAM    4

// Standard
#define	HIGH							1
#define	LOW								0

#define SPI_PATH "/dev/spidev1.0"
#define ADC_PATH "/sys/bus/iio/devices/iio:device0/in_voltage5_raw"

/*---------------------------------------------------------------------------*/
/* RF configuration */
/*---------------------------------------------------------------------------*/
/* Import the rf configuration set by CC1200_RF_CFG */
extern const cc1200_rf_cfg_t CC1200_RF_CFG;
//#define CC1200_RF_CFG cc1200_802154g_434mhz_2gfsk_50kbps_cw
#define CC1200_RF_CFG cc1200_802154g_868mhz_2gfsk_50kbps_cw

/******************************************************************************
 * Global variable Declarations                                                * 
 ******************************************************************************/
static uint32_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 7700000;
static uint16_t delay = 1;

static void *sharedMem;
static unsigned int *sharedMem_int;
static struct IQSample *sharedMem_struct;

uint8_t default_tx[] = {0xAF, 0x8F, 0x3D, 0xAF, 0x90, 0x3D, };

uint8_t default_rx[ARRAY_SIZE(default_tx)] = {0, };

int fd;
FILE *adc;

uint8_t buf[1024];

struct IQSample {
	uint8_t status0;
	uint8_t status1;
	uint8_t magn2;
	uint8_t magn1;
	uint8_t magn0;
	uint8_t ang1;
	uint8_t ang0;
} iqsample;

struct timespec diff(struct timespec start, struct timespec end)
{
	struct timespec temp;
	if ((end.tv_nsec - start.tv_nsec) < 0) {
		temp.tv_sec = end.tv_sec - start.tv_sec - 1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec - start.tv_sec;
		temp.tv_nsec = end.tv_nsec - start.tv_nsec;
	}
	return temp;
}

/*
static void hex_dump(const void *src, size_t length, size_t line_size, char *prefix)
{
	int i = 0;
	const unsigned char *address = src;
	const unsigned char *line = address;
	unsigned char c;

	printf("%s | ", prefix);
	while (length-- > 0) {
		printf("%02X ", *address++);
		if (!(++i % line_size) || (length == 0 && i % line_size)) {
			if (length == 0) {
				while (i++ % line_size)
					printf("__ ");
			}
			printf(" | ");  // right close
			while (line < address) {
				c = *line++;
				printf("%c", (c < 33 || c == 255) ? 0x2E : c);
			}
			printf("\n");
			if (length > 0)
				printf("%s | ", prefix);
		}
	}
}

static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	// send the SPI message (all of the above fields, inc. buffers)
	int status = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	if(status < 0) {
		perror("SPI: SPI_IOC_MESSAGE Failed");
		abort();
	}

	//hex_dump(tx, len, 32, "TX");
	//hex_dump(rx, len, 32, "RX");
}
*/

/// TODO ///
static int cc1200_cmd_strobe(uint8_t cmd)
{
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	buf[transfer.len++] = cmd;

	// send the SPI message (all of the above fields, inc. buffers)
	return ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
}

/*
static int cc1200_get_status(uint8_t *status)
{
	int ret;
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	buf[transfer.len++] = CC1200_SNOP;

	// send the SPI message (all of the above fields, inc. buffers)
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	if(ret < 0)
		return ret;
	else
		*status = buf[0];
	return ret;
}
*/

static int cc1200_write_register(uint16_t reg, uint8_t value)
{
	int ret;
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	// Reg
	if (!CC1200_IS_EXTENDED_ADDR(reg)) {
		buf[transfer.len++] = CC1200_WRITE_BIT | reg;
		buf[transfer.len++] = value;
	} 
	// Extended Address
	else {
		buf[transfer.len++] = CC1200_WRITE_BIT | CC1200_EXT_REG_MASK;
		buf[transfer.len++] = CC1200_UNEXTEND_ADDR(reg);
		buf[transfer.len++] = value;
	}

	// send the SPI message (all of the above fields, inc. buffers)
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	
	return ret;
}

static void cc1200_write_reg_settings(const registerSetting_t *reg_settings,
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

static int cc1200_read_register(uint16_t reg, uint8_t *data)
{
	int ret;
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	// Reg
	if (!CC1200_IS_EXTENDED_ADDR(reg)) {
		buf[transfer.len++] = CC1200_READ_BIT | reg;
	} 
	// Extended Address
	else {
		buf[transfer.len++] = CC1200_READ_BIT | CC1200_EXT_REG_MASK;
		buf[transfer.len++] = CC1200_UNEXTEND_ADDR(reg);
	}

	buf[transfer.len++] = CC1200_SNOP;

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	if(ret < 0)
		return ret;
	else
		*data = buf[transfer.len-1];

	//hex_dump(buf, transfer.len, 32, "RX");

	// send the SPI message (all of the above fields, inc. buffers)
	return ret;
}

/*
static int cc1200_burst_read_register(uint16_t reg, uint8_t *data, uint8_t read_bytes)
{
	int ret;
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	// Reg
	if (!CC1200_IS_EXTENDED_ADDR(reg)) {
		buf[transfer.len++] = CC1200_READ_BIT | CC1200_BURST_BIT | reg;
	} 
	// Extended Address
	else {
		buf[transfer.len++] = CC1200_READ_BIT | CC1200_BURST_BIT | CC1200_EXT_REG_MASK;
		buf[transfer.len++] = CC1200_UNEXTEND_ADDR(reg);
	}

	transfer.len += read_bytes;
	//buf[transfer.len++] = CC1200_SNOP;

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	if(ret < 0)
		return ret;
	else
	{
		int j;
		for(j = 0; j < read_bytes; j++)
		{
			data[j] = buf[j+2];
		}
	}

	//hex_dump(buf, transfer.len, 32, "RX");

	// send the SPI message (all of the above fields, inc. buffers)
	return ret;
}
*/
/// END TODO ///

int main(int argc, char* argv[]){
	int i, j = 0;
	uint8_t partnum = 0;
	uint8_t partver = 0;
	uint8_t isTX = 0;
	int num_samples = 0;
	int samples_size = 100;

	if(argc != 2)
	{
		printf("TX MODE\n");
		//return 0;
		isTX = 1;
	}
	else
	{
		printf("RX MODE\n");
		num_samples = atoi(argv[1]);
			
		if(num_samples <= 0)
		{
			printf("usage: %s [samples]\n", argv[0]);
			printf("RX Mode: samples > 0\n");
			return 0;
		}
	}

	// The following calls set up the SPI bus properties
	if((fd = open(SPI_PATH, O_RDWR))<0){
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

	// Reset Radio
	cc1200_cmd_strobe(CC1200_SRES);

	// Get Chip Info
	cc1200_read_register(CC1200_PARTNUMBER, &partnum);
	cc1200_read_register(CC1200_PARTVERSION, &partver);
	printf("CC1200 Chip Number: 0x%x Chip Version: 0x%x\n", partnum, partver);

	// Write registers to radio
	cc1200_write_reg_settings(CC1200_RF_CFG.register_settings, CC1200_RF_CFG.size_of_register_settings);

	// T/RX
	if (!isTX) {
		cc1200_cmd_strobe(CC1200_SRX);

		///////// START PRU /////////
		unsigned int ret;
		tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
		struct timespec sample_time;

		/* Initializing PRU */
		prussdrv_init();
		ret = prussdrv_open(PRU_EVTOUT_0);
		if (ret){
			printf("\tERROR: prussdrv_open open failed\n");
			return (ret);
		}

		prussdrv_pruintc_init(&pruss_intc_initdata);
		prussdrv_map_prumem(PRUSS0_SHARED_DATARAM, &sharedMem);

		sharedMem_int = (unsigned int *) sharedMem;
		sharedMem_int[OFFSET_SHAREDRAM + 0] = samples_size; //samples

		/* Executing PRU. */
		prussdrv_pru_write_memory(PRUSS0_PRU0_IRAM, PRU_NUM, PRUcode, sizeof(PRUcode));

		for(j = 0;j < num_samples; j++)
		{
			prussdrv_pruintc_init(&pruss_intc_initdata);
			sharedMem_int[OFFSET_SHAREDRAM + 0] = samples_size; //samples
			prussdrv_pru_enable(PRU_NUM);

			ret = prussdrv_pru_wait_event(PRU_EVTOUT_0);

			sharedMem_struct = ( struct IQSample *) sharedMem;

			long unsigned int sum_magn = 0;
			double sum_power = 0;
			long int sum_ang = 0;
			int prev_phase = (sharedMem_struct[0].ang0&0xff) + ((sharedMem_struct[0].ang1<<8)&0xff00);

			for(i = 0; i < samples_size; i++) {
				long unsigned int mag = (sharedMem_struct[i].magn0&0xff) + ((sharedMem_struct[i].magn1<<8)&0xff00) + ((sharedMem_struct[i].magn2<<16)&0x010000);
				long unsigned int ang = (sharedMem_struct[i].ang0&0xff) + ((sharedMem_struct[i].ang1<<8)&0xff00);
				sum_magn += mag;
				sum_power += (double)mag * (double)mag;

				// delta phase
				int delta_phase = ang - prev_phase;
				prev_phase = ang;
				
				// Unwrap
				if(delta_phase < -512)
				{
					delta_phase+=1024;
				}
				else if(delta_phase > 512)
				{
					delta_phase-=1024;
				}

				sum_ang += delta_phase;

			}

			printf("%d ", j);
			printf("%lu %lf %ld", sum_magn, sum_power, sum_ang);

			// Print out AGC_GAIN3
			uint8_t agc_gain;
			cc1200_read_register(CC1200_AGC_GAIN3, &agc_gain);
			printf(" %d", agc_gain);

			// Read ripbelt
			/*
			char buff[64];
			if((adc = fopen(ADC_PATH, "r")) == NULL){
				return 1;
			}

			if((fgets(buff, 64, adc)) == NULL){
				printf(" NaN");
			}
			else
			{
				printf(" %d", atoi(buff));
			}
			fclose(adc);
			*/
			
			// Read time
			clock_gettime(CLOCK_REALTIME, &sample_time);
			printf(" %ld.%09lu\n", sample_time.tv_sec, sample_time.tv_nsec);

			/* Disable PRU*/
			prussdrv_pru_disable(PRU_NUM);
		}

		/* Exit PRU */
		prussdrv_exit();

		///////// END PRU /////////

	}
	else {
		cc1200_cmd_strobe(CC1200_STX);
	}

	close(fd);               //close the file
	return 0;
}
