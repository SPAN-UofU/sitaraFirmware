/*
 * platform.c
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

#include "platform.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
 #include "nrf_gpio.h"
#include <string.h>
//#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "deca_regs.h"

#define SPI_SPEED_SLOW            NRF_DRV_SPI_FREQ_2M //( 3000000) // G : changing slow mode to 2MHz instead of 3MHz as the nrf doesn't support 3MHz SPI operation
#define SPI_SPEED_FAST            NRF_DRV_SPI_FREQ_8M //(10000000) // G : changing slow mode to 2MHz instead of 8MHz as the nrf doesn't support 10MHz SPI operation

#define RST NRF_GPIO_PIN_MAP(1,4)

// static uint32_t mode  = 0;  // not used
// static uint8_t bits   = 8;  // not used
//static uint32_t speed   = SPI_SPEED_SLOW; 
// static nrf_drv_spi_frequency_t speed = SPI_SPEED_SLOW; // not used
// static uint16_t delay   = 0; //
//static int fd;
static nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

static uint8_t tx_len, rx_len;
static uint8_t status;



#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

// static int RSTPin = 46; /* Reset GPIO pin - GPIO1_14 or pin 16 on the P8 header */   // G: Not required with nRF
// static int IRQPin = 47; /* Reset GPIO pin - GPIO1_15 or pin 15 on the P8 header */   // G: Not required with nRF
// static FILE *resetGPIO = NULL;                                                       // G: Not required with nRF
// static FILE *irqGPIO = NULL;                                                         // G: Not required with nRF



#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);

uint8_t tx_buf[128] = {0};
uint8_t rx_buf[128] = {0};
static volatile bool spi_xfer_done;

void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void *p_context)
{
  spi_xfer_done = true;
    // NRF_LOG_INFO("Transfer completed.\r\n");
    // if (m_rx_buf[0] != 0)
    // {
    //     NRF_LOG_INFO(" Received: \r\n");
    //     NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    // }
}

void gpio_init(void)
{
  nrf_gpio_cfg_output(RST);
}


/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_sleep(unsigned int time_ms)
{
  //sleep_ms(time_ms);
  nrf_delay_ms(time_ms);

}

void sleep_ms(unsigned int time_ms)
{
  //usleep(time_ms * 1000);
  nrf_delay_ms(time_ms * 1000);
  
}

int spi_set_rate_low (void)
{
  spi_config.frequency = SPI_SPEED_SLOW;
  return 0;

}

int spi_set_rate_high (void)
{
  spi_config.frequency = SPI_SPEED_FAST;
  return 0;
  
}

int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint32_t bodylength, const uint8_t *bodyBuffer)
{
  tx_len = headerLength+bodylength;
  rx_len = headerLength+bodylength;
  memset(tx_buf, 0, sizeof(tx_buf));
  memset(rx_buf, 0, sizeof(rx_buf));

  int j;
  for(j = 0; j < headerLength; j++)
  {
    tx_buf[j] = headerBuffer[j];
  }

  for(j = 0; j < bodylength; j++)
  {
    tx_buf[headerLength+j] = bodyBuffer[j];
  }
  spi_xfer_done = false;
  status = nrf_drv_spi_transfer(&spi, tx_buf, tx_len, rx_buf, rx_len); // SPI TRANSFER
  APP_ERROR_CHECK(status);
  while(!spi_xfer_done){} 
    NRF_LOG_HEXDUMP_INFO(headerBuffer, strlen((const char *)tx_buf));


  //tx_buf[tx_len++] = *headerBuffer;
  //rx_len = tx_len+1;

  //NRF_LOG_HEXDUMP_INFO(tx_buf, strlen((const char *)tx_buf));
  
  //spi_xfer_done = false; 
  //status = nrf_drv_spi_transfer(&spi, tx_buf, tx_len, rx_buf, rx_len); // SPI TRANSFER
  //APP_ERROR_CHECK(status);
  //while(!spi_xfer_done){} 
  //NRF_LOG_HEXDUMP_INFO(headerBuffer, tx_len);// wait
  
  //NRF_LOG_FLUSH();

  //uint8_t tx_len = 0;
  //uint8_t rx_len = 0;
  //memset(tx_buf, 0, sizeof(tx_buf));
  //memset(rx_buf, 0, sizeof(rx_buf));

  //tx_buf[tx_len++] = *bodyBuffer;
  //rx_len = tx_len+1;

  //NRF_LOG_HEXDUMP_INFO(tx_buf, strlen((const char *)tx_buf));
  
  //spi_xfer_done = false; 
  //status = nrf_drv_spi_transfer(&spi, tx_buf, tx_len, rx_buf, rx_len); // SPI TRANSFER
  //APP_ERROR_CHECK(status);
  //NRF_LOG_HEXDUMP_INFO(bodyBuffer, tx_len);
  //while(!spi_xfer_done){} // wait
  
  NRF_LOG_FLUSH();
  return DWT_SUCCESS;
  
  //return 0;

} // end writetospi()

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
  tx_len = headerLength+readlength;
  rx_len = headerLength+readlength;
  memset(tx_buf, 0, sizeof(tx_buf));
  memset(rx_buf, 0, sizeof(rx_buf));

  //rx_buf[tx_len++] = *headerBuffer;
  //tx_len = rx_len+1;

  //NRF_LOG_HEXDUMP_INFO(tx_buf, strlen((const char *)tx_buf));
  
  spi_xfer_done = false; 
  status = nrf_drv_spi_transfer(&spi, tx_buf, tx_len, rx_buf, rx_len); // SPI TRANSFER
  APP_ERROR_CHECK(status);
  while(!spi_xfer_done){} // wait

    int j;
  for(j = 0; j < readlength; j++)
  {
    readBuffer[j] = rx_buf[j+headerLength];
  }

  NRF_LOG_FLUSH();

  //tx_len = 0;
  //rx_len = 0;
  //memset(tx_buf, 0, sizeof(tx_buf));
  //memset(rx_buf, 0, sizeof(rx_buf));

  //rx_buf[tx_len++] = *readBuffer;
  //tx_len = rx_len+1;

  //NRF_LOG_HEXDUMP_INFO(tx_buf, strlen((const char *)tx_buf));
  
  //spi_xfer_done = false; 
  //status = nrf_drv_spi_transfer(&spi, tx_buf, tx_len, rx_buf, rx_len); // SPI TRANSFER
  //APP_ERROR_CHECK(status);
  //while(!spi_xfer_done){} // wait
  
  //NRF_LOG_FLUSH();
  //return 0;
  return DWT_SUCCESS;

} // end readfromspi()

void hardware_init ()
{
  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  spi_config.ss_pin   = SPI_SS_PIN;
  spi_config.miso_pin = SPI_MISO_PIN;
  spi_config.mosi_pin = SPI_MOSI_PIN;
  spi_config.sck_pin  = SPI_SCK_PIN;
  spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
  APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}

void hardware_close()
{

}

int reset_DW1000(void)
{
   // need to toggle the RSTn pin on the DW1000
  nrf_gpio_pin_clear(RST);
  nrf_delay_ms(10);
  nrf_gpio_pin_set(RST);
  nrf_delay_ms(10);
  return 0;
}

void spi_tr()                 // test function
{
 memset(m_rx_buf, 0, m_length);
 spi_xfer_done = false;

 APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));

 while (!spi_xfer_done)
 {
  __WFE();
}

NRF_LOG_FLUSH();

   //bsp_board_led_invert(BSP_BOARD_LED_0);
nrf_delay_ms(200);


}