/***************************************************************************//**
 *   @file   Platform.c
 *   @brief  Implementation of Platform Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <HAL/STM32F4/F4SysTimer.h>
#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4GPIO.h>
using namespace STM32F4;
using namespace HAL;

extern "C" 
{
#include "platform.h"
}


static F4SPI spi3(SPI3);
static F4GPIO cs(GPIOD, GPIO_Pin_2);
uint8_t dummy[10];
uint8_t dummy0[10] = {0};

/***************************************************************************//**
 * @brief spi_init
*******************************************************************************/
extern "C" int32_t spi_init(uint32_t device_id,
				 uint8_t  clk_pha,
				 uint8_t  clk_pol)
{
	spi3.set_mode(0,1);
	cs.set_mode(MODE_OUT_PushPull);
	cs.write(true);
	
	spi3.txrx2(dummy, dummy0, 2);
	spi3.set_speed(1000000);
	
	return 0;
}

/***************************************************************************//**
 * @brief spi_read
*******************************************************************************/
extern "C" int32_t spi_read(uint8_t *data,
				 uint8_t bytes_number)
{
	// not called
	return 0;
}

/***************************************************************************//**
 * @brief spi_write_then_read
*******************************************************************************/

extern "C" int spi_write_then_read(struct spi_device *spi,
		const unsigned char *txbuf, unsigned n_tx,
		unsigned char *rxbuf, unsigned n_rx)
{
	cs.write(false);
	spi3.txrx2(txbuf, dummy, n_tx);
	spi3.txrx2(dummy0, rxbuf, n_rx);	
	cs.write(true);
	
	return 0;
}

/***************************************************************************//**
 * @brief gpio_init
*******************************************************************************/
extern "C" void gpio_init(uint32_t device_id)
{

}

/***************************************************************************//**
 * @brief gpio_direction
*******************************************************************************/
extern "C" void gpio_direction(uint8_t pin, uint8_t direction)
{

}

/***************************************************************************//**
 * @brief gpio_is_valid
*******************************************************************************/
extern "C" bool gpio_is_valid(int number)
{
	return 0;
}

/***************************************************************************//**
 * @brief gpio_data
*******************************************************************************/
extern "C" void gpio_data(uint8_t pin, uint8_t data)
{

}

/***************************************************************************//**
 * @brief gpio_set_value
*******************************************************************************/
extern "C" void gpio_set_value(unsigned gpio, int value)
{

}

/***************************************************************************//**
 * @brief udelay
*******************************************************************************/
extern "C" void udelay(unsigned long usecs)
{
	systimer->delayus(usecs);
}


/***************************************************************************//**
 * @brief usleep
*******************************************************************************/
extern "C" void usleep(unsigned long usleep)
{
	systimer->delayus(usleep);
}


/***************************************************************************//**
 * @brief mdelay
*******************************************************************************/
extern "C" void mdelay(unsigned long msecs)
{
	systimer->delayms(msecs);
}

/***************************************************************************//**
 * @brief msleep_interruptible
*******************************************************************************/
unsigned long msleep_interruptible(unsigned int msecs)
{
	systimer->delayms(msecs);
	return 0;
}
