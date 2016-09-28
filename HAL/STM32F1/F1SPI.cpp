#include "F1SPI.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <stm32F10x_gpio.h>
#include <stm32F10x_rcc.h>
namespace STM32F1
{
	F1SPI::F1SPI()
	{
	}
	
	F1SPI::F1SPI(SPI_TypeDef* SPIx)
	{
		init(SPIx);
	}
	
	int F1SPI::init()
	{
		return -1;
	}
	int F1SPI::init(SPI_TypeDef* SPIx)
	{
		this->SPIx = SPIx;
		memset(&SPI_InitStructure, 0, sizeof(SPI_InitStructure));

		GPIO_InitTypeDef GPIO_InitStructure;
		
		if (SPIx == SPI1)
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
			
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			
			SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
			SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
			SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
			SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
			SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
			SPI_InitStructure.SPI_CRCPolynomial = 7;
			SPI_Init(SPIx, &SPI_InitStructure);
			SPI_Cmd(SPIx, ENABLE);
		}
		else
		{
			return -1;
		}
		return 0;
	}
	
	int F1SPI::set_speed(int speed)	// speed in hz
	{
		// SPI1 on APB2, 42mhz
		// SPI2 on APB1, 84mhz
		int APB_frequency = SPIx == SPI1 ? 72000000 : 36000000;
		if (speed > APB_frequency / 2)
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
		else if (speed > APB_frequency / 4)
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
		else if (speed > APB_frequency / 8)
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
		else if (speed > APB_frequency / 16)
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
		else if (speed > APB_frequency / 32)
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
		else if (speed > APB_frequency / 64)
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
		else if (speed > APB_frequency / 128)
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
		else if (speed > APB_frequency / 256)
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
		else
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;

		SPI_Cmd(SPIx, DISABLE);
		SPI_Init(SPIx, &SPI_InitStructure);
		SPI_Cmd(SPIx, ENABLE);
		
		return 0;
	}
	
	int F1SPI::set_mode(int CPOL, int CPHA)// CPOL: 0 = Idle Low, 1 = Idle High; CPHA: 0 = capture at first edge, 1 = capture at second edge
	{
		SPI_InitStructure.SPI_CPOL = CPOL ? SPI_CPOL_High : SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = CPHA ? SPI_CPHA_2Edge : SPI_CPHA_1Edge;
		SPI_Cmd(SPIx, DISABLE);
		SPI_Init(SPIx, &SPI_InitStructure);
		SPI_Cmd(SPIx, ENABLE);
		
		return 0;
	}
	
	uint8_t F1SPI::txrx(uint8_t data)
	{
		while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPIx, data);
		while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);

		uint8_t o = SPI_I2S_ReceiveData(SPIx);
		//printf("spi:%02x - %02x\n", data, o);
		return o;
	}
}
