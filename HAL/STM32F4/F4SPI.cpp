#include "F4SPI.h"
#include <stdint.h>
#include <string.h>
#include <stm32f4xx_rcc.h>

#include "stm32F4xx_gpio.h"
namespace STM32F4
{
	F4SPI::F4SPI()
	{
	}
	
	F4SPI::F4SPI(SPI_TypeDef* SPIx)
	{
		init(SPIx);
	}
	
	int F4SPI::init(SPI_TypeDef* SPIx)
	{
		this->SPIx = SPIx;
		memset(&SPI_InitStructure, 0, sizeof(SPI_InitStructure));

		GPIO_InitTypeDef GPIO_InitStructure;
		
		if (SPIx == SPI1)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
			GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
			GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
			
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
		
		else if (SPIx == SPI2)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
			
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);
			GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
			GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);
			
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
		
		return 0;
	}

	int F4SPI::init()
	{
		return 0;
	}
	
	int F4SPI::set_speed(int speed)	// speed in hz
	{
		// TODO
		SPI_Cmd(SPIx, DISABLE);
		SPI_Init(SPIx, &SPI_InitStructure);
		SPI_Cmd(SPIx, ENABLE);
		
		return 0;
	}
	
	int F4SPI::set_mode(int mode)		// see http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus for mode definition
	{
		// TODO
		switch(mode)
		{
			case 0:
				break;
			case 1:
				break;
			case 2:
				break;
			case 3:
				break;
			default:
				// reset mcu?
				break;
		}
		SPI_Cmd(SPIx, DISABLE);
		SPI_Init(SPIx, &SPI_InitStructure);
		SPI_Cmd(SPIx, ENABLE);
		
		return 0;
	}
	
	uint8_t F4SPI::txrx(uint8_t data)
	{
		while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPIx, data);
		while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);

		return SPI_I2S_ReceiveData(SPIx);
	}
}
