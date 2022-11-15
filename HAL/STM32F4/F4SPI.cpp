#include "F4SPI.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stm32f4xx_rcc.h>
#include "stm32F4xx_gpio.h"

namespace STM32F4
{
	F4SPI::F4SPI()
	{
	}
	
	F4SPI::F4SPI(SPI_TypeDef* SPIx, bool use_dma)
	{
		init(SPIx, use_dma);
	}
	
	int F4SPI::init(SPI_TypeDef* SPIx, bool use_dma)
	{
		this->SPIx = SPIx;
		this->use_dma = use_dma;

		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
		
		memset(&SPI_InitStructure, 0, sizeof(SPI_InitStructure));
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CRCPolynomial = 7;
		SPI_Init(SPIx, &SPI_InitStructure);
		SPI_Cmd(SPIx, ENABLE);
		
		if (SPIx == SPI1)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
			
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
			GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
			GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
			
			if (use_dma)
			{
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
				tx_dma_channel = DMA_Channel_3;
				rx_dma_channel = DMA_Channel_3;
				tx_dma = DMA2_Stream5;
				rx_dma = DMA2_Stream2;
				tx_clear_reg = &DMA2->HIFCR;
				rx_clear_reg = &DMA2->LIFCR;
				tx_clear_flag = DMA_FLAG_TCIF5;
				rx_clear_flag = DMA_FLAG_TCIF2;
			}
		}
		
		else if (SPIx == SPI2)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
			
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);
			GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
			GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);
			
			if (use_dma)
			{
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
				tx_dma_channel = DMA_Channel_0;
				rx_dma_channel = DMA_Channel_0;
				tx_dma = DMA1_Stream4;
				rx_dma = DMA1_Stream3;
				tx_clear_reg = &DMA1->HIFCR;
				rx_clear_reg = &DMA1->LIFCR;
				tx_clear_flag = DMA_FLAG_TCIF4;
				rx_clear_flag = DMA_FLAG_TCIF3;
			}
		}

		else if (SPIx == SPI3)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
			
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
			GPIO_Init(GPIOC, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SPI3);
			GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SPI3);
			GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3);
			
			if (use_dma)
			{
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
				tx_dma_channel = DMA_Channel_0;
				rx_dma_channel = DMA_Channel_0;
				tx_dma = DMA1_Stream5;
				rx_dma = DMA1_Stream2;
				tx_clear_reg = &DMA1->HIFCR;
				rx_clear_reg = &DMA1->LIFCR;
				tx_clear_flag = DMA_FLAG_TCIF5;
				rx_clear_flag = DMA_FLAG_TCIF2;
			}
		}
		
		if (use_dma)
		{
			SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Tx, ENABLE);
			SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Rx, ENABLE);

			DMA_InitTypeDef DMA_InitStructure = {0};
			
			// TX
			DMA_DeInit(tx_dma);  
			DMA_InitStructure.DMA_Channel = tx_dma_channel; 
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPIx->DR));
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)0;
			DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
			DMA_InitStructure.DMA_BufferSize = 4;
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
			DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
			DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
			DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
			DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
			DMA_Init(tx_dma, &DMA_InitStructure);
			
			// RX
			DMA_DeInit(rx_dma);
			DMA_InitStructure.DMA_Channel = rx_dma_channel; 
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)0;
			DMA_Init(rx_dma, &DMA_InitStructure);
		}
		else
		{
			SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Tx, DISABLE);
			SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Rx, DISABLE);
		}
		
		return 0;
	}

	int F4SPI::init()
	{
		return 0;
	}
	
	int F4SPI::set_speed(int speed)	// speed in hz
	{
		// SPI1 on APB2, 84mhz
		// SPI2 on APB1, 42mhz
		int APB_frequency = SPIx == SPI1 ? 84000000 : 42000000;
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
	
	int F4SPI::set_mode(int CPOL, int CPHA)// CPOL: 0 = Idle Low, 1 = Idle High; CPHA: 0 = capture at first edge, 1 = capture at second edge
	{
		SPI_InitStructure.SPI_CPOL = CPOL ? SPI_CPOL_High : SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = CPHA ? SPI_CPHA_2Edge : SPI_CPHA_1Edge;
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

		uint8_t o = SPI_I2S_ReceiveData(SPIx);
		return o;
	}

	uint8_t F4SPI::txrx2(const uint8_t *tx, uint8_t *rx, int len)
	{
		if (use_dma)
		{
			*rx_clear_reg = rx_clear_flag;
			*tx_clear_reg = tx_clear_flag;
			
			rx_dma->M0AR = (uint32_t)rx;
			rx_dma->NDTR = len;

			tx_dma->M0AR = (uint32_t)tx;
			tx_dma->NDTR = len;
			
			// start rx first
			DMA_Cmd(rx_dma, ENABLE);
			DMA_Cmd(tx_dma, ENABLE);
			
			// wait for both done
			while((tx_dma->CR & DMA_SxCR_EN) || (rx_dma->CR & DMA_SxCR_EN))
				;
		}
		else
		{
			
			for(int i=0; i<len; i++)
			{
				while (!(SPIx->SR & SPI_I2S_FLAG_TXE))
					;
				SPIx->DR = tx[i];
				while (!(SPIx->SR & SPI_I2S_FLAG_RXNE))
					;
				if (rx) rx[i] = SPIx->DR;
			}
		}
		return len;
	}
}
