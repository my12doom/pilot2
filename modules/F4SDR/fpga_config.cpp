#include "fpga_config.h"
#include <stm32f4xx_rcc.h>
#include <stm32F4xx_gpio.h>
#include <stm32F4xx_spi.h>
#include <stm32F4xx_dma.h>

#include <stdlib.h>
#include <string.h>
#include <Protocol/crc32.h>
#include <HAL/STM32F4/F4SysTimer.h>

extern "C" 
{
#include <utils/minilzo.h>
}

void SPI1_DMA_TX(const uint8_t *p, int count)
{
	while(DMA2_Stream3->CR & DMA_SxCR_EN)
		;
	DMA_Cmd(DMA2_Stream3, DISABLE);
	DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
	
	DMA2_Stream3->M0AR = (uint32_t)p;
	DMA2_Stream3->NDTR = count;		
	
	DMA_Cmd(DMA2_Stream3, ENABLE);
}

uint8_t blk_buffer[4096];
int dec = 0;
uint32_t dec_crc = 0;
int blk_cb(lzo_callback_p hcb, lzo_voidp ptr, lzo_uint out_size, lzo_uint read_size)
{
	if (out_size > 0)
	{
		// wait for DMA finish, to avoid corrupting
		while(DMA2_Stream3->CR & DMA_SxCR_EN)
			;
		memcpy(blk_buffer, ptr, out_size);
		dec += out_size;
		SPI1_DMA_TX(blk_buffer, out_size);
		dec_crc = crc32(dec_crc, blk_buffer, out_size);
	}
	
	return 0;
}

int fpga_config(HAL::IGPIO* init_b /* = NULL */)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	DMA_InitTypeDef DMA_InitStructure = {0};
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1);
	//GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
	
	if (init_b)
	{
		init_b->write(1);
		init_b->set_mode(HAL::MODE_IN);
		while(!init_b->read())
			;
	}
	else
	{
		systimer->delayms(10);
	}

	DMA_DeInit(DMA2_Stream3);  
	DMA_InitStructure.DMA_Channel = DMA_Channel_3; 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI1->DR));
	DMA_InitStructure.DMA_Memory0BaseAddr = 0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = 4;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
	DMA_Init(DMA2_Stream3, &DMA_InitStructure);
	
	
	

	int stream_size = (*(const uint32_t*)0x08020000) & 0x00ffffff;
	bool lzo = (*(const uint32_t*)0x08020000) & 0x80000000;
	uint32_t crc = *(const uint32_t*)0x08020004;
	const uint8_t *bitstream = (const uint8_t*)0x08020008;
	if (stream_size > 700*1024 || crc32(0, bitstream, stream_size) != crc)
		return 0;

	if (lzo)
	{
		lzo_init();
		dec = 0;
		dec_crc = 0;
		lzo_uint decompress_len = MAX_BS_LEN*2+4096;
		static uint8_t buffer[MAX_BS_LEN*2+4096];
		int r = lzo1x_decompress_safe(bitstream, stream_size, buffer, &decompress_len, blk_cb, 4096);
		SPI1_DMA_TX(buffer, decompress_len);
		dec += decompress_len;
		dec_crc = crc32(dec_crc, buffer, decompress_len);
		while(DMA2_Stream3->CR & DMA_SxCR_EN)
			;
	}
	else
	{	
		for(int i=0; i<stream_size; i+= 32768)
			SPI1_DMA_TX(bitstream+i, (stream_size-i) > 32768 ? 32768 : (stream_size-i));
	}

	// wait for all data
	while(DMA2_Stream3->CR & DMA_SxCR_EN)
		;
	
	return (int)bitstream;
}

