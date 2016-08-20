#include "ads1256.h"
#include <stdio.h>
#include <string.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_spi.h>
#include <HAL/Interface/ISysTimer.h>

void ads1256_begin()
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_9);
}
void ads1256_end()
{
	GPIO_SetBits(GPIOA, GPIO_Pin_9);
}

uint8_t ads1256_tx_rx(uint8_t Data)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, Data);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	Data = SPI_I2S_ReceiveData(SPI1);

	return Data;
} 


uint8_t ads1256_read_registers(uint8_t start, uint8_t n, void *out)
{
	int i;
	uint8_t *p = (uint8_t*)out;
	ads1256_begin();
	ads1256_tx_rx((start&0xf) | 0x10);
	ads1256_tx_rx(n-1);
	systimer->delayms(2);
	for(i=0; i<n; i++)
		p[i]=ads1256_tx_rx(0xff);

	ads1256_end();

	return n;
}

uint8_t ads1256_write_registers(uint8_t start, uint8_t n, void *data)
{
	int i;
	uint8_t *p = (uint8_t*)data;
	ads1256_begin();
	ads1256_tx_rx((start&0xf) | 0x50);
	ads1256_tx_rx(n-1);
	systimer->delayms(2);
	for(i=0; i<n; i++)
		ads1256_tx_rx(p[i]);

	ads1256_end();

	return n;
}

uint8_t ads1256_read_register(uint8_t reg)
{
	uint8_t reg1, reg2;
	for(int i=0; i<10; i++)
	{
		reg1 = 0;
		reg2 = 3;		
		ads1256_read_registers(reg, 1, &reg1);
		ads1256_read_registers(reg, 1, &reg2);
		
		if (reg1 == reg2)
			return reg1;
	}
	return 0xff;
}
void ads1256_write_register(uint8_t reg, uint8_t data)
{
	for(int i=0; i<10; i++)
	{
		uint8_t data_read = data+1;
		ads1256_write_registers(reg, 1, &data);
		ads1256_read_registers(reg, 1, &data_read);
		
		if (data_read == data)
			return;
	}
}



int ads1256_init(void)
{
	systimer->delayms(50);
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8_t data = 55;
	int i = 0;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	// 配置 SCK,MISO,MOSI 引脚，GPIOA^5,GPIOA^6,GPIOA^7
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);

	//配置 CS 引脚 PA9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	systimer->delayms(50);
	ads1256_end();

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //双线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//主模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//数据大小 8 位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							//时钟极性，空闲时为低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						//第 1 个边沿有效，上升沿为采样时刻
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//NSS 信号由软件产生
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;	//8 分频，9MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//高位在前
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	// Enable SPI1
	SPI_Cmd(SPI1, ENABLE);

	ads1256_begin();
	ads1256_tx_rx(0xff);
	ads1256_end();

	for(i=0; i<10; i++)
	{
		ads1256_read_registers(i, 1, &data);

		printf("reg(%d)=0x%02x\n", i, data);
	}

	return 0;
}

int ads1256_startconvert(void)
{
	
	return 0;
}

void ads1256_go()
{
	SPI_InitTypeDef  SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //双线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//主模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//数据大小 8 位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							//时钟极性，空闲时为低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						//第 1 个边沿有效，上升沿为采样时刻
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//NSS 信号由软件产生
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;	//8 分频，9MHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//高位在前
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
}

int ads1256_getresult(short *result)		// return -1 if still converting, 0 if conversion completed, further calls return the last conversion result.
{
	
	return 0;
}

short ads1256_convert(void)				// a simplfied version which start a new conversion ,wait it to complete and returns the result directly
{
	short v;
	ads1256_startconvert();
	while (ads1256_getresult(&v)!=0)
		;
	return v;
}
