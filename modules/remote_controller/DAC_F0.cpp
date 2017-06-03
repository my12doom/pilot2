#include <stdint.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_dac.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <HAL/Interface/ISysTimer.h>

static uint8_t data[2] = {0xff};
	
int dac_run(const void *data, int data_point_count, int data_type);
int dac_config();
	
int dac_config()
{	
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_DeInit(TIM15);
	TIM_InternalClockConfig(TIM15);
	TIM_TimeBaseStructure.TIM_Prescaler= 48-1;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=22;						// 48Mhz/48/23 = 43.48KHZ ~= 44Khz
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM15,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM15,TIM_FLAG_Update);
	TIM_SelectOutputTrigger(TIM15, TIM_TRGOSource_Update);

	TIM_Cmd(TIM15,ENABLE);
	
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&DAC->DHR8R1;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&data;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = sizeof(data);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel3, ENABLE);

	DAC_InitTypeDef DAC_InitStruct;
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_T15_TRGO;
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStruct);
	
	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_DMACmd(DAC_Channel_1, ENABLE);
		
	return 0;
}

int dac_run(const void *data, int data_point_count, int data_type)
{	
	int i=0;
	uint8_t *data8 = (uint8_t*)data;
	while(0)
	{
	}
	
	return 0;
}
