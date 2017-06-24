#include <stdint.h>
#include <stdlib.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_dac.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <HAL/Interface/ISysTimer.h>
#include "DAC_F0.h"

using namespace STM32F0;
using namespace HAL;

static F0GPIO *beep_en = NULL;
bool reverse_polary = false;
static uint32_t repeat = 1;

int dac_config(STM32F0::F0GPIO *en, bool _reverse_polary)
{
	beep_en = en;
	reverse_polary = _reverse_polary;
	beep_en->write(false ^ reverse_polary);
	beep_en->set_mode(MODE_OUT_PushPull);

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
	DMA_InitStructure.DMA_MemoryBaseAddr = 0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

	DAC_InitTypeDef DAC_InitStruct;
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_T15_TRGO;
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStruct);
	
	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_DMACmd(DAC_Channel_1, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
	
	
	return 0;
}

extern "C" void DMA1_Channel2_3_IRQHandler()
{
	DMA_ClearFlag(DMA1_FLAG_TC3);
	if (repeat == 0)
	{
		beep_en->write(true ^ reverse_polary);
		DMA_Cmd(DMA1_Channel3, DISABLE);
	}
	repeat --;
}

int dac_run(const void *data, int data_point_count, int data_type, int repeat/* = 1*/)
{	
	beep_en->write(false ^ reverse_polary);
	DMA_Cmd(DMA1_Channel3, DISABLE);
	DMA1_Channel3->CNDTR = data_point_count;
	DMA1_Channel3->CMAR = (uint32_t)data;
	::repeat = repeat;
	beep_en->write(false ^ reverse_polary);
	DMA_Cmd(DMA1_Channel3, ENABLE);
	
	return 0;
}
