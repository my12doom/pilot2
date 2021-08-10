#include "adc.h"
#include <stdlib.h>

#include "stm32f4xx.h"
#include <stm32f4xx_adc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_dma.h>
#include <stm32F4xx_tim.h>

#include <HAL/Interface/ISysTimer.h>


dma_adc *me = NULL;
extern "C" void DMA2_Stream0_IRQHandler(void)
{
    if (me)
        me->irq();
}

dma_adc::dma_adc()
{
    cb = NULL;
    me = this;
 }

dma_adc::~dma_adc()
{

}

int dma_adc::init(GPIO_TypeDef *GPIOx, int channel)
{
	stop();
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

    // TIM2 config
	// sample rate = 84Mhz / (Prescaler+1) / (Period+1)
	//             = 84Mhz / 4 / 21
	//             = 1Mhz
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_DeInit(TIM2);
	TIM_InternalClockConfig(TIM2);
	TIM_TimeBaseStructure.TIM_Prescaler = 3;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=20;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM2,DISABLE);
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

    // DMA2 channel1 configuration 
	// Enable the DMA2_Stream0 global Interrupt
	NVIC_InitTypeDef   NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x06;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// ADC configuration
    ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	//
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_3Cycles);
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);	
	ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    // Configure analog input pin
    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = (1 << (channel%8));
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(channel>=8?GPIOB:GPIOA, &GPIO_InitStructure);

    return 0;
}

int dma_adc::begin()
{
    // DMA2 Stream0 channel0 configuration
    DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)0x40012308;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adc_data[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = max_adc_data_count;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_DeInit(DMA2_Stream0);
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);
	DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_HTIF0);
	DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0);
	//DMA_ITConfig(DMA2_Stream0, DMA_IT_HT, ENABLE);
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);

    _full = false;

    TIM_Cmd(TIM2,ENABLE);
	
	return 0;
}
int dma_adc::stop()
{
    TIM_Cmd(TIM2, DISABLE);
	systimer->delayus(10);
    DMA_Cmd(DMA2_Stream0, DISABLE);
	
	return 0;
}

bool dma_adc::full()
{
    return _full;
}

void dma_adc::irq()
{
    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0) && DMA_GetCmdStatus(DMA2_Stream0))
	{
		if (!_full && cb)
			cb(user);
        _full = true;
	}
    
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_FLAG_HTIF0 | DMA_FLAG_TCIF0);
}

int dma_adc::pos()
{
    int pos = max_adc_data_count - DMA2_Stream0->NDTR;

    return pos;
}

int dma_adc::set_cb(dma_full_cb cb, void *user)
{
    this->cb = cb;
    this->user = user;
    return 0;
}
