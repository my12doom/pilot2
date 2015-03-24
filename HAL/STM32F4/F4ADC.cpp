#include "F4ADC.h"
#include <stdint.h>
#include "stm32f4xx_rcc.h"
namespace STM32F4
{
	F4ADC::F4ADC(ADC_TypeDef* ADCx,uint8_t ADC_Channel)
	{
		this->ADCx=ADCx;
		this->ADC_Channel=ADC_Channel;
		ADC_InitTypeDef ADC_InitStructure;
		ADC_CommonInitTypeDef ADC_CommonInitStructure;
		//open all ADC Clock And GPIO clock 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC|RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_ADC3, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);
		//Init ADC config:
		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
		ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
		ADC_CommonInit(&ADC_CommonInitStructure);
		
		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfConversion = 1;
		
		ADC_Init(ADCx, &ADC_InitStructure);
		ADC_RegularChannelConfig(ADCx, ADC_Channel, 1, ADC_SampleTime_480Cycles);
		ADC_Cmd(ADCx, ENABLE);
	}
	int	F4ADC::read()
	{
		ADC_ClearFlag(ADCx, ADC_FLAG_EOC);
		ADC_SoftwareStartConv(ADCx);
		while (ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) == RESET);
		return ADC_GetConversionValue(ADCx);
	}
}
