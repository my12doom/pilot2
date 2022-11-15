#include "RCOUT.h"
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>
#include <string.h>

#define OC 12

namespace dev_v2
{
	
// PE14, PE13, PE11, PE9
// TIM1_CH4, CH3, CH2, CH1
RCOUT::RCOUT()
{
	memset(channel_datas, 0, sizeof(channel_datas));
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	// open B0,1,4,5 as output (TIM1 channel 1~4)
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);		// TIM1_CH4
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);	// TIM1_CH3
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1); 	// TIM1_CH2
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);	// TIM1_CH1
	
	// Time base configuration
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = 2000*OC-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 168/OC-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
		
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM1->BDTR |= 1<<15 | 75;
}

// total channel count
int RCOUT::get_channel_count()
{
	return MAX_CHANNEL;
}

// return num channel written
// generate an error if index overrun/underrun, and won't update any channel
// return negative value to indicate an error
int RCOUT::write(const int16_t *data, int start_channel, int count)
{
	//for(int i=0; i<3; i++)
	//	((int16_t *)data)[i] = 1000;
	volatile uint32_t *registers[MAX_CHANNEL] =
	{
		&TIM1->CCR1,
		&TIM1->CCR4,
		&TIM1->CCR2,
		&TIM1->CCR3,
	};
	
	((int16_t *)data)[0] = 1000;
	((int16_t *)data)[1] = 1000;
	((int16_t *)data)[2] = 1000;
	//((int16_t *)data)[3] = 1000;
	
	if (start_channel < 0 || start_channel + count > MAX_CHANNEL)
		return -1;
	
	for(int i=0; i<count; i++)
	{
		channel_datas[i+start_channel] = data[i];
		registers[i+start_channel][0] = data[i] * OC;
	}
	
	TIM_Cmd(TIM1, ENABLE);
	
	return 0;
}

// return num channel read
// return any possible read if index overrun
// return negative value to indicate an error
int RCOUT::read(int16_t *out, int start_channel, int max_count)
{
	if (start_channel<0 || start_channel >= MAX_CHANNEL)
		return 0;
	
	int count = MAX_CHANNEL - start_channel;
	if (count > max_count)
		count = max_count;
	
	for(int i=0; i<count; i++)
		out[i] = channel_datas[i+start_channel];
	
	return count;	
}

}