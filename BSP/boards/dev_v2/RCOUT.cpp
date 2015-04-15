#include "RCOUT.h"
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>
#include <string.h>

#define OC 12

namespace dev_v2
{
	
RCOUT::RCOUT()
{
	memset(channel_datas, 0, sizeof(channel_datas));
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
	
	// open A0 A1 A2 A3 A6 A7 as output
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	

	TIM_OCInitTypeDef  TIM_OCInitStructure;

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 3000*OC-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 84/OC-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
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
	volatile uint32_t *registers[MAX_CHANNEL] =
	{
		&TIM2->CCR1,
		&TIM2->CCR2,
		&TIM2->CCR3,
		&TIM2->CCR4,
		&TIM3->CCR1,
		&TIM3->CCR2,
	};
	
	if (start_channel < 0 || start_channel + count > MAX_CHANNEL)
		return -1;
	
	for(int i=0; i<count; i++)
		registers[i+start_channel][0] = data[i] * OC;
	
	/*
	if (channel_to_update & PPM_OUTPUT_CHANNEL0)
		TIM_SetCompare1(TIM2, g_ppm_output[0]*OC);		// PA0

	if (channel_to_update & PPM_OUTPUT_CHANNEL1)
		TIM_SetCompare2(TIM2, g_ppm_output[1]*OC);		// PA1

	if (channel_to_update & PPM_OUTPUT_CHANNEL2)
		TIM_SetCompare3(TIM2, g_ppm_output[2]*OC);		// PA2

	if (channel_to_update & PPM_OUTPUT_CHANNEL3)
		TIM_SetCompare4(TIM2, g_ppm_output[3]*OC);		// PA3

	if (channel_to_update & PPM_OUTPUT_CHANNEL4)
		TIM_SetCompare1(TIM3, g_ppm_output[4]*OC);		// PA6

	if (channel_to_update & PPM_OUTPUT_CHANNEL5)
		TIM_SetCompare2(TIM3, g_ppm_output[5]*OC);		// PA7
	*/
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