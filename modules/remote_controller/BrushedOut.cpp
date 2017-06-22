#include "BrushedOut.h"
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_rcc.h>
#include <misc.h>
#include <string.h>
#include <stdio.h>
#include <HAL/Interface/ISysTimer.h>

// PB0: M2PWM	(TIM3_CH3)
// PB1: M1PWM	(TIM3_CH4)
// PB4: M3PWM1	(TIM3_CH1)
// PB5: M3PWM2	(TIM3_CH2)
// PB8: M3P1
// PB9: M3P2
// 

int channel_count = 4;

BrushedOut::BrushedOut()
{
	memset(channel_data, 0, sizeof(channel_data));
	armed = false;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5);
	GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// Time base configuration
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = 1000-1;		// 24Mhz/1000 = 24Khz
	TIM_TimeBaseStructure.TIM_Prescaler = 3-1;		// 72Mhz/3 = 24Mhz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM3, ENABLE);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;
	
	// remap TIM3
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

	// disable JTAG, it conflicts with TIM3, leave SW-DP enabled, 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

}

int16_t trim[4] =
{
	0,
	0,
	-100,
	0,
};

float yaw_ratio = 0.4f;
float pitch_ratio = 1.0f;

int BrushedOut::write(const int16_t *data, int count, int start_channel)
{
	if (count > channel_count)
		count = channel_count;
	
	if (data[2] < 1000)
	{
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
		TIM3->CCR3 = 0;
		TIM3->CCR4 = 0;
		return -1;
	}
	
	int16_t after_trim[4];
	for(int i=0; i<count; i++)
	{
		channel_data[i+start_channel] = data[i];
		after_trim[i] = data[i]-1000 + trim[i];
		
		if (after_trim[i]<0)
			after_trim[i] = 0;
		if (after_trim[i]>1000)
			after_trim[i] = 1000;
	}
	
	int16_t mix[4];
	mix[0] = after_trim[2] - (after_trim[3]-500)*yaw_ratio;
	mix[1] = after_trim[2] + (after_trim[3]-500)*yaw_ratio;
	mix[2] = after_trim[1];
	if (after_trim[2] <= 0)
		armed = true;
	
	if (!armed )
	{
		mix[0] = 0;
		mix[1] = 0;
		mix[2] = 500;
	}
	
	if (after_trim[2] <= 0)
	{
		mix[0] = 0;
		mix[1] = 0;
	}
	
	for(int i=0; i<count; i++)
	{
		if (mix[i]<0)
			mix[i] = 0;
		if (mix[i]>1000)
			mix[i] = 1000;
	}

	TIM3->CCR3 = mix[0];
	TIM3->CCR4 = mix[1];
	
	// m3, "3D"
	mix[2] -= 500;
	bool new_m3positive = mix[2] > 0;
	if (m3positive != new_m3positive)
	{
		// close mosfets and wait a few milli-seconds to let H-bridge settle down
		GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
		systimer->delayms(1);
		m3positive = new_m3positive;
	}
	
	if (mix[2] < 70 && mix[2] > -70)
	{
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
		GPIO_SetBits(GPIOB, GPIO_Pin_8);
		GPIO_SetBits(GPIOB, GPIO_Pin_9);
	}
	else if (mix[2] > 0)
	{
		// use M3P2+M3PWM1, PB9 + PB4(TIM3_CH1)
		GPIO_SetBits(GPIOB, GPIO_Pin_8);
		GPIO_ResetBits(GPIOB, GPIO_Pin_9);
		TIM3->CCR1 = mix[2] * 2 * pitch_ratio;
		TIM3->CCR2 = 0;
	}
	else if (mix[2] < 0)
	{
		// use M3P1+M3PWM2, PB8 + PB5(TIM3_CH2)
		GPIO_ResetBits(GPIOB, GPIO_Pin_8);
		GPIO_SetBits(GPIOB, GPIO_Pin_9);
		TIM3->CCR1 = 0;
		TIM3->CCR2 = -mix[2] * 2 * pitch_ratio;
	}
	
	return 0;
}

int BrushedOut::get_channel_count()
{
	return channel_count;
}

int BrushedOut::read(int16_t *out, int start_channel, int max_count)
{
	if (start_channel<0 || start_channel >= channel_count)
		return 0;
	
	int count = channel_count - start_channel;
	if (count > max_count)
		count = max_count;
	
	for(int i=0; i<count; i++)
		out[i] = channel_data[i+start_channel];
	
	return count;	
}
