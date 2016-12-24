#include "RGBLED.h"
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>
#include <string.h>

#include <Protocol/common.h>

#define period 36000

namespace dev_v2
{
	
RGBLED::RGBLED()
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM2, ENABLE);
	
	// open B9(TIM4_CH4) B10(TIM2_CH3) B11(TIM2_CH4) as output
	GPIO_ResetBits(GPIOB, GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);
	
	// Time base configuration
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = period-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 7-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ARRPreloadConfig(TIM2, ENABLE);

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM4, ENABLE);
	TIM_ARRPreloadConfig(TIM4, ENABLE);

	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	
	TIM_SetCompare4(TIM2, 0*period);
	TIM_SetCompare3(TIM2, 0*period);
	TIM_SetCompare4(TIM4, 0*period);
}


int RGBLED::write(float R, float G, float B)
{	
	R = limit(R, 0, 1);
	G = limit(G, 0, 1);
	B = limit(B, 0, 1);
		
	R=R*R;
	G=G*G;
	B=B*B;
	
	TIM_SetCompare4(TIM2, B*period);
	TIM_SetCompare3(TIM2, G*period);
	TIM_SetCompare4(TIM4, R*period);

	return 0;
}
}
