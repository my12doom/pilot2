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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
	
	// open B1(TIM3_CH4) B3(TIM2_CH3) B11(TIM2_CH4) as output
	GPIO_SetBits(GPIOB, GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_11);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
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

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM3, ENABLE);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	
	TIM_SetCompare2(TIM2, period-1);
	TIM_SetCompare4(TIM3, period-1);
	TIM_SetCompare4(TIM2, period-1);
}


int RGBLED::write(float B, float G, float R)
{
	R = limit(R, 0, 1);		// PB11, TIM2_CH4
	G = limit(G, 0, 1);		// PB1,  TIM3_CH4
	B = limit(B, 0, 1);		// PB3,  TIM2_CH2
		
	R=1-R*R;
	G=1-G*G;
	B=1-B*B;
	
	TIM_SetCompare2(TIM2, B*period);
	TIM_SetCompare4(TIM3, G*period);
	TIM_SetCompare4(TIM2, R*period);

	return 0;
}
}
