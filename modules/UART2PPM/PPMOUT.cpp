#include "PPMOUT.h"
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>
#include <string.h>
#include <stdio.h>
#include <HAL/Interface/ISysTimer.h>

PPMOUT * _this;

extern "C" void TIM3_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);
	_this->cb();
	
	
}

PPMOUT::PPMOUT()
{
	_this = this;
	sending = -1;
	memset(sending_data, 0, sizeof(sending_data));
	memset(sending_data, 0, sizeof(sending_data));

	GPIO_InitTypeDef GPIO_InitStructure = {0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	
	// Time base configuration
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = 20000-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM3, DISABLE);
	//TIM_Cmd(TIM3, ENABLE);

	TIM_ARRPreloadConfig(TIM3, DISABLE);
	TIM_SelectOnePulseMode(TIM3, TIM_OPMode_Single);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);

	TIM3->CCR3 = 20;
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	NVIC_Init(&NVIC_InitStructure);
	

}

int PPMOUT::send()
{
	int c = 350;
	if (sending > 0)
	{
		c = sending_data[sending-1];
		GPIO_ToggleBits(GPIOB, GPIO_Pin_1);
	}
		
	
	TIM3->ARR = c;
	TIM3->CCR3 = c-300;	
	TIM_Cmd(TIM3, ENABLE);	
	
	return 0;
}

int PPMOUT::enable()
{
	return 0;
}

int PPMOUT::disable()
{
	
	return 0;
}

int PPMOUT::cb()
{
	//printf("cb:%d\n", sending);
	
	if (sending >= channel_count)
	{
		sending = -1;
	}
	
	else
	{
		sending++;
		send();
	}
	return 0;
}

int PPMOUT::write(const int16_t *data, int count)
{
	volatile uint32_t *reg = &TIM3->CCR3;
		
	for(int i=0; i<count; i++)
	{
		channel_data[i] = data[i];
	}
	
	channel_count = count;
	channel_data[channel_count] = 1000;
	
	
	if (sending == -1)
	{
		for(int i=0; i<count; i++)
			sending_data[i] = channel_data[i];

		sending = 0;
		send();
	}
	return 0;
}
