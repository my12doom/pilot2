#pragma once
#include <stdint.h>
#include "F4Timer.h"
#include "stm32f4xx_tim.h"
using namespace HAL;
namespace STM32F4
{
	F4Timer::F4Timer(TIM_TypeDef* TIMx)
	{	
		this->TIMx=TIMx;
	}
	void F4Timer::TimerInit(TIM_TypeDef* TIMx)
	{
		NVIC_InitTypeDef NVIC_InitStructure;
		if(TIM1==TIMx)
		{	
			NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(TIM2==TIMx)
		{
			NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(TIM3==TIMx)
		{
			NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(TIM4==TIMx)
		{
			NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(TIM5==TIMx)
		{
			NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(TIM6==TIMx)
		{
			NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;		// TODO: IRQ?
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
	}
	void F4Timer::set_period(uint32_t period)
	{
		if(TIM1==TIMx)
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
		if(TIM2==TIMx)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
		if(TIM3==TIMx)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
		if(TIM4==TIMx)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
		if(TIM5==TIMx)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
		if(TIM6==TIMx)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_DeInit(TIMx);
		TIM_InternalClockConfig(TIMx);
		TIM_TimeBaseStructure.TIM_Prescaler=167;//1Mhz 1us 65536
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period=period;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
		TIM_TimeBaseInit(TIMx,&TIM_TimeBaseStructure);
		TIM_ClearFlag(TIMx,TIM_FLAG_Update);
		TIM_ARRPreloadConfig(TIMx,DISABLE);
		TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE);
		TIM_Cmd(TIMx,ENABLE);
		TimerInit(this->TIMx);
	}
	void F4Timer::set_callback(timer_callback cb)
	{		
		this->cb=cb;
	}
	void F4Timer::call_callback()
	{
		if(TIM1==TIMx)
			TIM_ClearITPendingBit(TIM1 , TIM_FLAG_Update);
		else if(TIM2==TIMx)
			TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
		else if(TIM3==TIMx)
			TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);
		else if(TIM4==TIMx)
			TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);
		else if(TIM5==TIMx)
			TIM_ClearITPendingBit(TIM5 , TIM_FLAG_Update);
		else if(TIM6==TIMx)
			TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);
		else if(TIM7==TIMx)
			TIM_ClearITPendingBit(TIM7 , TIM_FLAG_Update);
		else if(TIM8==TIMx)
			TIM_ClearITPendingBit(TIM8 , TIM_FLAG_Update);
		cb();
	}

}
