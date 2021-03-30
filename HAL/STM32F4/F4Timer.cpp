#include <stdint.h>
#include <stdio.h>
#include "F4Timer.h"
#include "stm32f4xx_tim.h"
using namespace HAL;
namespace STM32F4
{
	F4Timer::F4Timer(TIM_TypeDef* TIMx)
	{	
		this->TIMx=TIMx;
		cb = NULL;
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
		if(TIM7==TIMx)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
		if(TIM8==TIMx)
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	}
	void F4Timer::TimerInit(TIM_TypeDef* TIMx)
	{
		NVIC_InitTypeDef NVIC_InitStructure;
		if(TIM1==TIMx)
		{	
			NVIC_InitStructure.NVIC_IRQChannel = IRQn = TIM1_UP_TIM10_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(TIM2==TIMx)
		{
			NVIC_InitStructure.NVIC_IRQChannel = IRQn = TIM2_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(TIM3==TIMx)
		{
			NVIC_InitStructure.NVIC_IRQChannel = IRQn = TIM3_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(TIM4==TIMx)
		{
			NVIC_InitStructure.NVIC_IRQChannel = IRQn = TIM4_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(TIM5==TIMx)
		{
			NVIC_InitStructure.NVIC_IRQChannel = IRQn = TIM5_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(TIM6==TIMx)
		{
			NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;		// TODO: IRQ?
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(TIM7==TIMx)
		{
			NVIC_InitStructure.NVIC_IRQChannel = IRQn = TIM7_IRQn;		// TODO: IRQ?
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(TIM8==TIMx)
		{
			NVIC_InitStructure.NVIC_IRQChannel = IRQn = TIM8_UP_TIM13_IRQn;		// TODO: IRQ?
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
	}
	void F4Timer::set_period(uint32_t period)
	{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_DeInit(TIMx);
		TIM_InternalClockConfig(TIMx);
		TIM_TimeBaseStructure.TIM_Prescaler = (TIMx == TIM1 || TIMx == TIM8) ? 167: 83;//1Mhz 1us 65536
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period=period-1;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
		TIM_TimeBaseInit(TIMx,&TIM_TimeBaseStructure);
		TIM_ClearFlag(TIMx,TIM_FLAG_Update);
		TIM_ARRPreloadConfig(TIMx,DISABLE);
		TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE);
		TIM_Cmd(TIMx,ENABLE);
		TimerInit(this->TIMx);
	}
	void F4Timer::set_callback(timer_callback cb, void *user_data /*= NULL*/)
	{		
		this->cb=cb;
		this->user_data = user_data;
	}
	
	void F4Timer::restart()
	{
		TIMx->CNT = 0;
		TIM_ClearITPendingBit(TIMx , TIM_FLAG_Update);
	}
	void F4Timer::enable_cb()
	{
		NVIC_EnableIRQ(IRQn);
		TIM_Cmd(TIMx,ENABLE);
	}
	
	void F4Timer::disable_cb()
	{
		TIM_Cmd(TIMx,DISABLE);
		NVIC_DisableIRQ(IRQn);
	}
	
	void F4Timer::call_callback()
	{
		if (TIM_GetITStatus(TIMx, TIM_FLAG_Update))
		{
			TIM_ClearITPendingBit(TIMx , TIM_FLAG_Update);
			if(cb)
				cb(user_data);
		}
	}
}
