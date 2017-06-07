#include "F0Timer.h"
#include <stdint.h>
#include <stdio.h>
#include <stm32F0xx_misc.h>

using namespace HAL;

static STM32F0::F0Timer * timer_table[8] = {0};
/*
extern "C" void TIM1_BRK_UP_TRG_COM_IRQHandler()
{
	if (timer_table[0])
		timer_table[0]->call_callback();
}

extern "C" void TIM2_IRQHandler()
{
	if (timer_table[1])
		timer_table[1]->call_callback();
}

extern "C" void TIM3_IRQHandler()
{
	if (timer_table[2])
		timer_table[2]->call_callback();
}
*/

extern "C" void TIM14_IRQHandler()
{
	if (timer_table[4])
		timer_table[4]->call_callback();
}

extern "C" void TIM15_IRQHandler()
{
	if (timer_table[5])
		timer_table[5]->call_callback();
}

extern "C" void TIM16_IRQHandler()
{
	if (timer_table[6])
		timer_table[6]->call_callback();
}

extern "C" void TIM17_IRQHandler()
{
	if (timer_table[7])
		timer_table[7]->call_callback();
}

namespace STM32F0
{
	F0Timer::F0Timer(TIM_TypeDef* TIMx)
	{	
		this->TIMx=TIMx;
		cb = NULL;
		TimerInit(TIMx);
	}
	void F0Timer::TimerInit(TIM_TypeDef* TIMx)
	{
		NVIC_InitTypeDef NVIC_InitStructure;
		if(TIM1==TIMx)
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
			
			NVIC_InitStructure.NVIC_IRQChannel = IRQn = TIM1_BRK_UP_TRG_COM_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			
			timer_table[0] = this;
		}
		else if(TIM2==TIMx)
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
			
			NVIC_InitStructure.NVIC_IRQChannel = IRQn = TIM2_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			
			timer_table[1] = this;
		}
		else if(TIM3==TIMx)
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
			
			NVIC_InitStructure.NVIC_IRQChannel = IRQn = TIM3_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			
			timer_table[2] = this;
		}
		else if(TIM14==TIMx)
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
			
			NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			
			timer_table[4] = this;
		}
		else if(TIM15==TIMx)
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15,ENABLE);
			
			NVIC_InitStructure.NVIC_IRQChannel = IRQn = TIM15_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			
			timer_table[5] = this;
		}
		else if(TIM16==TIMx)
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16,ENABLE);
			
			NVIC_InitStructure.NVIC_IRQChannel = IRQn = TIM16_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			
			timer_table[6] = this;
		}
		else if(TIM17==TIMx)
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17,ENABLE);
			
			NVIC_InitStructure.NVIC_IRQChannel = IRQn = TIM17_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			
			timer_table[7] = this;
		}
	}
	
	void F0Timer::set_period(uint32_t period)
	{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_DeInit(TIMx);
		TIM_InternalClockConfig(TIMx);
		SystemCoreClockUpdate();
		TIM_TimeBaseStructure.TIM_Prescaler= SystemCoreClock / 1000000-1;
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
	
	void F0Timer::set_callback(timer_callback cb, void *user_data)
	{		
		this->cb=cb;
		this->user_data = user_data;
	}
	
	void F0Timer::restart()
	{
		TIMx->CNT = 0;
		TIM_ClearITPendingBit(TIMx , TIM_FLAG_Update);
	}
	
	void F0Timer::enable_cb()
	{
		NVIC_EnableIRQ(IRQn);
		TIM_Cmd(TIMx,ENABLE);
	}
	
	void F0Timer::disable_cb()
	{
		TIM_Cmd(TIMx,DISABLE);
		NVIC_DisableIRQ(IRQn);
	}
	
	void F0Timer::call_callback()
	{
		if(cb)
			cb(user_data);
		TIM_ClearITPendingBit(TIMx , TIM_FLAG_Update);
	}
}
