#include "F0Interrupt.h"

#include <protocol/common.h>
#include <HAL/Interface/ISysTimer.h>

#include "stm32F0xx_exti.h"
#include "stm32F0xx_misc.h"

using namespace HAL;
static STM32F0::F0Interrupt *int_table[16] = {0};

extern "C" void EXTI4_15_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line4) != RESET)
	{
		if (int_table[4])
			int_table[4]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
	if (EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		if (int_table[5])
			int_table[5]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
	if (EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		if (int_table[6])
			int_table[6]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line6);
	}
	if (EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		if (int_table[7])
			int_table[7]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
	if (EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		if (int_table[8])
			int_table[8]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
	if (EXTI_GetITStatus(EXTI_Line9) != RESET)
	{
		if (int_table[9])
			int_table[9]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line9);
	}
	if (EXTI_GetITStatus(EXTI_Line10) != RESET)
	{
		if (int_table[10])
			int_table[10]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line10);
	}
	if (EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
		if (int_table[11])
			int_table[11]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line11);
	}
	if (EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		if (int_table[12])
			int_table[12]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
	if (EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
		if (int_table[13])
			int_table[13]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line13);
	}
	if (EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		if (int_table[14])
			int_table[14]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line14);
	}
	if (EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		if (int_table[15])
			int_table[15]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line15);
	}
}
extern "C" void EXTI2_3_IRQHandler(void)
{
	int64_t t = systimer->gettime();
	if (EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		if (int_table[2])
			int_table[2]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
	if (EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
		if (int_table[3])
			int_table[3]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line3);
	}
}

extern "C" void EXTI0_1_IRQHandler(void)
{
	int64_t t = systimer->gettime();
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		if (int_table[0])
			int_table[0]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		if (int_table[1])
			int_table[1]->call_callback();
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

namespace STM32F0
{	
	int Pin2PinSource(uint32_t GPIO_Pin)
	{
		for(int i=0; i<16; i++)
			if (GPIO_Pin == (1 << i))
				return i;
		return -1;
	}
	
	EXTITrigger_TypeDef flag2trigger(int flag)
	{
		if (flag == interrupt_rising)
			return EXTI_Trigger_Rising;
		if (flag == interrupt_falling)
			return EXTI_Trigger_Falling;
		if (flag == interrupt_rising_or_falling)
			return EXTI_Trigger_Rising_Falling;
		
		return EXTI_Trigger_Rising_Falling;
	}
	
	int port2port_source(GPIO_TypeDef* GPIOx)
	{
		// only A - F supported yet.
		if (GPIOx < GPIOA || GPIOx > GPIOF)
			return -1;		
		
		return ((uint32_t)GPIOx - (uint32_t)GPIOA) / 0x0400;
	}
	
	IRQn_Type pin2irqn(uint32_t GPIO_Pin)
	{
		if (GPIO_Pin == GPIO_Pin_0 || GPIO_Pin == GPIO_Pin_1)
			return EXTI0_1_IRQn;
		if (GPIO_Pin == GPIO_Pin_2 || GPIO_Pin == GPIO_Pin_3)
			return EXTI2_3_IRQn;
		if (GPIO_Pin == GPIO_Pin_3 || GPIO_Pin == GPIO_Pin_4|| (GPIO_Pin >= GPIO_Pin_5 && GPIO_Pin <= GPIO_Pin_15))
			return EXTI4_15_IRQn;
		
		return (IRQn_Type)-1;
	}
	
	F0Interrupt::F0Interrupt()
	{
	}
	
	F0Interrupt::~F0Interrupt()
	{
		for(int i=0; i<16; i++)
			if (int_table[i] == this)
				int_table[i] = NULL;
	}

	bool F0Interrupt::init(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, int flag)
	{
		this->GPIOx = GPIOx;
		this->GPIO_Pin = GPIO_Pin;
		this->flag = flag;

		int port_source = port2port_source(GPIOx);
		int pin_source =  Pin2PinSource(GPIO_Pin);
		if (pin_source < 0 || port_source < 0)
		{
			//LOGE("invalid interrupt\n");
			return false;
		}
		
		if (int_table[pin_source])
			;//LOGE("warning: duplicate exti %d\n", pin_source);
		int_table[pin_source] = this;		
		
		// open everything....
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB,ENABLE);

		// configure GPIO.
		GPIO_InitTypeDef GPIO_InitStructure = {0};
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOx, &GPIO_InitStructure);

		// configure exti
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		SYSCFG_EXTILineConfig(port_source, pin_source);

		EXTI_ClearITPendingBit(GPIO_Pin);
		
		EXTI_InitTypeDef EXTI_InitStructure;
		EXTI_InitStructure.EXTI_Line = GPIO_Pin;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = flag2trigger(flag);
		EXTI_Init(&EXTI_InitStructure);
		
		/*
		EXTI_InitTypeDef EXTI_InitStructure;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = flag2trigger(flag);
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_InitStructure.EXTI_Line = GPIO_Pin;
		
		EXTI_ClearITPendingBit(GPIO_Pin);
		GPIO_EXTILineConfig(port_source, pin_source);
		EXTI_Init(&EXTI_InitStructure);
		*/

		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = pin2irqn(GPIO_Pin);
		NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);		
		
		return true;
	}

	int F0Interrupt::enable()
	{
		NVIC_EnableIRQ(pin2irqn(GPIO_Pin));
		return 0;
	}

	int F0Interrupt::disable()
	{
		NVIC_DisableIRQ(pin2irqn(GPIO_Pin));
		return 0;
	}
	
	void F0Interrupt::set_callback(HAL::interrupt_callback cb, void *parameter)
	{
		this->cb=cb;
		this->parameter = parameter;
	}
	
	void F0Interrupt::call_callback()
	{
		if(cb)
			cb(parameter, flag);
	}
}
