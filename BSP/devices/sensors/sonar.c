#include "sonar.h"
#include "../common/timer.h"
#include "../common/mcu.h"

#define SONAR_TIMEOUT 1000000
#define SOUND_SPEED 3.4f		// sound speed in mm/10us
#define SONAR_MIN 1			// min valid distance
#define SONAR_MAX 5000			// max valid distance

static int result = -1;			// unit: cm
static int64_t last_send = -SONAR_TIMEOUT;
static int rising_time = -1;

// assume timer already initialized
int sonar_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	// TIM6 as timer, 10us resolution, 60000 overflow ~= 600ms
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	TIM_DeInit(TIM6);
	TIM_InternalClockConfig(TIM6);
	#ifdef STM32F1
	TIM_TimeBaseStructure.TIM_Prescaler=719;
	#endif
	#ifdef STM32F4
	TIM_TimeBaseStructure.TIM_Prescaler=839;
	#endif
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=60000-1;
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM6,TIM_FLAG_Update);
	TIM_ARRPreloadConfig(TIM6,DISABLE);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM6,ENABLE);

	// A0 as input
#ifdef STM32F1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
#endif
#ifdef STM32F4
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
#endif

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
#ifdef STM32F1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
#endif
#ifdef STM32F4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
#endif
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// A1 as trigger
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
#ifdef STM32F1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
#endif
#ifdef STM32F4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
#endif
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);

	// EXTI
	EXTI_ClearITPendingBit(EXTI_Line0);
#ifdef STM32F1
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
#endif
#ifdef STM32F4
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
#endif

	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_Init(&EXTI_InitStructure);

	// priority : lowest
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	return 0;
}

int sonar_update()
{
	if (getus() - last_send > SONAR_TIMEOUT)
	{
		// trigger if data arrived or timed out
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
		delayus(20);
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);

		last_send = getus();
		return 0;
	}

	return -1;
}

int sonar_result()
{
	return result;
}

void EXTI0_IRQHandler(void)
{
	volatile int time = TIM6->CNT;
	if(GPIOA->IDR & GPIO_Pin_0)
		rising_time = time;
	else if (rising_time > 0)
	{
		volatile int delta_time = time - rising_time;
		if (delta_time < 0)
			delta_time += 60000;
		delta_time *= SOUND_SPEED/2;
		if (delta_time < SONAR_MIN || delta_time > SONAR_MAX)
			result = -1;
		else
			result = delta_time;
		last_send = -1;
		rising_time = -1;
	}
	else
	{
		result = rising_time = -1;	// no corresponding rising edge.
		last_send = -1;
	}

	EXTI_ClearITPendingBit(EXTI_Line0);
}
