#include "board.h"

#include <HAL/STM32F1/F1SPI.h>
#include <HAL/STM32F1/F1GPIO.h>
#include <HAL/STM32F1/F1Interrupt.h>
#include <HAL/STM32F1/F1SysTimer.h>
#include <misc.h>
#include <HAL/STM32F1/F1Timer.h>
#include <HAL/STM32F1/F1UART.h>
#include <stm32f10x.h>
#include <string.h>

#include <stm32f10x_tim.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <misc.h>
#include <Protocol/common.h>

using namespace STM32F1;
using namespace HAL;

int16_t adc_data[6] = {0};

F1GPIO _cs(GPIOA, GPIO_Pin_8);
F1GPIO _ce(GPIOA, GPIO_Pin_9);
F1GPIO _irq(GPIOA, GPIO_Pin_10);

F1GPIO _dbg(GPIOA, GPIO_Pin_4);
F1GPIO _dbg2(GPIOA, GPIO_Pin_5);

F1SPI _spi;
F1Interrupt _interrupt;
F1Timer _timer(TIM2);

extern "C" void TIM2_IRQHandler(void)
{
	_timer.call_callback();
}

// channel index starts from 0
const int channel_count = 6;
class PWMOUT : public HAL::IRCOUT
{
public:
	PWMOUT();
	~PWMOUT(){}

	virtual int write(const int16_t *data, int count, int start_channel);
	virtual int read(int16_t *data, int count, int start_channel);
	virtual int get_channel_count(){return 6;}
protected:
	int16_t channel_data[channel_count];
};


int board_init()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);		
	
	cs = &_cs;
	ce = &_ce;
	irq = &_irq;
	dbg = &_dbg;
	dbg2 = &_dbg2;
	SCL = NULL;
	SDA = NULL;
	spi = &_spi;
	interrupt = &_interrupt;
	timer = &_timer;
	
	//	
	_spi.init(SPI2);
	_interrupt.init(GPIOA, GPIO_Pin_10, interrupt_falling);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	static PWMOUT pwmout;
	ppm = &pwmout;
			
	return 0;
}

PWMOUT::PWMOUT()
{
	memset(channel_data, 0, sizeof(channel_data));

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

	GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// Time base configuration
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = 2500-1;		// 1Mhz/2500 = 400hz
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;		// 72Mhz/72 = 1Mhz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;
	TIM4->CCR4 = 0;
	
	// remap TIM3
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

	// disable JTAG, it conflicts with TIM3, leave SW-DP enabled, 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

// return num channel read
// return any possible read if index overrun
// return negative value to indicate an error
int PWMOUT::read(int16_t *out, int start_channel, int max_count)
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

// return num channel written
// generate an error if index overrun/underrun, and won't update any channel
// return negative value to indicate an error
int PWMOUT::write(const int16_t *data, int count, int start_channel)
{
	volatile uint16_t *registers[channel_count] =
	{
		&TIM4->CCR4,
		&TIM4->CCR3,
		&TIM4->CCR2,
		&TIM4->CCR1,
		&TIM3->CCR2,
		&TIM3->CCR1,
	};
	
	int16_t mix[6] = {data[0], data[1], data[2], data[3], data[4], data[5]};
	
	if (mix[0] && mix[1] && 0)
	{
		if (1)
			mix[0] = 3000 - data[0];
		if (1)
			mix[1] = 3000 - data[1];
		
		int16_t tmp[2] = {(mix[0] + mix[1]) - 1500, (mix[0] - mix[1]) + 1500};
		mix[0] = limit(tmp[0], 1000, 2000);
		mix[1] = limit(tmp[1], 1000, 2000);
	}
	
	
	if (start_channel < 0 || start_channel + count > channel_count)
		return -1;
	
	for(int i=0; i<count; i++)
	{
		channel_data[i+start_channel] = mix[i];
		registers[i+start_channel][0] = mix[i];
	}

	TIM_Cmd(TIM4, ENABLE);
	TIM_Cmd(TIM3, ENABLE);

	return 0;
}


