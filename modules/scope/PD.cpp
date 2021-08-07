#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4SysTimer.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4HSBulk.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4Timer.h>
#include <HAL/STM32F4/F4ADC.h>
#include <HAL/Interface/II2C.h>
#include <HAL/aux_devices/OLED_I2C.h>
#include <Protocol/common.h>

#include <stm32f4xx_tim.h>
#include <stm32f4xx_dac.h>

#include "adc.h"

using namespace STM32F4;
using namespace HAL;

F4GPIO led0(GPIOB, GPIO_Pin_5);
F4Interrupt trig;
F4Timer timer(TIM4);
dma_adc adc;

devices::OLED96 led;

int pwm_set(int16_t value);
int pwm_init();

int counter = 0;
float avgf;
float lpf_p = NAN;
float lpf_alpha = 0.9;
int pulse_length = 0;
int64_t pulse_start = 0;

int count;

void capture_start()
{
	adc.init(GPIOA, 2);
	adc.begin();
	counter ++;
}

void capture_end(bool discard_data = false)
{
	adc.stop();

	if (!discard_data)
	{
		int avg = 0;
		count = adc.full() ? max_adc_data_count : adc.pos();
		for(int i=0; i<count; i++)
			avg += adc.adc_data[i];

		avgf = (float)avg/count;

		lpf_p = isnan(lpf_p) ? avgf : (lpf_p * lpf_alpha + avgf*(1-lpf_alpha));
	}
}

void trig_cb(void *p, int flags)
{
	if (flags == interrupt_rising)
	{
		pulse_length = systimer->gettime() - pulse_start;
		capture_end();
		led0.write(0);
	}
	else if (flags == interrupt_falling)
	{
		pulse_start = systimer->gettime();

		capture_start();
		led0.write(1);
	}
	else
	{
	}
}

void pulse_main()
{
	F4GPIO test_on_led(GPIOA, GPIO_Pin_8);
	test_on_led.set_mode(HAL::MODE_OUT_PushPull);
	test_on_led.write(0);

	F4GPIO button(GPIOB, GPIO_Pin_15);
	button.set_mode(HAL::MODE_IN);

	bool test_on = false;
	
	F4GPIO channels[8] = 
	{
		F4GPIO(GPIOB, GPIO_Pin_0),
		F4GPIO(GPIOB, GPIO_Pin_1),
		F4GPIO(GPIOB, GPIO_Pin_9),
		F4GPIO(GPIOB, GPIO_Pin_8),
		F4GPIO(GPIOB, GPIO_Pin_7),
		F4GPIO(GPIOB, GPIO_Pin_6),
		F4GPIO(GPIOB, GPIO_Pin_5),
		F4GPIO(GPIOB, GPIO_Pin_4),
	};
	
	for(int i=0; i<8; i++)
	{
		channels[i].set_mode(HAL::MODE_OUT_PushPull);
		channels[i].write(0);
	}
	
	bool last_button_state = false;

	while(1)
	{

		test_on_led.write(test_on);
		bool s = button.read();

		if (s && !last_button_state)
			test_on = !test_on;

		last_button_state = s;

		int64_t t = systimer->gettime();
		
		int on_time = 10000;
		int off_time = 90000;
		
		channels[0].write((systimer->gettime() % (on_time+off_time)) < on_time && test_on);
	}
	
	
}


int DAC_test()
{
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	DAC_InitTypeDef DAC_InitStruct;
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits11_0;
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStruct);
	DAC_Init(DAC_Channel_2, &DAC_InitStruct);
	
	DAC_Cmd(DAC_Channel_2, ENABLE);
	DAC_Cmd(DAC_Channel_1, ENABLE);

	while(1)
	{
		int v = sin(systimer->gettime() * 2 * PI / 1000000) * 2047 + 2048;
		DAC->DHR12R2=v;
		DAC->DHR12R1=4095-v;
		
		DAC_DualSoftwareTriggerCmd(ENABLE);
	}
}


int main()
{
	// trigger at falling edge
	// stop at adc full or rising edge
	trig.init(GPIOA, GPIO_Pin_8, interrupt_rising_or_falling);
	trig.set_callback(trig_cb, NULL);
	
	pwm_init();
	pwm_set(1.5*4095/3.3);

	led0.set_mode(HAL::MODE_OUT_PushPull);
	led0.write(0);

	F4GPIO scl(GPIOA, GPIO_Pin_10);
	F4GPIO sda(GPIOA, GPIO_Pin_9);
	I2C_SW i2c(&scl, &sda);
	i2c.set_speed(1);
	led.init(&i2c, 0x78);
	led.show_str(0, 0, "HelloWorld");


	while(1)
	{
		float v = lpf_p * 3.3 / 4095;
		float dbm = -40.61*v + 51.39;
		float dbm5g = -39.85*v + 56.18;
		char tmp[100];

		// free run entry (no trig for 500ms)
		if (systimer->gettime() - pulse_start > 500000)
		{
			capture_start();
			systimer->delayus(500);
			capture_end();
			
			pulse_length = 0;

			led0.write(0);
		}
		

		if (!isnan(lpf_p) && fabs(v) < 3.3)
		{
			sprintf(tmp, "%dus, %.1fdbm(5G)  ", pulse_length, dbm5g);
			led.show_str(0,0,tmp);
			sprintf(tmp, "%.3fV, %.1fdbm(2G)", v, dbm);
			led.show_str(0,3,tmp);

			systimer->delayms(16);
		}
	}
}

int pwm_init()
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	// open B1(TIM4_CH3) as output
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	
	// Time base configuration
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = 4094;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
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
	
	TIM_SetCompare1(TIM3, 2048);
}

int pwm_set(int16_t value)
{
	TIM_SetCompare1(TIM3, value);

	return 0;
}
