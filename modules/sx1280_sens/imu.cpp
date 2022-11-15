#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <HAL/Interface/ISysTimer.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4SPI.h>
#include <HAL/sensors/MPU6000.h>
#include <stm32f4xx_tim.h>

using namespace STM32F4;
using namespace sensors;
using namespace devices;

F4SPI spi(SPI1);
F4GPIO cs(GPIOC, GPIO_Pin_5);

MPU6000 mpu;
gyro_data dgyro;
accelerometer_data dacc;


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
	
	TIM_SetCompare1(TIM3, 10);
	
	return 0;
}

int pwm_set(int16_t value)
{
	TIM_SetCompare1(TIM3, value);

	return 0;
}


int o = 0;
float max_power = 1;
float kp = 0.05f;
float kI = 0.01f;
float tset = 50;
float control = 0;
float I = 0;
float p = 0;
int main()
{
	pwm_init();
	spi.set_mode(0,0);
	spi.set_speed(10e6);
	//while(1)
		spi.txrx(0x55);
	
	//while(1)
		o = mpu.init(&spi, &cs);
	


	float dt = 1e-3;
	while(1)
	{
		mpu.read(&dgyro);
		mpu.read(&dacc);

		p = tset - dacc.temperature;
		if (control > 0 && control < 1)
		I += p* dt;
		
		float imax = max_power / kI;
		if (I<-imax)
			I = -imax;
		if (I>imax)
			I = imax;
		
		systimer->delayms(1);
		
		control = p * kp + I * kI;
		if (control > max_power)
			control = max_power;
		if (control < 0)
			control = 0;
		
		pwm_set(control*4094);		
	}
}
