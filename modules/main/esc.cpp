#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/Resources.h>
#include <utils/param.h>
#include <FileSystem/ff.h>
#include <utils/log.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>
#include <HAL/sensors/MPU6000.h>

using namespace HAL;
using namespace devices;
using namespace sensors;
using namespace STM32F4;

struct __FILE { int handle; /* Add whatever you need here */ };
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

extern "C" int fputc(int ch, FILE *f)
{
	if (DEMCR & TRCENA) 
	{
		while (ITM_Port32(0) == 0);
		ITM_Port8(0) = ch;
	}
	return (ch);
}

static void swap(void *buf, int size)
{
	char *p = (char*)buf;
	int i;
	for(i=0; i<size/2; i++)
	{
		char t = p[i];
		p[i] = p[size-1-i];
		p[size-1-i] = t;
	}
}

int init_tim1()
{
	// GPIO:
	// open A8~10 as output (TIM1 channel 1~3)
	GPIO_InitTypeDef GPIO_InitStructure = {0};	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM1);
	
	// Time base configuration
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_Period = 4096;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	
	// OC ( Output Control)
	TIM_OCInitTypeDef  TIM_OCInitStructure;	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	
	// ËÀÇø
	/*
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = 0;  //ËÀÇø0-0xff
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	*/
	TIM1->BDTR |= 1<<15 | 150;


	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);


	TIM_Cmd(TIM1, ENABLE);

	
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	
	return 0;
}

void CalcTimes()
{
}

int apply_power(float Vr1, float Vr2, float Vr3)
{
	uint16_t period = TIM1->ARR;
	float T1, T2;
	int dPWM1, dPWM2, dPWM3;
	int Ta, Tb, Tc;
	
	if( Vr1 >= 0 )
	{
		// (xx1)
		if( Vr2 >= 0 )
		{
			// (x11)
			// Must be Sector 3 since Sector 7 not allowed
			// Sector 3: (0,1,1)  0-60 degrees
			T1 = Vr2;
			T2 = Vr1;
			CalcTimes();
			dPWM1 = Ta;
			dPWM2 = Tb;
			dPWM3 = Tc;
		}
		else
		{
			// (x01)
			if( Vr3 >= 0 )
			{
				// Sector 5: (1,0,1)  120-180 degrees
				T1 = Vr1;
				T2 = Vr3;
				CalcTimes();
				dPWM1 = Tc;
				dPWM2 = Ta;
				dPWM3 = Tb;
			}
			else
			{
				// Sector 1: (0,0,1)  60-120 degrees
				T1 = -Vr2;
				T2 = -Vr3;
				CalcTimes();
				dPWM1 = Tb;
				dPWM2 = Ta;
				dPWM3 = Tc;
			}
		}
	}
	else
	{
		// (xx0)
		if( Vr2 >= 0 )
		{
			// (x10)
			if( Vr3 >= 0 )
			{
				// Sector 6: (1,1,0)  240-300 degrees
				T1 = Vr3;
				T2 = Vr2;
				CalcTimes();
				dPWM1 = Tb;
				dPWM2 = Tc;
				dPWM3 = Ta;
			}
			else
			{
				// Sector 2: (0,1,0)  300-0 degrees
				T1 = -Vr3;
				T2 = -Vr1;
				CalcTimes();
				dPWM1 = Ta;
				dPWM2 = Tc;
				dPWM3 = Tb;
			}
		}
		else
		{            
			// (x00)
			// Must be Sector 4 since Sector 0 not allowed
			// Sector 4: (1,0,0)  180-240 degrees
			T1 = -Vr1;
			T2 = -Vr2;
			CalcTimes();
			dPWM1 = Tc;
			dPWM2 = Tb;
			dPWM3 = Ta;
		}
	}
	
	T1 *= TIM1->ARR;
	T2 *= TIM1->ARR;
	TIM1->CCR1 = (TIM1->ARR-T1-T2)/2;
	TIM1->CCR2 = TIM1->CCR1 + T1;
	TIM1->CCR3 = TIM1->CCR2 + T2;

	return 0;
}

int main()
{
	log_init();
	//bsp_init_all();
	
	/*
	while(0)
	{
		pa1.write(false);
		systimer->delayms(1);
		pa1.write(true);
		systimer->delayms(1);
	}
	*/
	
	init_tim1();
		
	F4GPIO led(GPIOC, GPIO_Pin_4);
	led.set_mode(MODE_OUT_PushPull);
	led.write(true);
	F4GPIO pb5(GPIOB, GPIO_Pin_5);
	pb5.set_mode(MODE_OUT_PushPull);
	pb5.write(false);
	
	F4GPIO scl(GPIOB, GPIO_Pin_13);
	F4GPIO sda(GPIOB, GPIO_Pin_15);
	I2C_SW i2c(&scl, &sda);
	MPU6000 mpu;
	printf("MPU init=%d\n", mpu.init(&i2c, 0xd0));
	mpu.gyro_axis_config(0, 1, 2, 1, 1, 1);
	
	F4GPIO pa0(GPIOA, GPIO_Pin_0);
	pa0.set_mode(MODE_IN);
	
	while(0)
	{
		led.write(false);
		systimer->delayms(500);
		led.write(true);
		systimer->delayms(500);
	}
	
	
	float phase = 0;
	int power = 250;
	int range = 4096;
	float PI = acos(-1.0);
	devices::gyro_data gyro = {0};
	
	while(0)
	{
		led.toggle();
		uint8_t state = 0;
		uint16_t value = 0;

		i2c.read_regs(0x6c, 0x0B, (uint8_t*)&state, 1);
	}
	
	float v1 = 100 * PI;
	float a = v1 / 0.5f;
	
	float v = 0;
	float pos = 0;
	
	// start
	pb5.write(true);
	int A = power * sin(0 + PI * 0 / 3) + range/2;
	int B = power * sin(0 + PI * 2 / 3) + range/2;
	int C = power * sin(0 + PI * 4 / 3) + range/2;
	TIM1->CCR1 = A;
	TIM1->CCR2 = B;
	TIM1->CCR3 = C;
	systimer->delayms(200);
	
	// ramp up	
	int64_t last_t = systimer->gettime();
	
	for(int64_t t=systimer->gettime(); t<500000; t = systimer->gettime())
	{
		float dt = (t-last_t)/1000000.0f;
		last_t = t;
		
		pos += v * dt;
		v += a * dt;
		
		int A = power * sin(pos + PI * 0 / 3) + range/2;
		int B = power * sin(pos + PI * 2 / 3) + range/2;
		int C = power * sin(pos + PI * 4 / 3) + range/2;
		TIM1->CCR1 = A;
		TIM1->CCR2 = B;
		TIM1->CCR3 = C;
		
		if (pos >= 2 * PI)
			pos -= 2*PI;
	}
	
	pos = 0;
	
	while(1)
	{
		int64_t t=systimer->gettime();
		float dt = (t-last_t)/1000000.0f;
		last_t = t;
		pos += v * dt;
		
		pos = 0;
		
		int A = power * sin(pos + PI * 0 / 3) + range/2;
		int B = power * sin(pos + PI * 2 / 3) + range/2;
		int C = power * sin(pos + PI * 4 / 3) + range/2;
		TIM1->CCR1 = A;
		TIM1->CCR2 = B;
		TIM1->CCR3 = C;
		
		if (pos >= 2 * PI)
			pos -= 2*PI;		
	}
	
	while(1)
	{
		//float phase2 = int(phase / (PI*2/8)) * (PI*2/8);
		//phase = 0;
		
		//mpu.read(&gyro);
		
		//printf("\rgyro:%f    ", gyro.z * 180 / PI);
		
		uint8_t state = 0;
		uint16_t value = 0;

		i2c.read_regs(0x6c, 0x0B, (uint8_t*)&state, 1);
		bool mag_detected = state & 0x20;
		bool mag_too_strong = state & 0x08;
		bool mag_too_weak = state & 0x10;
		if (i2c.read_regs(0x6c, 0xc, (uint8_t*)&value, 2) == 0)
		{
			swap(&value, 2);
			
			if (!mag_detected || mag_too_strong || mag_too_weak)
				printf("\ras5600 magnet error    ");
			else			
			{
				printf("\ras5600=%.3f(%d), state=%02x        ", value*360.0f/4095*4-206, value, state);
				phase = (value*360.0f/4095*4-206 + 100)*PI/180;
				//phase = 0;
			}
		}
		else
		{
			printf("\ras5600 error    ");
		}
		
		int A = power * sin(phase + PI * 0 / 3) + range/2;
		int B = power * sin(phase + PI * 2 / 3) + range/2;
		int C = power * sin(phase + PI * 4 / 3) + range/2;
		
		//systimer->delayus(100);
		
		
		//phase += 2 * PI / 10000 * 5;
		
		//if (phase > 2 * PI)
		//	phase -= 2*PI;
		
		//led.write(phase > PI);
		led.toggle();
		
		//if (systimer->gettime() > 500000 && systimer->gettime() < 5000000)
		if (1)
		{
			TIM1->CCR1 = A;
			TIM1->CCR2 = B;
			TIM1->CCR3 = C;
			pb5.write(true);
		}
		else
		{
			
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
			pb5.write(true);
		}
		
	}
}

