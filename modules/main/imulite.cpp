#include <stdarg.h>

#include <HAL/STM32F1/F1GPIO.h>
#include <HAL/STM32F1/F1SPI.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/sensors/MPU6000.h>
#include <HAL/sensors/MS5611_SPI.h>

#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>


extern "C"
{
#include <utils/SEGGER_RTT.h>
}

using namespace STM32F1;
using namespace HAL;
using namespace sensors;
using namespace devices;

#define LOGI(...) log_printf(##__VA_ARGS__)

int log_printf(const char*format, ...)
{
	char buffer[512];
	
	va_list args;
	va_start (args, format);
	int count = vsprintf (buffer,format, args);
	va_end (args);
	
	if (count < 0)
		return count;
	
	SEGGER_RTT_WriteString(0, buffer);
	
	return 0;
}

int init_heater()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// disable JTAG, it conflicts with TIM3, leave SW-DP enabled, 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = {0};
	TIM_TimeBaseStructure.TIM_Period = 10000-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_Cmd(TIM3, ENABLE);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_OCInitTypeDef  TIM_OCInitStructure = {0};
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 100;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	
	return 0;
}

int set_heater(float v)
{
	TIM3->CCR3 = v*9999;
	
	return 0;
}

float limit(float v, float l, float h)
{
	if (v<l)
		return l;
	if (v>h)
		return h;
	return v;
}

int main()
{
	F1GPIO a5(GPIOA, GPIO_Pin_12);
	a5.set_mode(MODE_OUT_PushPull);
	a5.write(true);
	
	F1GPIO cs_mpu(GPIOB, GPIO_Pin_2);
	F1GPIO cs_5611(GPIOA, GPIO_Pin_4);
	cs_mpu.set_mode(MODE_OUT_PushPull);
	cs_mpu.write(true);
	cs_5611.set_mode(MODE_OUT_PushPull);
	cs_5611.write(true);
	F1SPI spi(SPI1);
	MPU6000 mpu;
	MS5611_SPI ms5611;
	
	if (mpu.init(&spi, &cs_mpu) != 0 || ms5611.init(&spi, &cs_5611) != 0)
	{
		LOGI("sensor error\n");
		
		while(1)
		{
			a5.write(true);
			systimer->delayms(1);
			a5.write(false);
			systimer->delayms(1);
		}
	}
	
	init_heater();
	
	float I = 0;
	float set = 55.0f;
	float Kp = 0.3f;
	float Ki = 0.2f;
	float imax = 0.5f;
	
	int64_t last_t = systimer->gettime();
	
	while(1)
	{
		int64_t t = systimer->gettime();
		float dt= (t-last_t) / 1000000.0f;
		last_t = t;
					
		accelerometer_data d;
		mpu.read(&d);
		
		systimer->delayms(100);
		
		float P = set - d.temperature;
		I += P * dt;
		I = limit(I, -imax, +imax);
		
		float v = limit(P*Kp + I*Ki, 0, 1);
		
		set_heater(v);
		
		LOGI("\rt=%f/%f, v=%d%    /", (d.temperature), (set), int(v*100));
	}
}
