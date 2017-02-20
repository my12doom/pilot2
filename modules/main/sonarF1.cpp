#include <stdarg.h>
#include <stdio.h>
#include <HAL/STM32F1/F1GPIO.h>
#include <HAL/STM32F1/F1SysTimer.h>
#include <HAL/STM32F1/F1Interrupt.h>
#include <HAL/sensors/Sonar.h>

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

int main()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	// disable JTAG, it conflicts with TIM3, leave SW-DP enabled, 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

	
	F1GPIO gain2(GPIOA, GPIO_Pin_12);
	F1GPIO trig(GPIOA, GPIO_Pin_8);
	F1GPIO silencer(GPIOA, GPIO_Pin_9);
	F1Interrupt interrupt;
	
	interrupt.init(GPIOB, GPIO_Pin_0, interrupt_falling);
	Sonar sonar;
	sonar.init(&trig, &interrupt, NULL, &gain2);

	
	gain2.set_mode(MODE_OUT_PushPull);
	trig.set_mode(MODE_OUT_PushPull);
	silencer.set_mode(MODE_OUT_PushPull);
	trig.write(false);
	silencer.write(false);
	
	while(1)
	{
		sonar.trigger();
		float v;
		if (sonar.read(&v) == 0)
			printf("%.2f\n", v);
	}
	
	while(1)
	{
		gain2.write(true);
		silencer.write(false);
		
		trig.write(true);
		systimer->delayus(12.5f);
		trig.write(false);
		systimer->delayus(12.5f);
		trig.write(true);
		systimer->delayus(12.5f);
		trig.write(false);
		systimer->delayus(12.5f);
		trig.write(true);
		systimer->delayus(12.5f);
		trig.write(false);
		systimer->delayus(12.5f);
		trig.write(true);
		systimer->delayus(12.5f);
		trig.write(false);
		systimer->delayus(12.5f);
		trig.write(true);
		systimer->delayus(12.5f);
		trig.write(false);
		systimer->delayus(12.5f);
		trig.write(true);
		systimer->delayus(12.5f);
		trig.write(false);
		systimer->delayus(12.5f);
		trig.write(true);
		systimer->delayus(12.5f);
		trig.write(false);
		systimer->delayus(12.5f);
		silencer.write(true);
		
		systimer->delayus(1500);
		
		gain2.write(false);
		
		systimer->delayms(100);
	}
}
