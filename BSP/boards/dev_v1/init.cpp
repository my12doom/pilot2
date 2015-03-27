#include <BSP\Resources.h>
#include "stm32F4xx.h"


//Define Battery Voltage Funtion Pin and channel:
#include <BSP\boards\dev_v1\BatteryVoltage.h>
#include <BSP\devices\IBatteryVoltage.h>
using namespace BSP;
using namespace STM32F4;
F4ADC f4adc(ADC1,ADC_Channel_4);
BatteryVoltage battery_voltage(&f4adc,1.0);
IBatteryVoltage * pBattery_Voltage= &battery_voltage;


//Define LED Function Pin:
#include <BSP\devices\ILED.h>
#include <BSP\boards\dev_v1\LED.h>
F4GPIO f4gpioC1(GPIOC,GPIO_Pin_1);
LED led(&f4gpioC1);
LED * pLED= &led;


//Define TIMER Function:
//Timer1
#include <HAL\STM32F4\F4Timer.h>
#include <HAL\Interface\ITimer.h>
F4Timer f4TIM1(TIM1);
ITimer * pTIM1 = &f4TIM1;
void TIM1_Callback()
{
}
void init_timer1()
{
	pTIM1->set_period(50000);//50ms
	pTIM1->set_callback(TIM1_Callback);
}
extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
	pTIM1->call_callback();
}
//Timer2
