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
LED led_red(&f4gpioC1);
LED * pLED_RED= &led_red;
void init_led()
{
	//manager.Register_LED("LED_RED",pLED_RED);
}

//Define TIMER Function:
#include <HAL\STM32F4\F4Timer.h>
#include <HAL\Interface\ITimer.h>
//Timer1
F4Timer f4TIM1(TIM1);
ITimer * pTIM1 = &f4TIM1;

void TIM1_Callback()
{
	pLED_RED->toggle();
}
void init_timer1()
{
	pTIM1->set_period(65000);//50ms
	pTIM1->set_callback(TIM1_Callback);
}
extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
	pTIM1->call_callback();
}
//Timer2


//Define UART Funtion:
#include <HAL\STM32F4\F4UART.h>
#include <HAL\Interface\IUART.h>
F4UART f4uart(UART4);
IUART * pUART4 = &f4uart;
void init_uart4()
{
	pUART4->set_baudrate(115200);
	pUART4->write("12345\n", 6);
}

extern "C" void UART4_IRQHandler(void)
{
	f4uart.UART4_IRQHandler();
}
extern "C" void DMA1_Stream4_IRQHandler()
{
	f4uart.DMA1_Steam4_IRQHandler();
}


