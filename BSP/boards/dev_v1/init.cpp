#include <BSP\Resources.h>
#include <BSP\boards\dev_v1\init.h>
#include "stm32F4xx.h"


//Define Battery Voltage Funtion Pin and channel:
#include <BSP\boards\dev_v1\BatteryVoltage.h>
#include <BSP\devices\IBatteryVoltage.h>
using namespace BSP;
using namespace STM32F4;



//Define LED Function Pin:
#include <BSP\devices\ILED.h>
#include <BSP\boards\dev_v1\LED.h>
//set gpioc pin4:
F4GPIO f4gpioC4(GPIOC,GPIO_Pin_4);
LED led_red(&f4gpioC4);
LED * pLED_RED= &led_red;
//set gpio pin5:
F4GPIO f4gpioC5(GPIOC,GPIO_Pin_5);
LED led_green(&f4gpioC5);
LED * pLED_GREEN= &led_green;
void init_led()
{
	f4gpioC4.set_mode(MODE_OUT_PushPull);
	f4gpioC5.set_mode(MODE_OUT_PushPull);
	manager.register_LED("LED_RED",pLED_RED);
	manager.register_LED("LED_GREEN",pLED_GREEN);
}

//Define TIMER Function:
#include <HAL\STM32F4\F4Timer.h>
#include <HAL\Interface\ITimer.h>
//Timer1
F4Timer f4TIM1(TIM1);
ITimer * pTIM1 = &f4TIM1;

void TIM1_Callback()
{
	manager.getLED("LED_RED")->toggle();
}
void init_timer1()
{
	manager.register_Timer("Timer1",pTIM1);
	manager.getTimer("Timer1")->set_period(50000);
	manager.getTimer("Timer1")->set_callback(TIM1_Callback);
}
extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
	f4TIM1.call_callback();
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
	manager.register_UART("UART4",pUART4);
}


#include <HAL\STM32F4\F4SPI.h>
#include "sensors.h"
F4SPI spi2;
F4GPIO cs_mpu(GPIOE, GPIO_Pin_8);
F4GPIO cs_ms5611(GPIOE, GPIO_Pin_7);
F4GPIO cs_hmc5983(GPIOE, GPIO_Pin_10);
sensors::MPU6000 mpu6000device;
sensors::MS5611_SPI ms5611device;
sensors::HMC5983 hmc5983device;
void init_sensors()
{
	spi2.init(SPI2);
	mpu6000device.init(&spi2, &cs_mpu);
	ms5611device.init(&spi2, &cs_ms5611);
	hmc5983device.init(&spi2, &cs_hmc5983);
	
	static dev_v1::mpu6000res res6000(&mpu6000device);
	static dev_v1::HMC5983res res5983(&hmc5983device);
	static dev_v1::MS5611res res5611(&ms5611device);
	
	
	manager.register_accelerometer(&res6000);
	manager.register_gyroscope(&res6000);
	manager.register_magnetometer(&res5983);
	manager.register_barometer(&res5611);
	
}

extern "C" void UART4_IRQHandler(void)
{
	f4uart.UART4_IRQHandler();
}
extern "C" void DMA1_Stream4_IRQHandler()
{
	f4uart.DMA1_Steam4_IRQHandler();
}


//Define BattertVoltage Function:
F4ADC f4adc(ADC1,ADC_Channel_4);
BatteryVoltage battery_voltage(&f4adc,1.0);
IBatteryVoltage * pBattery_Voltage= &battery_voltage;
void init_BatteryVoltage()
{
	manager.register_BatteryVoltage("BattertVoltage",pBattery_Voltage);
	//manager.getBatteryVoltage("BatteryVoltage")->read();
}

int bsp_init_all()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	init_led();
	init_uart4();
	init_timer1();
	init_BatteryVoltage();
	init_sensors();
	
	return 0;
}
