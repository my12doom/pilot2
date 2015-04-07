#include <BSP\Resources.h>
#include <BSP\boards\dev_v1\init.h>
#include "stm32F4xx.h"
#include "RCIN.h"
#include "RCOUT.h"

//Define Battery Voltage Funtion Pin and channel:
#include <BSP\boards\dev_v1\BatteryVoltage.h>
#include <BSP\devices\IBatteryVoltage.h>
#include <BSP/devices/sensors/UartNMEAGPS.h>
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

static F4Timer f4TIM1(TIM1);
static F4Timer f4TIM4(TIM4);
void init_timer1()
{
	manager.register_Timer("mainloop", &f4TIM1);
	manager.register_Timer("log", &f4TIM4);
}
extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
	f4TIM1.call_callback();
}

extern "C" void TIM4_IRQHandler(void)
{
	f4TIM4.call_callback();
}

//Timer2


//Define UART Funtion:
#include <HAL\STM32F4\F4UART.h>
#include <HAL\Interface\IUART.h>

//For uart4:
F4UART f4uart4(UART4);
IUART * pUART4 = &f4uart4;
void init_uart4()
{
	pUART4->set_baudrate(115200);
	pUART4->write("This is UART4\n", 6);
	manager.register_UART("UART4",pUART4);
}
extern "C" void UART4_IRQHandler(void)
{
	f4uart4.UART4_IRQHandler();
}
extern "C" void DMA1_Stream4_IRQHandler()
{
	f4uart4.DMA1_Steam4_IRQHandler();
}
//For usart3:
F4UART f4uart3(USART3);
IUART * pUART3 = &f4uart3;
void init_uart3()
{
	pUART3->set_baudrate(115200);
	pUART3->write("This is UART3\n", 6);
	manager.register_UART("UART3",pUART3);
}
extern "C" void USART3_IRQHandler(void)
{
	f4uart3.USART3_IRQHandler();
}
extern "C" void DMA1_Stream3_IRQHandler()
{
	f4uart3.DMA1_Steam3_IRQHandler();
}

//For usart2:
F4UART f4uart2(USART2);
IUART * pUART2 = &f4uart2;
void init_uart2()
{
	pUART2->set_baudrate(115200);
	pUART2->write("This is UART2\n", 14);
	manager.register_UART("UART2",pUART2);
}
extern "C" void USART2_IRQHandler(void)
{
	f4uart2.USART2_IRQHandler();
}
extern "C" void DMA1_Stream6_IRQHandler()
{
	f4uart2.DMA1_Steam6_IRQHandler();
}


//For usart1:
F4UART f4uart1(USART1);
IUART * pUART1 = &f4uart1;
void init_uart1()
{
	pUART1->set_baudrate(115200);
	pUART1->write("12345\n", 6);
	manager.register_UART("UART1",pUART1);
}
extern "C" void USART1_IRQHandler(void)
{
	f4uart1.USART1_IRQHandler();
}
extern "C" void DMA2_Stream7_IRQHandler()
{
	f4uart1.DMA2_Steam7_IRQHandler();
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

int init_RC()
{
	static dev_v1::RCIN rcin;
	static dev_v1::RCOUT rcout;
	
	manager.register_RCIN(&rcin);
	manager.register_RCOUT(&rcout);
	
	return 0;
}

int init_GPS()
{
	static sensors::UartNMEAGPS gps;
	gps.init(&f4uart3, 115200);
	
	manager.register_GPS(&gps);
	
	return 0;
}

//Define BattertVoltage Function:
F4ADC f4adc1_Ch2(ADC1,ADC_Channel_2);
BatteryVoltage battery_voltage(&f4adc1_Ch2,1.0);
IBatteryVoltage * pBattery_Voltage= &battery_voltage;
void init_BatteryVoltage()
{
	manager.register_BatteryVoltage("BatteryVoltage",pBattery_Voltage);
	//manager.getBatteryVoltage("BatteryVoltage")->read();
}
//Define BattertVoltage Function:
F4ADC f4adc2_Ch8(ADC2,ADC_Channel_8);
BatteryVoltage battery_current(&f4adc2_Ch8,1.0);
IBatteryVoltage * pBattery_Current= &battery_current;
void init_BatteryCurrent()
{
	manager.register_BatteryVoltage("BatteryCurrent",pBattery_Voltage);
	//manager.getBatteryVoltage("BatteryVoltage")->read();
}

int bsp_init_all()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	init_led();
	init_BatteryVoltage();
	init_BatteryCurrent();
//	init_uart4();
	init_uart3();
	init_uart2();
	init_timer1();
//	init_uart1();
	init_RC();
//	init_sensors();
	init_GPS();
	
	return 0;
}

void reset_system()
{
	NVIC_SystemReset();
}