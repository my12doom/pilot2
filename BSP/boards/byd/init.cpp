#include <stdio.h>
#include <protocol/common.h>
#include <HAL\Resources.h>
#include "RCIN.h"
#include "RCOUT.h"
#include "AsyncWorker.h"
#include <HAL\STM32F4\F4Timer.h>
#include <HAL\STM32F4\F4VCP.h>
#include <HAL/sensors/UartUbloxNMEAGPS.h>
#include <HAL/sensors/Sonar.h>
#include <HAL\Interface\ILED.h>
#include <HAL/sensors/PX4Flow.h>
#include <utils/param.h>
#include "RGBLED.h"

extern "C" const char bsp_name[] = "BYD";

using namespace HAL;
using namespace devices;
using namespace STM32F4;
using namespace dev_v2;
using namespace sensors;

void init_led()
{
	static F4GPIO f4gpioC4(GPIOC,GPIO_Pin_4);
	static GPIOLED led_red(&f4gpioC4);
	static F4GPIO f4gpioC5(GPIOC,GPIO_Pin_5);
	static GPIOLED led_green(&f4gpioC5);
	static RGBLED rgb;
	static F4GPIO f4gpioC0(GPIOC,GPIO_Pin_0);	
	static GPIOLED flashlight(&f4gpioC0, true);
	
	
	manager.register_LED("SD",&led_red);
	manager.register_LED("state",&led_green);
	manager.register_LED("flashlight",&flashlight);
	manager.register_RGBLED("rgb", &rgb);
}

//Define TIMER Function:
//Timer1

static F4Timer f4TIM1(TIM1);
static F4Timer f4TIM2(TIM2);
void init_timers()
{
	manager.register_Timer("mainloop", &f4TIM1);
	manager.register_Timer("log", &f4TIM2);
}
extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
	f4TIM1.call_callback();
}

extern "C" void TIM2_IRQHandler(void)
{
	f4TIM2.call_callback();
}
//Timer2


//Define UART Funtion:
#include <HAL\STM32F4\F4UART.h>
#include <HAL\Interface\IUART.h>

//For uart4:
/*
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
*/

//For usart3:
F4UART f4uart3(USART3);
IUART * pUART3 = &f4uart3;
void init_uart3()
{
	pUART3->set_baudrate(115200);
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

//For usart4:
/*
F4UART f4uart4(UART4);
void init_uart4()
{
	f4uart4.set_baudrate(115200);
	//manager.register_UART("UART4", &f4uart4);
}
extern "C" void UART4_IRQHandler(void)
{
	f4uart4.UART4_IRQHandler();
}
extern "C" void DMA1_Stream4_IRQHandler()
{
	f4uart4.DMA1_Steam4_IRQHandler();
}
*/

//For usart1:
F4UART f4uart1(USART1);
IUART * pUART1 = &f4uart1;
void init_uart1()
{
	pUART1->set_baudrate(115200);
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
#include <HAL/sensors/MPU6000.h>
#include <HAL/sensors/HMC5983SPI.h>
#include <HAL/sensors/MS5611_SPI.h>
F4SPI spi1;
F4GPIO cs_mpu(GPIOA, GPIO_Pin_15);
F4GPIO cs_ms5611(GPIOC, GPIO_Pin_2);
F4GPIO cs_hmc5983(GPIOC, GPIO_Pin_3);
sensors::MPU6000 mpu6000device;
sensors::MS5611_SPI ms5611device;
sensors::HMC5983 hmc5983device;
void init_sensors()
{
	spi1.init(SPI1);
	
	cs_mpu.set_mode(MODE_OUT_PushPull);
	cs_ms5611.set_mode(MODE_OUT_PushPull);
	cs_hmc5983.set_mode(MODE_OUT_PushPull);
	cs_mpu.write(true);
	cs_ms5611.write(true);
	cs_hmc5983.write(true);
	
	if (mpu6000device.init(&spi1, &cs_mpu) == 0)
	{
		mpu6000device.accelerometer_axis_config(1, 0, 2, -1, -1, 1);
		mpu6000device.gyro_axis_config(1, 0, 2, +1, +1, -1);
		manager.register_accelerometer(&mpu6000device);
		manager.register_gyroscope(&mpu6000device);
	}

	if (ms5611device.init(&spi1, &cs_ms5611) == 0)
	{
		manager.register_barometer(&ms5611device);
	}

	if (hmc5983device.init(&spi1, &cs_hmc5983) == 0)
	{
		hmc5983device.axis_config(2, 0, 1, +1, +1, -1);
		manager.register_magnetometer(&hmc5983device);
	}
}

int init_external_compass()
{
	F4GPIO SCL(GPIOC, GPIO_Pin_13);
	F4GPIO SDA(GPIOC, GPIO_Pin_14);
	I2C_SW i2c(&SCL, &SDA);
	
	sensors::HMC5983 hmc5983;
	if (hmc5983.init(&i2c) == 0)
	{
		LOGE("found HMC5983 on I2C(PC13,PC14)\n");
		static F4GPIO SCL(GPIOC, GPIO_Pin_13);
		static F4GPIO SDA(GPIOC, GPIO_Pin_14);
		static I2C_SW i2c(&SCL, &SDA);
		static sensors::HMC5983 hmc5983;
		hmc5983.init(&i2c);
		hmc5983.axis_config(0, 2, 1, +1, +1, +1);

		manager.register_magnetometer(&hmc5983);
	}

	return 0;	
}

int init_RC()
{
	static dev_v2::RCIN rcin;
	static dev_v2::RCOUT rcout;
	
	manager.register_RCIN(&rcin);
	manager.register_RCOUT(&rcout);
	
	return 0;
}

int init_GPS()
{
	static sensors::UartUbloxNMEAGPS gps;
	if (gps.init(&f4uart1, 115200) == 0)	
		manager.register_GPS(&gps);
	
	return 0;
}

int init_asyncworker()
{
	static dev_v2::AsyncWorker worker;
	manager.register_asyncworker(&worker);
	
	return 0;
}

//Define BattertVoltage Function:
F4ADC f4adc1_Ch2(ADC1,ADC_Channel_2);
ADCBatteryVoltage battery_voltage(&f4adc1_Ch2, 3.3f/4096*4.0f);
IBatteryVoltage * pBattery_Voltage= &battery_voltage;
void init_BatteryVoltage()
{
	manager.register_BatteryVoltage("BatteryVoltage",pBattery_Voltage);
	manager.getBatteryVoltage("BatteryVoltage")->read();
}
//Define BattertVoltage Function:
F4ADC f4adc1_Ch8(ADC1,ADC_Channel_8);
ADCBatteryVoltage battery_current(&f4adc1_Ch8, 3.3f/4096);
IBatteryVoltage * pBattery_Current= &battery_current;
void init_BatteryCurrent()
{
	manager.register_BatteryVoltage("BatteryCurrent", pBattery_Current);
	//manager.getBatteryVoltage("BatteryVoltage")->read();
}

int init_flow()
{
	F4GPIO SCL(GPIOC, GPIO_Pin_13);
	F4GPIO SDA(GPIOC, GPIO_Pin_14);
	I2C_SW i2c(&SCL, &SDA);
	
	sensors::PX4Flow px4flow;
	px4flow.init(&i2c);
	
	if (px4flow.healthy())
	{
		LOGE("found PX4FLOW on I2C(PC13,PC14)\n");
		static F4GPIO SCL(GPIOC, GPIO_Pin_13);
		static F4GPIO SDA(GPIOC, GPIO_Pin_14);
		static I2C_SW i2c(&SCL, &SDA);
		static sensors::PX4Flow px4flow;
		px4flow.init(&i2c);

		manager.register_flow(&px4flow);
	}

	return 0;
}

F4GPIO tx(GPIOC,GPIO_Pin_1);
F4GPIO level(GPIOA,GPIO_Pin_1);

sensors::Sonar sonar;
extern "C" void EXTI9_5_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line7) == SET)
	{
		sonar.echo();
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
}

int init_sonar()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// C7 as echo
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// EXTI
	EXTI_ClearITPendingBit(EXTI_Line7);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource7);

	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_Init(&EXTI_InitStructure);

	// priority : lowest
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
		
	sonar.init(&tx, &level);
	manager.register_device("sonar", &sonar);
	
	return 0;
}


int init_VCP()
{
	static F4VCP vcp;
	manager.register_UART("VCP", &vcp);
	
	return 0;
}

int bsp_init_all()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	//init_sonar();
	init_led();
	init_BatteryVoltage();
	init_BatteryCurrent();
//	init_uart4();
	init_uart3();
	init_uart2();
	init_VCP();
	init_timers();
	init_uart1();
	init_RC();
	init_sensors();
	//init_external_compass();
	init_asyncworker();
	init_led();
	init_flow();
	init_GPS();

	// parameter config
	param bsp_parameter("BSP", 1);
	if (bsp_parameter)
	{
		// remote
		for(int i=0; i<6; i++)
		{
			char tmp[5]="rcx1";
			tmp[2] = i + '0';
			param center(tmp, 1500);
			center = 1500;
		}
		param("rc51", 2000) = 2000;
		param("rc52", 3000) = 3000;
		param("rc53", 1) = 1;

		// ESC
		param("tmax", 1900) = 1900;
		param("tmin", 1100) = 1100;
		param("idle", 1100) = 1240;

		// alt hold
		param("accP", 0.12) = 0.12;
		param("accI", 0.24) = 0.24;

		// PID
		param("rP1", 0.2f)=0.45f;
		param("rI1", 0.3f)=0.45f;
		param("rD1", 0.005f)=0.01f;
		param("rP2", 0.36f)=0.55f;
		param("rI2", 0.4f)=0.55f;
		param("rD2", 0.01f)=0.01f;
		param("sP1", 4.5f)=4.5f;
		param("sP2", 4.5f)=4.5f;

		param("rP3", 1.2f)=1.2f;
		param("rI3", 0.15f)=0.15f;

		// frame
		param("mat", 1)=1;
	}
	
	return 0;
}

void reset_system()
{
	NVIC_SystemReset();
}