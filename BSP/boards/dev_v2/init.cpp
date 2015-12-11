#include <stdio.h>
#include <protocol/common.h>
#include <HAL\Resources.h>
#include "RCIN.h"
#include "RCOUT.h"
#include "AsyncWorker.h"
#include <HAL\STM32F4\F4Timer.h>
#include <HAL/sensors/UartUbloxNMEAGPS.h>
#include <HAL\Interface\ILED.h>
#include <HAL/sensors/PX4Flow.h>
#include "RGBLED.h"

using namespace HAL;
using namespace devices;
using namespace STM32F4;
using namespace dev_v2;
using namespace sensors;

__attribute__((section("dma"))) F4UART f4uart1(USART1);
__attribute__((section("dma"))) F4UART f4uart2(USART2);
__attribute__((section("dma"))) F4UART f4uart3(USART3);
__attribute__((section("dma"))) F4UART f4uart4(UART4);

void init_led()
{
	static F4GPIO f4gpioC4(GPIOC,GPIO_Pin_4);
	static GPIOLED led_red(&f4gpioC4);
	static F4GPIO f4gpioC5(GPIOC,GPIO_Pin_5);
	static GPIOLED led_green(&f4gpioC5);
	static RGBLED rgb;
	
	manager.register_LED("SD",&led_red);
	manager.register_LED("state",&led_green);
	manager.register_RGBLED("rgb", &rgb);
}

//Define TIMER Function:
//Timer1

static F4Timer f4TIM1(TIM1);
static F4Timer f4TIM2(TIM2);
static F4Timer f4TIM7(TIM7);
void init_timers()
{
	manager.register_Timer("mainloop", &f4TIM1);
	manager.register_Timer("log", &f4TIM2);
	manager.register_Timer("imu", &f4TIM7);
}
extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
	f4TIM1.call_callback();
}

extern "C" void TIM2_IRQHandler(void)
{
	f4TIM2.call_callback();
}
extern "C" void TIM7_IRQHandler(void)
{
	f4TIM7.call_callback();
}


//Define UART Funtion:
#include <HAL\STM32F4\F4UART.h>
#include <HAL\Interface\IUART.h>

void init_uart()
{
	f4uart1.set_baudrate(115200);
	f4uart2.set_baudrate(115200);
	f4uart3.set_baudrate(115200);
	f4uart4.set_baudrate(115200);
	manager.register_UART("UART1",&f4uart1);
	manager.register_UART("UART2",&f4uart2);
	manager.register_UART("UART3",&f4uart3);
	manager.register_UART("UART4",&f4uart4);
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
		mpu6000device.accelerometer_axis_config(1, 0, 2, -1, -1, +1);
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
		hmc5983device.axis_config(0, 2, 1, +1, -1, -1);
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
	if (gps.init(&f4uart4, 115200) == 0)	
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

int bsp_init_all()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
//	init_led();
	init_BatteryVoltage();
	init_BatteryCurrent();
	init_uart();
	init_timers();
	init_RC();
	init_sensors();
	init_external_compass();
	init_asyncworker();
	init_led();
	init_flow();
	init_GPS();
	
	return 0;
}

void reset_system()
{
	NVIC_SystemReset();
}