#include <stdio.h>
#include <protocol/common.h>
#include <HAL\Resources.h>
#include "RCOUT.h"
#include "AsyncWorker.h"
#include <HAL\STM32F4\F4Timer.h>
#include <HAL\STM32F4\F4VCP.h>
#include <HAL\STM32F4\F4Interrupt.h>
#include <HAL\STM32F4\F4GPIO.h>
#include <HAL/sensors/UartUbloxNMEAGPS.h>
#include <HAL/sensors/Sonar.h>
#include <HAL/sensors/SBusIn.h>
#include <HAL/sensors/EBusIn.h>
#include <HAL/sensors/PPMIn.h>
#include <HAL/sensors/NRF_RC.h>
#include <HAL\Interface\ILED.h>
#include <HAL/sensors/PX4Flow.h>
#include <utils/param.h>
#include <HAL\STM32F4\F4UART.h>
#include <HAL\STM32F4\F4ADC.h>
#include <HAL\STM32F4\F4SPI.h>
#include <HAL/sensors/MPU6000.h>
#include <HAL/sensors/BMI160.h>
#include <HAL/sensors/HMC5983SPI.h>
#include <HAL/sensors/MS5611_SPI.h>
#include <HAL/sensors/ADS1115.h>
#include <bootloader/bootloader_bin.h>
#include "RGBLED.h"
#include <Protocol/crc32.h>
#include <string.h>

extern "C" const char bsp_name[] = "zz1504fz";

using namespace HAL;
using namespace devices;
using namespace STM32F4;
using namespace dev_v2;
using namespace sensors;

__attribute__((section("dma"))) F4UART f4uart1(USART1);
__attribute__((section("dma"))) F4UART f4uart2(USART2);
__attribute__((section("dma"))) F4UART f4uart4(UART4);
__attribute__((section("dma"))) F4UART f4uart3(USART3);
__attribute__((section("dma"))) F4UART sbus_uart(USART6);

void init_led()
{
	static RGBLED rgb;
	manager.register_RGBLED("rgb", &rgb);
}

//Define TIMER Function:
//Timer1

static F4Timer f4TIM4(TIM4);
static F4Timer f4TIM6(TIM6);
static F4Timer f4TIM7(TIM7);
void init_timers()
{
	f4TIM4.set_priority(4, 0);
	manager.register_Timer("mainloop", &f4TIM4);
	manager.register_Timer("log", &f4TIM6);
	manager.register_Timer("imu", &f4TIM7);
}
extern "C" void TIM4_IRQHandler(void)
{
	f4TIM4.call_callback();
}

extern "C" void TIM6_DAC_IRQHandler(void)
{
	f4TIM6.call_callback();
}
extern "C" void TIM7_IRQHandler(void)
{
	f4TIM7.call_callback();
}

void init_uart()
{
	f4uart1.set_baudrate(115200);
	f4uart2.set_baudrate(115200);
	f4uart4.set_baudrate(115200);
	manager.register_UART("UART1",&f4uart1);
	manager.register_UART("UART2",&f4uart2);
}


F4SPI spi1;
F4SPI spi_fram;
F4GPIO cs_mpu(GPIOC, GPIO_Pin_15);
F4GPIO cs_ms5611(GPIOD, GPIO_Pin_7);
F4GPIO cs_bmi(GPIOC, GPIO_Pin_2);
F4GPIO cs_fram(GPIOD, GPIO_Pin_10);
sensors::MPU6000 mpu6000device;
sensors::MS5611_SPI ms5611device;
sensors::HMC5983 hmc5983device;
sensors::BMI160 bmi160device;
void init_sensors()
{
	spi1.init(SPI1);
	spi_fram.init(SPI2);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);
	
	cs_mpu.set_mode(MODE_OUT_PushPull);
	cs_ms5611.set_mode(MODE_OUT_PushPull);
	cs_fram.set_mode(MODE_OUT_PushPull);
	cs_bmi.set_mode(MODE_OUT_PushPull);
	cs_mpu.write(true);
	cs_ms5611.write(true);
	cs_bmi.write(true);
	cs_fram.write(true);
	
	systimer->delayms(50);

	/*	
	if (mpu6000device.init(&spi1, &cs_mpu) == 0)
	{
		mpu6000device.accelerometer_axis_config(0, 1, 2, +1, -1, +1);
		mpu6000device.gyro_axis_config(0, 1, 2, -1, +1, -1);
		manager.register_accelerometer(&mpu6000device);
		manager.register_gyroscope(&mpu6000device);
	}
	*/
	if (bmi160device.init(&spi1, &cs_bmi) == 0)
	{
		bmi160device.accelerometer_axis_config(0, 1, 2, +1, -1, +1);
		bmi160device.gyro_axis_config(0, 1, 2, -1, +1, -1);
		bmi160device.accelerometer_axis_config(0, 1, 2, -1, +1, +1);
		bmi160device.gyro_axis_config(0, 1, 2, +1, -1, -1);

		manager.register_accelerometer(&bmi160device);
		manager.register_gyroscope(&bmi160device);
	}
	else
	{
		//reset_system();
	}
	
	if (ms5611device.init(&spi_fram, &cs_ms5611) == 0)
	{
		manager.register_barometer(&ms5611device);
	}
}

	static SBusIN sbus;
int init_RC()
{
	// SBUS
	F4GPIO sbus_inv(GPIOC, GPIO_Pin_13);
	sbus_inv.set_mode(MODE_OUT_PushPull);
	sbus_inv.write(false);
	
	sbus.init(&sbus_uart);
	manager.register_RCIN(&sbus);		// PC7

	static dev_v2::RCOUT rcout;	
	manager.register_RCOUT(&rcout);
	
	return 0;
}

int init_GPS()
{
	static sensors::UartUbloxBinaryGPS gps;
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


void init_BatteryMonitor()
{
	static F4ADC f4adc1_Ch2(ADC1,ADC_Channel_2);
	static F4ADC f4adc1_Ch3(ADC1,ADC_Channel_3);
	static ADCBatteryVoltage battery_voltage(&f4adc1_Ch2, 3.3f/4095*(100.0f+6.8f)/6.8f);
	static ADCBatteryVoltage battery_current(&f4adc1_Ch3, 3.3f/4095 * 9.8f);
	
	manager.register_BatteryVoltage("BatteryVoltage",&battery_voltage);
	manager.register_BatteryVoltage("BatteryCurrent",&battery_current);
}

int init_VCP()
{
	static F4VCP vcp;
	manager.register_UART("VCP", &vcp);
	
	return 0;
}

void motor_test()
{
	HAL::IRCOUT *rcout = manager.get_RCOUT();

	int16_t g_ppm_output[4];
	for(int i=0; i<4; i++)
		g_ppm_output[i] = 1000;
	if (rcout)rcout->write(g_ppm_output, 0,  countof(g_ppm_output));

	systimer->delayms(500);
	
	//for(int i=0; i<3; i++)
		g_ppm_output[3] = 1100;
	if (rcout)rcout->write(g_ppm_output, 0,  countof(g_ppm_output));
	systimer->delayms(2000);

}


int bsp_init_all()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	init_BatteryMonitor();
	init_uart();
	init_VCP();
	init_timers();
	init_sensors();
	//init_external_compass();
	init_asyncworker();
	init_led();
	//init_flow();
	init_GPS();
	
	init_RC();
	//motor_test();
	
	init_led();
	param("tmin", 1100) = 1000;
	param("mat", 1)=3;
	param("err",0)= error_magnet | error_GPS;
	param("ekf", 0)=0;
	param("time", 3000)=2000;
	
	param("rP1", 0.2f)=0.1f;
	param("rI1", 0.3f)=0.3f;
	param("rD1", 0.005f)=0.002f;
	param("sP1", 8)=4.0f;

	param("rP2", 0.2)=0.1f;
	param("rI2", 0.3)=0.3f;
	param("rD2", 0.005f)=0.002f;
	param("sP2", 8)=4.0f;

	//
	param("rP1", 0.2f)=0;
	param("rI1", 0.3f)=0;
	param("rD1", 0.005f)=0;
	param("sP1", 8)=4.0f;

	param("rP2", 0.2)=0;
	param("rI2", 0.3)=0;
	param("rD2", 0.005f)=0;
	param("sP2", 8)=4.0f;
	

	param("rP3", 1.75f)=0.4;
	param("rI3", 0.25f)=0.25;
	param("rD3", 0.01f)=0;
	//param("sP3", 8)=4.0f;

	param("idle", 1100) = 1070;
	param("hov", 0.35) = 0.25;

	// parameter config
	param is_booting("boot", 1);
	if (is_booting)
	{
		
		// ESC
		param("tmax", 1900) = 1900;
		param("tmin", 1100) = 1100;
		param("idle", 1100) = 1240;

		// alt hold
		param("accP", 0.12) = 0.12;
		param("accI", 0.24) = 0.24;
		param("altP", 1) = 1.5;

		// PID
		param("rP1", 0.2f)=0.60f;
		param("rI1", 0.3f)=0.80f;
		param("rD1", 0.005f)=0.020f;
		param("rP2", 0.36f)=0.80f;
		param("rI2", 0.4f)=1.20f;
		param("rD2", 0.01f)=0.025f;
		param("sP1", 4.5f)=4.5f;
		param("sP2", 4.5f)=4.5f;

		param("rP3", 1.2f)=1.2f;
		param("rI3", 0.15f)=0.15f;
		
		param("triP", 0)=0;
		
		// tilt angle
		param("rngR", PI / 8)= PI / 8;
		param("rngP", PI / 8)= PI / 8;

		// frame
		param("mat", 1)=2;
		param("ekf", 0)=2;
		
		is_booting = 0;
		is_booting.save();
		
		param::save_all();
	}

	return 0;
}

void reset_system()
{
	NVIC_SystemReset();
}