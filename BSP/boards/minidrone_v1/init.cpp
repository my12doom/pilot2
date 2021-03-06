#include <stdio.h>
#include <protocol/common.h>
#include <HAL\Resources.h>
#include "RCOUT.h"
#include "AsyncWorker.h"
#include <HAL\STM32F4\F4Timer.h>
#include <HAL\STM32F4\F4VCP.h>
#include <HAL\STM32F4\F4Interrupt.h>
#include <HAL/sensors/UartUbloxNMEAGPS.h>
#include <HAL/sensors/Sonar.h>
#include <HAL/sensors/EBusIn.h>
#include <HAL\Interface\ILED.h>
#include <HAL/sensors/PX4Flow.h>
#include <utils/param.h>
#include <HAL\STM32F4\F4UART.h>
#include <HAL\STM32F4\F4SPI.h>
#include <HAL/sensors/MPU6000.h>
#include <HAL/sensors/HMC5983SPI.h>
#include <HAL/sensors/MS5611_SPI.h>
#include "RGBLED.h"

extern "C" const char bsp_name[] = "mini_drone";

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

//	static GPIOLED led_red(&f4gpioC4);
//	static F4GPIO f4gpioC5(GPIOC,GPIO_Pin_5);
//	static GPIOLED led_green(&f4gpioC5);
	static RGBLED rgb;
	manager.register_RGBLED("rgb", &rgb);

}

//Define TIMER Function:
//Timer1

static F4Timer f4TIM4(TIM4);
static F4Timer f4TIM2(TIM2);
static F4Timer f4TIM7(TIM7);
void init_timers()
{
	manager.register_Timer("mainloop", &f4TIM4);
	manager.register_Timer("log", &f4TIM2);
	manager.register_Timer("imu", &f4TIM7);
}
//extern "C" void TIM1_UP_TIM10_IRQHandler(void)
extern "C" void TIM4_IRQHandler(void)
{
	f4TIM4.call_callback();
}

extern "C" void TIM2_IRQHandler(void)
{
	f4TIM2.call_callback();
}
extern "C" void TIM7_IRQHandler(void)
{
	f4TIM7.call_callback();
}
//Timer2



void init_uart()
{
	f4uart1.set_baudrate(115200);
	f4uart2.set_baudrate(115200);
	f4uart3.set_baudrate(38400);
	f4uart4.set_baudrate(57600);
	manager.register_UART("UART1",&f4uart1);
	manager.register_UART("UART2",&f4uart2);
	//manager.register_UART("power",&f4uart3);	// power telmetry for stupid joystick
	manager.register_UART("Wifi",&f4uart4);
	
	while(0)
	{
		f4uart4.write("hello", 5);
		systimer->delayms(500);
	}
		
}


F4SPI spi1;
F4GPIO cs_mpu(GPIOC, GPIO_Pin_2);
F4GPIO cs_ms5611(GPIOC, GPIO_Pin_1);
sensors::MPU6000 mpu6000device;
sensors::MS5611_SPI ms5611device;
void init_sensors()
{
	spi1.init(SPI1);
	
	cs_mpu.set_mode(MODE_OUT_PushPull);
	cs_ms5611.set_mode(MODE_OUT_PushPull);
	cs_mpu.write(true);
	cs_ms5611.write(true);
	
	if (mpu6000device.init(&spi1, &cs_mpu) == 0)
	{
		mpu6000device.accelerometer_axis_config(0, 1, 2, +1, -1, +1);
		mpu6000device.gyro_axis_config(0, 1, 2, -1, +1, -1);
		manager.register_accelerometer(&mpu6000device);
		manager.register_gyroscope(&mpu6000device);
	}

	if (ms5611device.init(&spi1, &cs_ms5611) == 0)
	{
		manager.register_barometer(&ms5611device);
	}
}

int init_external_compass()
{
	F4GPIO SCL(GPIOB, GPIO_Pin_14);
	F4GPIO SDA(GPIOB, GPIO_Pin_15);
	I2C_SW i2c(&SCL, &SDA);
	
	sensors::HMC5983 hmc5983;	
	if (hmc5983.init(&i2c) == 0)
	{
		LOGE("found HMC5983 on I2C(PB14,PB15)\n");
		static F4GPIO SCL(GPIOB, GPIO_Pin_14);
		static F4GPIO SDA(GPIOB, GPIO_Pin_15);
		static I2C_SW i2c(&SCL, &SDA);
		static sensors::HMC5983 hmc5983;
		hmc5983.init(&i2c);
		hmc5983.axis_config(0, 2, 1, +1, +1, +1);

		manager.register_magnetometer(&hmc5983);
	}

	return 0;	
}

	EBusIN ebus;
int init_RC()
{
	ebus.init(&f4uart2);
	manager.register_RCIN(&ebus);

	static dev_v2::RCOUT rcout;	
	manager.register_RCOUT(&rcout);
	
	return 0;
}

int init_GPS()
{
	static sensors::UartUbloxBinaryGPS gps;
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

void init_BatteryMonitor()
{
//	static F4ADC ref25(ADC1,ADC_Channel_4);
//	static F4ADC f4adc1_Ch8(ADC1,ADC_Channel_8);	
//	static F4ADC f4adc1_Ch2(ADC1,ADC_Channel_2);
//	static F4ADC vcc5v_half_adc(ADC1,ADC_Channel_3);
//	
//	static ADCBatteryVoltage battery_voltage(&f4adc1_Ch2, 3.3f/4095*(150.0f+51.0f)/51.0f, &ref25, 2.495f/3.3f*4095);
//	static ADCBatteryVoltage vcc5v(&vcc5v_half_adc, 3.3f*2/4095, &ref25, 2.495f/3.3f*4095);
//	static ADCBatteryVoltage vcc5v_half(&vcc5v_half_adc, 3.3f/4095, &ref25, 2.495f/3.3f*4095);
//	static ADCBatteryVoltage current_ad(&f4adc1_Ch8, 3.3f/4095);
//	static differential_monitor battery_current(&vcc5v_half, &current_ad, 1/0.066f);
//	static ADCReference vcc(&ref25, 2.495f, 4095);

//	manager.register_BatteryVoltage("BatteryVoltage",&battery_voltage);
//	manager.register_BatteryVoltage("BatteryCurrent", &battery_current);
//	manager.register_BatteryVoltage("5V", &vcc5v);
//	manager.register_BatteryVoltage("vcc", &vcc);
}

int init_flow()
{
	F4GPIO SCL(GPIOB, GPIO_Pin_14);
	F4GPIO SDA(GPIOB, GPIO_Pin_15);
	I2C_SW i2c(&SCL, &SDA);
	
	sensors::PX4Flow px4flow;
	px4flow.init(&i2c);
	
	if (px4flow.healthy())
	{
		LOGE("found Optical Flow on I2C(PB14,PB15)\n");
		static F4GPIO SCL(GPIOB, GPIO_Pin_14);
		static F4GPIO SDA(GPIOB, GPIO_Pin_15);
		static I2C_SW i2c(&SCL, &SDA);
		static sensors::PX4Flow px4flow;
		px4flow.init(&i2c);

		manager.register_flow(&px4flow);
	}

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
	init_led();
	//init_sonar();
	
	init_BatteryMonitor();
	init_uart();
	init_VCP();
	init_timers();
	init_RC();
	init_sensors();
	//init_external_compass();
	init_asyncworker();
	//init_flow();
	//init_GPS();

	// parameter config
	param bsp_parameter("BSP", 1);
	if (bsp_parameter)
	{
		// alt hold
		param("accP", 0.12) = 0.12;
		param("accI", 0.24) = 0.24;
		param("altP", 1) = 1.5;

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
		param("ekf", 0)=0;
		param("time", 3000)=3000;
		
		//ignore some error
		param("err",0)=error_magnet|error_GPS;
		
	}
	
	return 0;
}

void reset_system()
{
	NVIC_SystemReset();
}