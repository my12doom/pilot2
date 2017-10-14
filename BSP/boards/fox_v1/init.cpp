#include <stdio.h>
#include <protocol/common.h>
#include <HAL\Resources.h>
#include "RCOUT.h"
#include "AsyncWorker.h"
#include <HAL\STM32F4\F4Timer.h>
#include <HAL\STM32F4\F4VCP.h>
#include <HAL\STM32F4\F4Interrupt.h>
#include <HAL\STM32F4\F4GPIO.h>
#include <HAL\STM32F4\F4ADC.h>
#include <HAL/sensors/UartUbloxNMEAGPS.h>
#include <HAL/sensors/Sonar.h>
#include <HAL/sensors/PPMIN.h>
#include <HAL\Interface\ILED.h>
#include <HAL/sensors/FoxFlow.h>
#include <HAL/sensors/VL53L0X.h>
#include <utils/param.h>
#include <HAL\STM32F4\F4UART.h>
#include <HAL/sensors/IST8307A.h>

extern "C" const char bsp_name[] = "v4";

using namespace HAL;
using namespace devices;
using namespace STM32F4;
using namespace dev_v2;
using namespace sensors;

// timers
static F4Timer f4TIM3(TIM3);
static F4Timer f4TIM2(TIM2);
static F4Timer f4TIM7(TIM7);
void init_timers()
{
	manager.register_Timer("mainloop", &f4TIM3);
	manager.register_Timer("log", &f4TIM2);
	manager.register_Timer("imu", &f4TIM7);
}
extern "C" void TIM3_IRQHandler(void)
{
	f4TIM3.call_callback();
}

extern "C" void TIM2_IRQHandler(void)
{
	f4TIM2.call_callback();
}

extern "C" void TIM7_IRQHandler(void)
{
	f4TIM7.call_callback();
}

#include <HAL/sensors/MPU6000.h>
#include <HAL/sensors/SPL06.h>

F4GPIO SCL_sensors(GPIOB, GPIO_Pin_6);
F4GPIO SDA_sensors(GPIOB, GPIO_Pin_7);
I2C_SW i2c_sensors(&SCL_sensors, &SDA_sensors);
sensors::MPU6000 mpu6000device;
sensors::SPL06 spl06device;

void init_sensors()
{
	if (mpu6000device.init(&i2c_sensors, 0xD0) == 0)
	{
		mpu6000device.accelerometer_axis_config(1, 0, 2, -1, -1, +1);
		mpu6000device.gyro_axis_config(1, 0, 2, +1, +1, -1);
		manager.register_accelerometer(&mpu6000device);
		manager.register_gyroscope(&mpu6000device);
	}
	if (spl06device.init(&i2c_sensors, 0xEE) == 0)
	{
		manager.register_barometer(&spl06device);
	}
}

int init_RC()
{
	static F4Interrupt interrupt;
	interrupt.init(GPIOA, GPIO_Pin_1, interrupt_rising);
	static PPMIN rcin;
	rcin.init(&interrupt);
	manager.register_RCIN(&rcin);

	static dev_v2::RCOUT rcout;	
	manager.register_RCOUT(&rcout);
	
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
	static F4ADC f4adc1_Ch8(ADC1,ADC_Channel_8);	
	static F4ADC f4adc1_Ch2(ADC1,ADC_Channel_2);
	static F4ADC vcc5v_half_adc(ADC1,ADC_Channel_3);
	
	static ADCBatteryVoltage battery_voltage(&f4adc1_Ch2, 3.3f/4095*(150.0f+50.0f)/51.0f);
	static ADCBatteryVoltage vcc5v(&vcc5v_half_adc, 3.3f*2/4095);
	static ADCBatteryVoltage vcc5v_half(&vcc5v_half_adc, 3.3f/4095);
	static ADCBatteryVoltage current_ad(&f4adc1_Ch8, 3.3f/4095);
	static differential_monitor battery_current(&vcc5v_half, &current_ad, 1/0.066f);

	manager.register_BatteryVoltage("BatteryVoltage",&battery_voltage);
	manager.register_BatteryVoltage("BatteryCurrent", &battery_current);
	manager.register_BatteryVoltage("5V", &vcc5v);
}

__attribute__((section("dma"))) F4UART f4uart2(USART2);
FoxFlow1 flow;

int init_flow()
{
	flow.init(&f4uart2, 0.2, 0.2);
	manager.register_flow(&flow);
	
	return 0;
}

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14
#define VL53L0X_Add (0x29<<1)
#define VL53L0X_WHO_AM_I							0xC0   // should be 0x40

uint8_t readReg(HAL::II2C* i2c, uint8_t reg)
{
	uint8_t value = 0;
	
	i2c->read_reg(VL53L0X_Add, reg, &value);
	
	return value;	
}

int init_tof()
{	
	static F4GPIO scl(GPIOB, GPIO_Pin_6);
	static F4GPIO sda(GPIOB, GPIO_Pin_7);
	static I2C_SW i2c(&scl, &sda);	
	
	static VL53L0X tof;
	if (tof.init(&i2c) == 0)
		manager.register_device("sonar", &tof);
	
	float v;
	while(0)
		tof.read(&v);
	
	return 0;	
}

int bsp_init_all()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
		
	//init_BatteryMonitor();
	init_timers();
	init_tof();
	init_RC();
	init_sensors();
	init_asyncworker();
	init_flow();
	param("err",0)=error_GPS | error_magnet;
	
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
		param("rc50", 2000) = 1000;
		param("rc51", 2000) = 2000;
		param("rc52", 3000) = 3000;
		param("rc53", 1) = 1;

		// ESC
		param("tmax", 1900) = 1900;
		param("tmin", 1100) = 900;
		param("idle", 1100) = 1030;

		// alt hold
		param("accP", 0.12) = 0.12;
		param("accI", 0.24) = 0.24;
		param("altP", 1) = 1.5;

		// PID
		float f = 1.0f;
		param("rP1", 0.2f)=0.60f * f;
		param("rI1", 0.3f)=0.80f * f;
		param("rD1", 0.005f)=0.020f * f;
		param("rP2", 0.36f)=0.80f * f;
		param("rI2", 0.4f)=1.20f* f;
		param("rD2", 0.01f)=0.025f* f;
		param("sP1", 4.5f)=6.5f;
		param("sP2", 4.5f)=6.5f;

		param("rP3", 1.2f)=2.4f;
		param("rI3", 0.15f)=0.25f;
		param("rD3", 0.01)=0.02f;

		// frame
		param("mat", 1)=3;
		param("ekf", 1)=2;
		param("time", 3000)=5000;
		
		// test
		param("limV", 50) = 50;
	}
	
	return 0;
}

void reset_system()
{
	NVIC_SystemReset();
}