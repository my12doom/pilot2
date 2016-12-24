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
#include <HAL/sensors/PPMIn.h>
#include <HAL/sensors/NRF_RC.h>
#include <HAL\Interface\ILED.h>
#include <HAL/sensors/PX4Flow.h>
#include <utils/param.h>
#include <HAL\STM32F4\F4UART.h>
#include <HAL\STM32F4\F4ADC.h>
#include <HAL\STM32F4\F4SPI.h>
#include <HAL/sensors/MPU6000.h>
#include <HAL/sensors/HMC5983SPI.h>
#include <HAL/sensors/MS5611_SPI.h>
#include <HAL/sensors/ADS1115.h>
#include <bootloader/bootloader_bin.h>
#include "RGBLED.h"

extern "C" const char bsp_name[] = "mo_v3";

using namespace HAL;
using namespace devices;
using namespace STM32F4;
using namespace dev_v2;
using namespace sensors;

__attribute__((section("dma"))) F4UART f4uart1(USART1);
__attribute__((section("dma"))) F4UART f4uart2(USART2);
__attribute__((section("dma"))) F4UART f4uart4(UART4);

void init_led()
{
	static RGBLED rgb;
	static F4GPIO f4gpioB2(GPIOB,GPIO_Pin_2);
	//static GPIOLED flashlight(&f4gpioB2, true);
	
	//manager.register_LED("flashlight",&flashlight);	
	manager.register_RGBLED("rgb", &rgb);
}

//Define TIMER Function:
//Timer1

static F4Timer f4TIM1(TIM1);
static F4Timer f4TIM6(TIM6);
static F4Timer f4TIM7(TIM7);
void init_timers()
{
	manager.register_Timer("mainloop", &f4TIM1);
	manager.register_Timer("log", &f4TIM6);
	manager.register_Timer("imu", &f4TIM7);
}
extern "C" void TIM1_UP_TIM10_IRQHandler(void)
{
	f4TIM1.call_callback();
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
	f4uart4.set_baudrate(38400);
	manager.register_UART("UART1",&f4uart1);
	manager.register_UART("UART2",&f4uart2);
	manager.register_UART("Wifi",&f4uart4);	
}


F4SPI spi1;
F4GPIO cs_mpu(GPIOC, GPIO_Pin_7);
F4GPIO cs_ms5611(GPIOC, GPIO_Pin_6);
F4GPIO cs_hmc5983(GPIOC, GPIO_Pin_13);
sensors::MPU6000 mpu6000device;
sensors::MS5611_SPI ms5611device;
sensors::HMC5983 hmc5983device;
void init_sensors()
{
	spi1.init(SPI1);
	
	cs_mpu.set_mode(MODE_OUT_PushPull);
	cs_ms5611.set_mode(MODE_OUT_PushPull);
	cs_mpu.write(true);
	cs_ms5611.write(true);
	cs_hmc5983.write(true);
	
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

	if (hmc5983device.init(&spi1, &cs_hmc5983) == 0)
	{
		hmc5983device.axis_config(0, 2, 1, +1, -1, -1);
		manager.register_magnetometer(&hmc5983device);
	}
}

int init_external_compass()
{
	F4GPIO SCL(GPIOC, GPIO_Pin_14);
	F4GPIO SDA(GPIOC, GPIO_Pin_15);
	I2C_SW i2c(&SCL, &SDA);
	
	sensors::HMC5983 hmc5983;	
	if (hmc5983.init(&i2c) == 0)
	{
		LOGE("found HMC5983 on I2C(PC14,PC15)\n");
		static F4GPIO SCL(GPIOC, GPIO_Pin_14);
		static F4GPIO SDA(GPIOC, GPIO_Pin_15);
		static I2C_SW i2c(&SCL, &SDA);
		static sensors::HMC5983 hmc5983;
		hmc5983.init(&i2c);
		hmc5983.axis_config(0, 2, 1, +1, +1, +1);

		manager.register_magnetometer(&hmc5983);
	}

	return 0;	
}

static F4Timer tim8(TIM8);
extern "C" void TIM8_UP_TIM13_IRQHandler(void)
{
	tim8.call_callback();
}

int init_RC()
{	
	// NRF reciever
	static NRFIn in;
	static F4SPI spi2(SPI2);
	static F4GPIO cs(GPIOB, GPIO_Pin_12);
	static F4GPIO ce(GPIOA, GPIO_Pin_15);
	static F4Interrupt irq;
	
	irq.init(GPIOA, GPIO_Pin_8, interrupt_falling);
	in.init(&spi2, &cs, &ce, &irq, &tim8);
	manager.register_RCIN(&in);
	
	
	// decrease irq priority manually, to prevent interrupting usart/sonar
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//while(1)
	{
		int16_t v[6];
		in.get_channel_data(v, 0, 6);
	}
	

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
	static F4ADC ref25(ADC1,ADC_Channel_4);
	static F4ADC f4adc1_Ch2(ADC1,ADC_Channel_2);
	static F4ADC f4adc1_Ch3(ADC1,ADC_Channel_3);
	static ADCBatteryVoltage battery_voltage(&f4adc1_Ch2, 3.3f/4095*(150.0f+51.0f)/51.0f, &ref25, 2.495f/3.3f*4095);
	static ADCBatteryVoltage battery_current(&f4adc1_Ch3, 3.3f/4095 * 20);
	
	manager.register_BatteryVoltage("BatteryVoltage",&battery_voltage);
	manager.register_BatteryVoltage("BatteryCurrent",&battery_current);
}

int init_flow()
{
	F4GPIO SCL(GPIOC, GPIO_Pin_14);
	F4GPIO SDA(GPIOC, GPIO_Pin_15);
	I2C_SW i2c(&SCL, &SDA);
	
	sensors::PX4Flow px4flow;
	px4flow.init(&i2c);
	
	if (px4flow.healthy())
	{
		LOGE("found Optical Flow on I2C(PC14,PC15)\n");
		static F4GPIO SCL(GPIOC, GPIO_Pin_14);
		static F4GPIO SDA(GPIOC, GPIO_Pin_15);
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

static F4GPIO PON(GPIOC,GPIO_Pin_0);
static F4GPIO POUT(GPIOB,GPIO_Pin_6);
static F4GPIO button(GPIOC,GPIO_Pin_1);
static bool button_up_foound = false;

void power_button_worker(int p)
{
	int64_t t = systimer->gettime();
	
	while(button.read())
	{
		float v = 1 - (systimer->gettime() - t)/1500000.0f;
		if (manager.getRGBLED("rgb"))
			manager.getRGBLED("rgb")->write(v,0,0);
		
		if (systimer->gettime() - t > 1500000)
		{
			LOGE("shutting down from power button");
			systimer->delayms(100);
			PON.write(false);
			POUT.write(false);
			
			manager.getTimer("mainloop")->set_callback(NULL);
			manager.getTimer("log")->set_callback(NULL);
			manager.getTimer("imu")->set_callback(NULL);
			if (manager.getRGBLED("rgb"))
				manager.getRGBLED("rgb")->write(0,0,0);
			
			while(1)
				;
		}
	}
}


static void button_entry(void *parameter, int flags)
{
	printf("power button\n");
	
	if (button.read())
		if (button_up_foound)
			manager.get_asyncworker()->add_work(power_button_worker, 0);
	else
		button_up_foound = true;
}



int out_power()
{
	PON.set_mode(MODE_OUT_PushPull);
	POUT.set_mode(MODE_OUT_PushPull);
	button.set_mode(MODE_IN);
	
	IBatteryVoltage * batt = manager.getBatteryVoltage("BatteryVoltage");
	
	float v = 0;
	while(v < 1)
	{
		if (button.read() || !batt || batt->read() < 6.0f)
			v += 0.001f;
		else
			v -= 0.0005f;
		
		if (v<0)
			v = 0;
		
		PON.write(v>=0.001f);
		systimer->delayms(1);
		manager.getRGBLED("rgb")->write(v,0,0);
	}
		
	static F4Interrupt interrupt;
	interrupt.init(GPIOC, GPIO_Pin_1, interrupt_rising_or_falling);
	POUT.write(true);
		
	interrupt.set_callback(button_entry, NULL);

	manager.getRGBLED("rgb")->write(0,1,0);
	systimer->delayms(500);
	
	return 0;
}

int init_sonar()
{
	static F4GPIO tx(GPIOC,GPIO_Pin_4);
	static F4GPIO gain(GPIOB,GPIO_Pin_2);

	static F4Interrupt echo;
	echo.init(GPIOC, GPIO_Pin_5, interrupt_rising_or_falling);

	static sensors::Sonar sonar;
	sonar.init(&tx, &echo, NULL, &gain);
	manager.register_device("sonar", &sonar);
}

static param ignore_error("err",0);
int bsp_init_all()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	//init_sonar();
	init_led();
	init_BatteryMonitor();
	init_uart();
	init_VCP();
	init_timers();
	init_RC();
	init_sensors();
	init_external_compass();
	init_asyncworker();
	init_led();
	init_flow();
	out_power();
	param("err",0)=error_GPS;
	param("idle",1140)=1240;
	if (! (int(ignore_error) & error_GPS))
		init_GPS();
	
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
		param("time", 3000)=3000;
		
		is_booting = 0;
		is_booting.save();
		
		param::save_all();
	}
	
	/*
	HAL::IStorage * bl_storage = get_bootloader_storage();
	
	int64_t t = systimer->gettime();
	bl_storage->erase(0);
	int erase_time = systimer->gettime() - t;
	
	t = systimer->gettime();
	bl_storage->write(0, bootloader_data, sizeof(bootloader_data));
	int program_time = systimer->gettime() - t;
	
	printf("erase:%d, program:%d\n", erase_time, program_time);
	*/
	
	return 0;
}

void reset_system()
{
	NVIC_SystemReset();
}