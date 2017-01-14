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
#include <HAL/sensors/SBusIn.h>
#include <HAL\Interface\ILED.h>
#include <HAL/sensors/PX4Flow.h>
#include <HAL/sensors/BM1383.h>
#include <HAL/sensors/SPL06.h>
#include <utils/param.h>
#include "RGBLED.h"
#include "TLC59208F.h"
#include <HAL\STM32F4\F4UART.h>

extern "C" const char bsp_name[] = "novatek_v1";

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
	static F4GPIO f4gpioB14(GPIOB,GPIO_Pin_14);	
	static F4GPIO f4gpioB15(GPIOB,GPIO_Pin_15);
	f4gpioB15.set_mode(HAL::MODE_OUT_PushPull);
	f4gpioB15.write(true);
	static GPIOLED flashlight(&f4gpioB14, true);
	
	flashlight.on();
	
	manager.register_LED("SD",&led_red);
	manager.register_LED("state",&led_green);
	manager.register_LED("flashlight",&flashlight);
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


#include <HAL\STM32F4\F4SPI.h>
#include <HAL/sensors/MPU6000.h>
#include <HAL/sensors/HMC5983SPI.h>
#include <HAL/sensors/MS5611_SPI.h>
#include <HAL/sensors/IST8307A.h>
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

static F4GPIO SCL2(GPIOC, GPIO_Pin_0);
static F4GPIO SDA2(GPIOC, GPIO_Pin_1);
static I2C_SW i2c2(&SCL2, &SDA2);
static sensors::IST8307A ist2;
int init_ist()
{
	if (manager.get_magnetometer_count())
	{
		LOGE("already have 1 magnetomter\n");
		return -1;
	}
	
	F4GPIO SCL1(GPIOC, GPIO_Pin_0);
	F4GPIO SDA1(GPIOC, GPIO_Pin_1);
	I2C_SW i2c1(&SCL1, &SDA1);

	sensors::IST8307A ist;
	if (ist.init(&i2c1) == 0)
	{
		LOGE("found IST8307A on I2C(PC13,PC14)\n");
		ist2.init(&i2c2);
		ist2.axis_config(1, 0, 2, -1, +1, -1);

		manager.register_magnetometer(&ist2);
	}

	return 0;
}

int init_RC()
{
	static F4Interrupt interrupt;
	interrupt.init(GPIOB, GPIO_Pin_12, interrupt_rising);
	static PPMIN rcin;
	rcin.init(&interrupt);
	manager.register_RCIN(&rcin);

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
	static F4ADC f4adc1_Ch8(ADC1,ADC_Channel_8);	
	static F4ADC f4adc1_Ch2(ADC1,ADC_Channel_2);
	static F4ADC vcc5v_half_adc(ADC1,ADC_Channel_3);
	
	static ADCBatteryVoltage battery_voltage(&f4adc1_Ch2, 3.3f/4095*(150.0f+51.0f)/51.0f, &ref25, 2.495f/3.3f*4095);
	static ADCBatteryVoltage vcc5v(&vcc5v_half_adc, 3.3f*2/4095, &ref25, 2.495f/3.3f*4095);
	static ADCBatteryVoltage vcc5v_half(&vcc5v_half_adc, 3.3f/4095, &ref25, 2.495f/3.3f*4095);
	static ADCBatteryVoltage current_ad(&f4adc1_Ch8, 3.3f/4095);
	static differential_monitor battery_current(&vcc5v_half, &current_ad, 1/0.066f);
	static ADCReference vcc(&ref25, 2.495f, 4095);

	manager.register_BatteryVoltage("BatteryVoltage",&battery_voltage);
	manager.register_BatteryVoltage("BatteryCurrent", &battery_current);
	manager.register_BatteryVoltage("5V", &vcc5v);
	manager.register_BatteryVoltage("vcc", &vcc);
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
		LOGE("found Optical Flow on I2C(PC13,PC14)\n");
		static F4GPIO SCL(GPIOC, GPIO_Pin_13);
		static F4GPIO SDA(GPIOC, GPIO_Pin_14);
		static I2C_SW i2c(&SCL, &SDA);
		static sensors::PX4Flow px4flow;
		px4flow.init(&i2c);

		manager.register_flow(&px4flow);
		manager.register_device("sonar", (IRangeFinder*)&px4flow);
	}

	return 0;
}

int init_VCP()
{
	static F4VCP vcp;
	manager.register_UART("VCP", &vcp);
	
	return 0;
}

int iTLC59208F()
{	
	static TLC59208F f;
	uint8_t channel_map[8] = {0,1,2,0,1,2,-1,-1};
	if (f.init(&i2c2, 0x40, channel_map) == 0)
		manager.register_RGBLED("eye", &f);
	
	return 0;
}

int init_1383()
{
	static SPL06 bm1383;
	static F4GPIO SCL(GPIOC, GPIO_Pin_13);
	static F4GPIO SDA(GPIOC, GPIO_Pin_14);
	static I2C_SW i2c(&SCL, &SDA);

	for(int i=0; i<5; i++)
	{
		systimer->delayms(50);
		if (bm1383.init(&i2c) == 0)
		{
			manager.register_barometer(&bm1383);
			
			LOGE("found SPL06\n");
			
			return 0;
		}
	}
	
	return -1;
}

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
	init_1383();
	//init_external_compass();
	init_ist();
	init_asyncworker();
	init_led();
	init_GPS();
	init_flow();
	iTLC59208F();
	
	// parameter config
	param is_booting("boot", 1);
	if (is_booting)
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
		param("time", 3000)=5000;
		
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