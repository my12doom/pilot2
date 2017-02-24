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
#include <HAL\Interface\ILED.h>
#include <HAL/sensors/PX4Flow.h>
#include <utils/param.h>
#include <HAL\STM32F4\F4UART.h>
#include <HAL\STM32F4\F4SPI.h>
#include <HAL/sensors/MPU6000.h>
#include <HAL/sensors/HMC5983SPI.h>
#include <HAL/sensors/MS5611_SPI.h>
#include <HAL/sensors/ADS1115.h>
#include <bootloader/bootloader_bin.h>

extern "C" const char bsp_name[] = "mo_v3";

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
	
	manager.register_LED("SD",&led_red);
	manager.register_LED("state",&led_green);
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
	//f4uart3.set_baudrate(38400);
	f4uart4.set_baudrate(57600);
	manager.register_UART("UART1",&f4uart1);
	manager.register_UART("UART2",&f4uart2);
	//manager.register_UART("power",&f4uart3);	// power telmetry for stupid joystick
	manager.register_UART("Wifi",&f4uart4);
	
	while(0)
	{
		f4uart1.write("hello", 5);
		systimer->delayms(500);
		
		char tmp[1000];
		int o = f4uart1.read(tmp, 1000);
		for(int i = 0; i<o; i++)
			fputc(tmp[i], NULL);
	}
		
}


F4SPI spi1;
F4GPIO cs_mpu(GPIOC, GPIO_Pin_7);
F4GPIO cs_ms5611(GPIOC, GPIO_Pin_6);
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
		//mpu6000device.accelerometer_axis_config(1, 0, 2, -1, -1, +1);
		//mpu6000device.gyro_axis_config(1, 0, 2, +1, +1, -1);
		mpu6000device.accelerometer_axis_config(1, 0, 2, -1, +1, -1);		// up side down
		mpu6000device.gyro_axis_config(1, 0, 2, +1, -1, +1);				// up side down
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
		//hmc5983.axis_config(0, 2, 1, +1, +1, +1);							
		hmc5983.axis_config(2, 0, 1, -1, +1, +1);							// rotated magneto

		manager.register_magnetometer(&hmc5983);
	}

	return 0;	
}

int init_RC()
{
	// SBUS
	/*
	static SBusIN sbus;
	sbus.init(&f4uart3);
	manager.register_RCIN(&sbus);
	*/
	
	// PPM
	
	static F4Interrupt interrupt;
	interrupt.init(GPIOB, GPIO_Pin_0, interrupt_rising);
	static PPMIN ppm;
	ppm.init(&interrupt);
	manager.register_RCIN(&ppm);
	

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

class ADS1115BatterySensor :public IBatteryVoltage
{
public:
	ADS1115BatterySensor(ADS1115 *ads, ads1115_channel channel, ads1115_speed speed, float offset, float scale)
	{
		this->offset = offset;
		this->scale = scale;
		this->ads = ads;
		ads->new_work(speed, channel, gain_4V, &data); 
		data = 0;
		n = 0;
	}
	~ADS1115BatterySensor(){}
	virtual float read()
	{
		if (n++ % 10 == 0)
			ads->go_on();
		
		return ( data * 4.096 / 32767 + offset) * scale;
	}

protected:
	int n;
	ADS1115 *ads;
	int16_t data;
	float offset;
	float scale;
};

void init_BatteryMonitor()
{
	F4GPIO SCL(GPIOB, GPIO_Pin_14);
	F4GPIO SDA(GPIOB, GPIO_Pin_15);
	I2C_SW i2c(&SCL, &SDA);
	ADS1115 ads;
	
	if (ads.init(&i2c, 0x90) == 0)
	{
		printf("ads1115 found\n");
		static F4GPIO SCL(GPIOB, GPIO_Pin_14);
		static F4GPIO SDA(GPIOB, GPIO_Pin_15);
		static I2C_SW i2c(&SCL, &SDA);
		static ADS1115 ads;
		ads.init(&i2c, 0x90);
		
		static ADS1115BatterySensor voltage(&ads, channnel_AIN0, speed_860sps, 0, 11);
		static ADS1115BatterySensor current(&ads, channnel_AIN1, speed_860sps, 0, 1.0f/0.001f*1000.0f/110000.0f);
		
		manager.register_BatteryVoltage("BatteryVoltage",&voltage);
		manager.register_BatteryVoltage("BatteryCurrent", &current);
		
	};	
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

static param ignore_error("err",0);
int bsp_init_all()
{
	param("fix", 0) = 1.5;

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
	if (! (int(ignore_error) & error_GPS))
		init_GPS();
	
	// parameter config
	param is_booting("boot", 1);
	if (is_booting)
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
		param("ekf", 0)=2;
		param("time", 3000)=5000;
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