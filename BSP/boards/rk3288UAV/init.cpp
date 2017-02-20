#include <HAL/Resources.h>
#include <HAL/rk32885.1/ATimer.h>
#include <HAL/rk32885.1/AGpio.h>
#include <HAL/rk32885.1/ASPI.h>
#include <HAL/rk32885.1/AUART.h>
#include <HAL/rk32885.1/AIMUFIFO.h>
#include <HAL/rk32885.1/AI2C.h>
#include <HAL//rk32885.1/AUIOTimer.h>

#include <HAL/sensors/UartUbloxNMEAGPS.h>
#include <HAL/sensors/Sonar.h>
#include <HAL/sensors/SBusIn.h>
#include <HAL/sensors/EBusIn.h>
#include <HAL/sensors/PPMIN.h>
#include <HAL/Interface/ILED.h>
#include <HAL/sensors/PX4Flow.h>
#include <HAL/sensors/MPU6000.h>
#include <HAL/sensors/HMC5983SPI.h>
#include <HAL/sensors/MS5611_SPI.h>
#include <HAL/sensors/ads1115.h>
#include <HAL/sensors/ACamera.h>
#include <HAL/sensors/AVideo.h>
#include <HAL/sensors/AFlow.h>
#include <HAL/sensors/ASonar.h>

#include <HAL/Interface/ILED.h>

#include <Protocol/common.h>
#include <stdio.h>
#include <utils/param.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>

#include "ARCOUT.h"

using namespace androidUAV;
using namespace HAL;
using namespace sensors;
using namespace devices;
static const char *spidevice = "/dev/oledEuler_dev";

static const char *gpiodevice = "/dev/luobogpio";

static const char *uart1device = "/dev/ttyS1";

static const char *uart3device = "/dev/ttyS3";

static const char *uart4device = "/dev/ttyS4";

static const char *pwmdevice = "/dev/eulerpwm0";

static const char *i2c2device = "/dev/mpu6050";;

static const char *imufifoPath = "/data/IMU_FIFO";
const char bsp_name[] = "androidUAV";

void reset_system()
{
	printf("system reset requested!!!\n");
	exit(1);
}

ASPI spi1(spidevice);
AGPIO cs_mpu(gpiodevice,230);	//GPIO7_A6
AGPIO cs_ms5611(gpiodevice,228);//GPIO7_A4
AGPIO cs_hmc5983(gpiodevice,237);//GPIO7_B5
AUART uart1(uart1device);
AUART uart3(uart3device);
AUART uart4(uart4device);


ARCOUT rcout(pwmdevice);
AFIFO imufifo(imufifoPath);
AI2C i2c2(i2c2device);

//static GPIOLED led_red(&cs_ms5611);

MPU6000 mpu6000device;
MS5611_SPI ms5611device;
UartUbloxBinaryGPS gpsdevice;
EBusIN ebus;
HMC5983 hmc5983;
ASonar sonar;
AFlow Aflow;

//RK3288Camera51 ACamera;
RK3288Video camera0;
devices::frame_format cameraOpticalParams = {
	.width  = 320,
	.height =240,
};
void init_sensor()
{
	/*if (mpu6000device.init(&i2c2,0x68) == 0)
	{
		mpu6000device.accelerometer_axis_config(1, 0, 2, -1, -1, +1);
		mpu6000device.gyro_axis_config(1, 0, 2, +1, +1, -1);
		manager.register_accelerometer(&mpu6000device);
		manager.register_gyroscope(&mpu6000device);
	}*/
	cs_mpu.set_mode(MODE_OUT_PushPull);
	cs_ms5611.set_mode(MODE_OUT_PushPull);
	cs_hmc5983.set_mode(MODE_OUT_PushPull);
	cs_mpu.write(true);
	cs_ms5611.write(true);
	cs_hmc5983.write(true);
	//spi1.setSpeed(10000000);
	if (mpu6000device.init(&spi1, &cs_mpu) == 0)
	{
		mpu6000device.accelerometer_axis_config(0, 1, 2, +1, -1, +1);
		mpu6000device.gyro_axis_config(0, 1, 2, -1, +1, -1);
		manager.register_accelerometer(&mpu6000device);
		manager.register_gyroscope(&mpu6000device);
		LOG2("androidUAV:Init MPU6000 ok\n");
	}
	
	/*if(hmc5983.init(&i2c2) == 0)
	{
		LOG2("androidUAV:found HMC5983 on I2C2\n");
		hmc5983.axis_config(0, 2, 1, +1, -1, -1);
		manager.register_magnetometer(&hmc5983);
	}*/
	/*if(hmc5983.init(&spi1,&cs_hmc5983) == 0)
	{
		LOG2("androidUAV:found HMC5983 on spi1\n");
		hmc5983.axis_config(0, 2, 1, +1, -1, -1);
		manager.register_magnetometer(&hmc5983);
	}*/
	if(ms5611device.init(&spi1,&cs_ms5611) == 0)
	{
		manager.register_barometer(&ms5611device);
		LOG2("androidUAV:found ms5611device\n");
	}
	//manager.register_LED("state",&led_red);
	
	uart4.uartInit(115200);
	if(sonar.init(&uart4) == 0)
	{
		manager.register_device("sonar",(IRangeFinder *)&sonar);
	}
	if(Aflow.initPIPE(FLOWPIPE,PIP_OPEN) == 0)
	{
#ifndef USERKDRV
		if(camera0.init(VIDEODEV,NULL)==0)
#endif
		{
#ifdef USERKDRV
			if(Aflow.init(NULL) == 0)
#else
			if(Aflow.init(&camera0) == 0)
#endif
			{
				//LOG2("androidUAV:init optical flow base cuccess\n");
				manager.register_flow(&Aflow);
			}
			else
			{
				LOG2("androidUAV:init optical-flow pipe error\n");
			}
		}
		else
		{
			FILE * file = NULL;
			file = fopen("/data/ALOG.txt","a+");
			fprintf(file,"%s\n","Init camra failed\n");
			fclose(file);
		}
	}
	else
	{
		LOG2("androidUAV:init optical flow base error\n");
	}
}
int init_rc()
{
	ebus.init(&uart1);
	manager.register_RCIN(&ebus);
	return 0;
}
int init_GPS()
{
	int ret = -1;
	//uart3.set_baudrate(9600);
	ret = gpsdevice.init(&uart3,115200);
	if(ret < 0)
	{
		LOG2("androidUAV:Warning GPS init failed\n");
	}
	else
	{
		manager.register_GPS(&gpsdevice);
	}
	return 0;
}
int init_RC()
{
	manager.register_RCOUT(&rcout);

	return 0;
}
void init_external_compass()
{
	sensors::HMC5983 hmc5983;
}
int init_imufifo()
{
	manager.register_FIFO("imu_FIFO",&imufifo);
	return 0;
}
int bsp_init_all()
{
	sleep(5);
	// Prio in the range -20  to  19
	setpriority(PRIO_PROCESS,getpid(),19);
	init_sensor();
	init_GPS();
	init_rc();
	init_RC();
	init_imufifo();
	static androidUAV::ATimer timer1(98);
	static androidUAV::ATimer timer2(97);
	//static androidUAV::ATimer timer3(96);
	
	//static androidUAV::AUIOTimer timer1(98);
	//static androidUAV::AUIOTimer timer2(97);
	static androidUAV::AUIOTimer timer3(96);
	manager.register_Timer("mainloop", &timer1);
	manager.register_Timer("log", &timer2);
	manager.register_Timer("imu", &timer3);
	timer1.set_period(0);
	timer2.set_period(0);
	timer3.set_period(0);
	param bsp_parameter("BSP",1);
	if(1)
	{
		param("time", 3000)=3000;
		param("ekf", 0)=2;
		param("err",0) = error_magnet|error_GPS;
		//
		
		param("mat", 1)=1;

		param("rc00", 1008) = 1008;
		param("rc01", 1515) = 1515;
		param("rc02", 1906) = 1906;

		param("rc10", 1008) = 1066;
		param("rc11", 1480) = 1480;
		param("rc12", 1906) = 1999;

		param("rc20", 1008) = 1008;
		param("rc21", 1453) = 1462;
		param("rc22", 1906) = 1931;

		param("rc30", 1008) = 1026;
		param("rc31", 1453) = 1509;
		param("rc32", 1906) = 1953;

		bsp_parameter = 0;
		bsp_parameter.save();
		param::save_all();
	}

	param("tmin", 1100)=1100;
	param("tmax", 1900)=1900;
	param("idle", 1240)=1240;


	return 0;
}
