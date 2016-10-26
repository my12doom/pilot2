#include <HAL/Resources.h>
#include <HAL/rk32885.1/ATimer.h>
#include <HAL/rk32885.1/AGpio.h>
#include <HAL/rk32885.1/ASPI.h>
#include <HAL/rk32885.1/AUART.h>
#include <rk3288UAV/ARCOUT.h>
#include <HAL/rk32885.1/AIMUFIFO.h>
#include <HAL/rk32885.1/AI2C.h>

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

#include <Protocol/common.h>
#include <stdio.h>
#include <utils/param.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>

#define	SMPLRT_DIV		0x19
#define	MPU9250_CONFIG	0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define EXT_SENS_DATA	0x49
#define	USER_CTRL		0x6A
#define	PWR_MGMT_1		0x6B
#define	PWR_MGMT_2		0x6C
#define	WHO_AM_I		0x75
using namespace androidUAV;
using namespace HAL;
using namespace sensors;
static const char *spidevice = "/dev/oledEuler_dev";

static const char *gpiodevice = "/dev/luobogpio";

static const char *uart3device = "/dev/ttyS3";

static const char *uart4device = "/dev/ttyS4";

static const char *pwmdevice = "/dev/eulerpwm0";

static const char *i2c2device = "/dev/mpu6050";;

const char bsp_name[] = "androidUAV";

void reset_system()
{
	printf("system reset requested!!!\n");
	exit(1);
}

ASPI spi1(spidevice);
AGPIO cs_mpu(gpiodevice,228);
AGPIO cs_ms5611(gpiodevice,230);
AUART uart3(uart3device);
AUART uart4(uart4device);
ARCOUT rcout(pwmdevice);
AFIFO imufifo;
AI2C i2c2(i2c2device);


MPU6000 mpu6000device;
MS5611_SPI ms5611device;
UartUbloxBinaryGPS gpsdevice;
EBusIN ebus;
typedef struct
{
	float x;
	float y;
	float z;
	float temperature;
}accelerometer_data;

accelerometer_data accData;

int write_reg_i2c(uint8_t reg,uint8_t data)
{
	char dataSend = 0;
	int ret;
	dataSend = data;
	ret = i2c2.write_regs(0x68,reg,(const uint8_t*)&dataSend,1);
	if(ret < 0)
	{
		printf(" init erro \n");
	}
	return ret;
}
void init_sensor()
{
	if (mpu6000device.init(&i2c2,0x68) == 0)
	{
		mpu6000device.accelerometer_axis_config(1, 0, 2, -1, -1, +1);
		mpu6000device.gyro_axis_config(1, 0, 2, +1, +1, -1);
		manager.register_accelerometer(&mpu6000device);
		manager.register_gyroscope(&mpu6000device);
	}
	/*cs_mpu.set_status(1);
	cs_ms5611.set_status(1);
	//spi1.setSpeed(10000000);
	if (mpu6000device.init(&spi1, &cs_mpu) == 0)
	{
		mpu6000device.accelerometer_axis_config(1, 0, 2, -1, -1, +1);
		mpu6000device.gyro_axis_config(1, 0, 2, +1, +1, -1);
		manager.register_accelerometer(&mpu6000device);
		manager.register_gyroscope(&mpu6000device);
	}*/
	if(ms5611device.init(&spi1,&cs_ms5611) == 0)
	{
		manager.register_barometer(&ms5611device);
	}

}
int init_rc()
{
	ebus.init(&uart4);
	manager.register_RCIN(&ebus);
	return 0;
}
int init_GPS()
{
	int ret = -1;
	ret = gpsdevice.init(&uart3,115200);
	manager.register_GPS(&gpsdevice);
	return 0;
}
int init_RC()
{
	manager.register_RCOUT(&rcout);
}
void init_external_compass()
{
	sensors::HMC5983 hmc5983;
}
int init_imufifo()
{
	int ret;
	ret = imufifo.open(FIFOPATH);
	if(ret < 0)
	{
		LOG2("androidUAV:open imu_fifo error\n");
		return -1;
	}
	else
	{
		manager.register_FIFO("imu_FIFO",&imufifo);
	}
}
int bsp_init_all()
{
	// Prio in the range -20  to  19
	setpriority(PRIO_PROCESS,getpid(),19);
	init_sensor();
	init_GPS();
	init_rc();
	init_RC();
	init_imufifo();
	//static androidUAV::ATimer timer[3];
	static androidUAV::ATimer timer1(98);
	static androidUAV::ATimer timer2(97);
	static androidUAV::ATimer timer3(96);
	manager.register_Timer("mainloop", &timer1);
	manager.register_Timer("log", &timer2);
	manager.register_Timer("imu", &timer3);
	timer1.set_period(0);
	timer2.set_period(0);
	timer3.set_period(0);
	param bsp_parameter("BSP",1);
	if(bsp_parameter)
	{
		param("time", 3000)=3000;
		param("ekf", 0)=2;
		param("err",0) = error_magnet|error_GPS;
		//
		
		param("mat", 1)=1;


		param("rc00", 1008) = 1008;
		param("rc01", 1453) = 1453;
		param("rc02", 1906) = 1906;

		param("rc10", 1008) = 1066;
		param("rc11", 1453) = 1444;
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


	return 0;
}
