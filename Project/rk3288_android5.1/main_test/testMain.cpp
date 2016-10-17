#include <stdlib.h>
#include <HAL/rk32885.1/AGpio.h>
#include <HAL/rk32885.1/ASysTimer.h>
#include <HAL/rk32885.1/ATimer.h>

#include <HAL/rk32885.1/ASPI.h>
#include <HAL/rk32885.1/AStorage.h>
#include <HAL/rk32885.1/AUART.h>
#include <BSP/boards/rk3288UAV/ARCOUT.h>
#include <HAL/sensors/MPU6000.h>
#include <HAL/sensors/MS5611_SPI.h>
#include <HAL/sensors/UartUbloxNMEAGPS.h>
#include <HAL/Interface/IUART.h>
static const char *spidevice = "/dev/oledEuler_dev";

static const char *gpiodevice = "/dev/luobogpio";

static const char *uart3device = "/dev/ttyS3";

static const char *uart4device = "/dev/ttyS4";

static const char *pwmdevice = "/dev/eulerpwm0";

androidUAV::AUART uart(uart3device);
androidUAV::AUART uart4(uart4device);

static sensors::UartUbloxBinaryGPS gps;
int init_GPSMain()
{


	if(gps.init(&uart,115200) == 0)

	return 0;
}
int16_t pwmdata[4] = {0};
int16_t pwmoutdata[4] = {0};
int main(int argc,char** arvgv)
{
	//storage test
	int ret;
	char send[2] = {0};
	androidUAV::ARCOUT pwms(pwmdevice);

	//androidUAV::AStorage storage;
	//storage.erase(0);
	/*Android_TIME::ASysTimer timer;
	sensors::MPU6000 mpu6000;
	sensors::MS5611_SPI ms5611;
	//mpu6000 and gpio test

	HAL::IGPIO *gpio = new HAL::AGPIO(gpiodevice,228);//6000 cs
	HAL::IGPIO *gpio2 = new HAL::AGPIO(gpiodevice,230);//ms5611 cs

	androidUAV::ASPI spi(spidevice);

	devices::accelerometer_data acc_data;
	devices::gyro_data gyro_data;
	devices::baro_data ms5611_data;
	spi.setSpeed(500000);

	ret = mpu6000.init(&spi,gpio);
	ret = ms5611.init(&spi,gpio2);
	if(ret < 0)
		printf("Init ms561 error\n");
	timer.delayms(100.0);
	int exit;

	mpu6000.accelerometer_axis_config(0, 1, 2, +1, +1, +1);
	mpu6000.gyro_axis_config(0, 1, 2, +1, +1, +1);*/
	//printf("in gps init process\n");
	//uart.set_baudrate(115200);
	/*if(gps.init(&uart,115200) == 0)
	{
		printf("init gps cuccess\n");
	}
	else
	{
		printf("Init error\n");
		//return -1;
	}
	while(1);*/
	uart4.set_baudrate(115200);
	pwmdata[0] = 15;
	pwms.write(pwmdata,3,1);
	pwms.read(pwmoutdata,0,4);
	for(int i = 0;i<4;i++)
		printf("r ead %d\n",pwmoutdata[i]);
	while(1)
	{
		uart4.write("hello",sizeof("hello"));
		sleep(1);
	}
	int cnt = 0;
	char data = 0;
	send[0] = '2';
	send[1] = '3';
	while(1)
	{
		//
		for(int i = 0;i < 1000;i++)
		{
			uart.write(send,1);
			//usleep(150);
		}
		send[0] = '3';
		uart.set_baudrate(9600);
		sleep(1);
		for(int i = 0;i < 1000;i++)
		{
			uart.write(send,1);
			//usleep(150);
		}
		break;
		//uart.read(&data,1);
		//printf("%c\n",data);
	}
	printf("end\n");
	while(1);
	    //sleep(1);

	while(1)
	{
		/*mpu6000.read(&acc_data);
		mpu6000.read(&gyro_data);
		printf("gyro = %f %f %f\n\n",gyro_data.x,gyro_data.y,gyro_data.z);
		printf("acc = %f %f %f\n",acc_data.x,acc_data.y,acc_data.z);*/
		/*ms5611.read(&ms5611_data);
		printf("press = %f %f\n",ms5611_data.pressure,ms5611_data.temperature);
		timer.delayms(100.0);*/
	}
	return 0;
}
