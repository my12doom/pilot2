#include <BSP/Resources.h>
#include <stdio.h>
#include <Algorithm/ahrs.h>
#include <utils/log.h>

using namespace devices;

int main(void)
{	
	bsp_init_all();
	log_init();
	
		
	IAccelerometer * accel = manager.get_accelerometer(0);
	IGyro * gyro = manager.get_gyroscope(0);
	IBarometer * baro = manager.get_barometer(0);
	IMagnetometer * mag = manager.get_magnetometer(0);
	F4GPIO debug(GPIOA,GPIO_Pin_7);
	debug.set_mode(MODE_OUT_PushPull);
	
	accelerometer_data aavg = {0};
	gyro_data gavg = {0};
	baro_data bavg = {0};
	mag_data mavg = {0};
	int baro_counter = 0;
	for(int i=0; i<1500; i++)
	{
		accelerometer_data adata;
		gyro_data gdata;
		baro_data bdata;
		mag_data mdata;
		
		int res = accel->read(&adata);
		res = gyro->read(&gdata);
		res = mag->read(&mdata);
		res = baro->read(&bdata);
		
		aavg.x += adata.x;
		aavg.y += adata.y;
		aavg.z += adata.z;
		
		gavg.x += gdata.x;
		gavg.y += gdata.y;
		gavg.z += gdata.z;
		
		mavg.x += mdata.x;
		mavg.y += mdata.y;
		mavg.z += mdata.z;
		
		if (baro == 0)
		{
			bavg.temperature += bdata.temperature;
			bavg.pressure += bdata.pressure;
			baro_counter ++;
		}
		
		systimer->delayms(3);
	}
	
	aavg.x /= 1500;
	aavg.y /= 1500;
	aavg.z /= 1500;
	
	gavg.x /= 1500;
	gavg.y /= 1500;
	gavg.z /= 1500;
	
	mavg.x /= 1500;
	mavg.y /= 1500;
	mavg.z /= 1500;
		
	bavg.temperature /= baro_counter;
	bavg.pressure /= baro_counter;
	
	NonlinearSO3AHRSinit(
	aavg.x, aavg.y, aavg.z,
	mavg.x, mavg.y, mavg.z, 
	gavg.x, gavg.y, gavg.z);
	
	
	int64_t tick = systimer->gettime();
	while(1)
	{
		volatile int64_t now = systimer->gettime();
		float dt = (now - tick)/1000000.0f;
		tick = now;
		accelerometer_data adata;
		gyro_data gdata;
		baro_data bdata;
		mag_data mdata;
		int res = accel->read(&adata);
		res = gyro->read(&gdata);
		res = baro->read(&bdata);
		res = mag->read(&mdata);
		
		NonlinearSO3AHRSupdate(
		adata.x, adata.y, adata.z,
		mdata.x, mdata.y, mdata.z, 
		gdata.x, gdata.y, gdata.z,
		1.5f, 0.015f, 0.15f, 0.0015f, dt);
		
		float scale = 180/3.1415926f;
		//printf("\r%d, %f,%f,%f,%f, angular rate: %f,%f,%f deg/s", res, adata.x, adata.y, adata.z, adata.temperature, gdata.x * scale, gdata.y * scale, gdata.z * scale);
		
		//printf("%f, %f", bdata.pressure, bdata.temperature);
		//systimer->delayms(10);
		
		printf("\reuler: %f,%f,%f", euler[0] * scale, euler[1] * scale, euler[2] * scale);
		systimer->delayms(3);
	}
}


struct __FILE { int handle; /* Add whatever you need here */ };
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

extern "C" int fputc(int ch, FILE *f)
{
	if (DEMCR & TRCENA) 
	{
		while (ITM_Port32(0) == 0);
		ITM_Port8(0) = ch;
	}
	return (ch);
}

