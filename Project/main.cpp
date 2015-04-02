#include <BSP/Resources.h>
#include <stdio.h>
#include <Algorithm/ahrs.h>
#include <utils/log.h>
#include <BSP/devices/sensors/UartNMEAGPS.h>

using namespace devices;
using namespace sensors;

int tmain(void)
{	
	bsp_init_all();
	char a[20];
	
	IGPS *gps = manager.get_GPS(0);
	IBarometer *baro = manager.get_barometer(0);
	
	/*
	while(1)
	{
		char data[1024];
		int count = uart3->read(data, 1024);
		for(int i=0; i<count; i++)
			fputc(data[i], NULL);
	}
	*/
	
	F4GPIO debug(GPIOA, GPIO_Pin_7);
	debug.set_mode(MODE_OUT_PushPull);
	
	
	while(1)
	{
		//gps_data data;
		//int res = gps->read(&data);
		
		baro_data data;
		baro->read(&data);
		
		systimer->delayms(3);
		
		printf("%f, %f       \n", data.pressure, data.temperature);
		if (data.temperature < 0)
			printf("WTF\n");
		debug.toggle();
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

