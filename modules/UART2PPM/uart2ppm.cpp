#include <HAL/STM32F4/F4UART.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/Interface/ISysTimer.h>
#include <utils/param.h>
#include <stdio.h>
#include <Protocol/crc32.h>
#include <string.h>
#include <stdint.h>
#include "PPMOUT.h"

// BSP
using namespace STM32F4;
using namespace HAL;


extern "C" int log_printf(const char*format, ...)
{
	// do nothing
	return 0;
}


void RDP()
{
	if(FLASH_OB_GetRDP() != SET)
	{
		FLASH_Unlock();
		FLASH_OB_Unlock();
		FLASH_OB_RDPConfig(OB_RDP_Level_1);
		FLASH_OB_Launch();
		FLASH_OB_Lock();
		FLASH_Lock();
		NVIC_SystemReset();
	}
}

int main()	
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

	//RDP();
	F4GPIO C4(GPIOB, GPIO_Pin_1);
	C4.set_mode(MODE_OUT_PushPull);
	C4.write(false);
	F4UART uart4(UART4);
	PPMOUT ppm;
	uart4.set_baudrate(57600);
	
	int64_t last_ppm_out = systimer->gettime();
	int16_t data[6] = {1520, 1520, 1000, 1520, 1520, 1520};
	uint8_t channel_data[8] = {0};
		
	while(1)
	{
		if (systimer->gettime() > last_ppm_out + 20000)
		{
			for(int i=0; i<5; i++)
			{
				data[i] = channel_data[i] * 4 + 1000;
			}
			
			data[0] = 1000 + channel_data[0] * 4;
			data[1] = 2000 - channel_data[1] * 4;
			data[2] = 2000 - channel_data[2] * 4;
			data[3] = 1000 + channel_data[3] * 4;
			data[5] = 1000 + channel_data[4] * 4;
			
			
			
			if (channel_data[5])
				data[4] = 2000;
			else if (channel_data[6])
				data[4] = 2000;
			else if (channel_data[7])
				data[4] = 1000;
			else
				data[4] = 1000;
			
			
			ppm.write(data, 6);
			last_ppm_out = systimer->gettime();
		}
		
		if (uart4.available() < 2)
			continue;
		
		uint8_t start_code[2];
		uart4.read(&start_code[0], 1);
		if (start_code[0] != 0x85)
			continue;

		uart4.read(&start_code[1], 1);
		if (start_code[1] != 0xA3)
			continue;
		
		int64_t timeout = systimer->gettime() + 10000;
		
		while(uart4.available() < sizeof(channel_data) && systimer->gettime() < timeout)
			;
		
		if (uart4.available() >= sizeof(channel_data))
		{
			uart4.read(channel_data, sizeof(channel_data));
			
			uart4.write("data:", 5);
			for(int i=0; i<sizeof(channel_data); i++)
			{
				char tmp[20];
				sprintf(tmp, "%d,", channel_data[i]);
				uart4.write(tmp, strlen(tmp));				
			}
			
			uart4.write("\n", 1);
		}
		else
		{
			uart4.write("invalid packet\n", 15);
		}
	}
}


struct __FILE { int handle;  };
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