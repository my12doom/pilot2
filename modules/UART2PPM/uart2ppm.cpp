#include <HAL/STM32F4/F4UART.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/Interface/ISysTimer.h>
#include <utils/param.h>
#include <stdio.h>
#include <Protocol/crc32.h>
#include <string.h>
#include <stdint.h>
#include "PPMOUT.h"
#include <HAL/sensors/PPMIn.h>
#include "checksum.h"
#include <BSP/boards/mo_v3/RCOUT.h>

// BSP
using namespace STM32F4;
using namespace HAL;
using namespace sensors;
using namespace dev_v2;


#define ESC_MAGIC 0xcc
int max_rpm = 0x4268;
int min_rpm = 0x1388;
int stop_pwm = 1000;
int idle_pwm = 1150;
int max_pwm = 1980;
RCOUT rcout;

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

static void swap(void *buf, int size)
{
	char *p = (char*)buf;
	int i;
	for(i=0; i<size/2; i++)
	{
		char t = p[i];
		p[i] = p[size-1-i];
		p[size-1-i] = t;
	}
}

uint16_t tobig(uint16_t v)
{
	return ((v&0xff) << 8) | (v >> 8);
}

uint16_t convert(uint16_t v, bool revert)
{
	int v0 = v-900;
	if (v0<0)
		v0 = 0;
	if (v0>1200)
		v0 = 1000;
	
	if (revert)
		return 2047 - v0*2047/1200;
	return v0*2047/1200;
}
F4GPIO C4(GPIOC, GPIO_Pin_4);
F4GPIO C5(GPIOC, GPIO_Pin_5);
int64_t last_print = 0;
int64_t last_esc_packet = 0;

int handle_esc(IUART *uart)
{
	uint8_t tmp[20] = {0};
	/*
	while(uart->available() >= 1)
	{
		uart->read(tmp, 1);
		if (systimer->gettime() - last_print > 2000)
		{
			printf("             \r");
		}
		printf("%02x,", tmp[0]);
		last_print = systimer->gettime();
	}
	*/
	
	if (systimer->gettime() - last_esc_packet > 200000)
	{
		int16_t zero[4] = {0};
		rcout.write(zero, 0, 4);
		C5.write(true);		
	}
	else
	{
		int v = systimer->gettime() % 500000;		
		C5.write(v>10000);
	}
	
	// wait for magic
	while (uart->available() >= 2)
	{
		uart->peak(tmp,2);
		if (tmp[0] == 0x0f && tmp[1] == 0x02)
			break;
		else
			uart->read(tmp, 1);
	}
	
	// wait for channel data and CRC
	if (uart->available() < 14)	// 14 = 2byte magic + 10 bytes channel data + 2 bytes CRC
		return -1;
		
	// check CRC
	uart->read(tmp, 14);
	uint16_t crc_calc = crc_calculate(tmp, 12);
	if (crc_calculate(tmp+2, 10) == *(uint16_t*)(tmp+12))
	{
		C4.write(false);
		//Output
	}
	else
	{
		C4.write(true);
	}
	
	// Output
	uint16_t *p = (uint16_t*)(tmp+2);
	int16_t out[4];
	for(int i=0; i<4; i++)
	{
		if (p[i] >= min_rpm)
		{
			C4.write(false);
			out[i] = idle_pwm + (p[i] - min_rpm) * (max_pwm-idle_pwm) / (max_rpm - min_rpm);
		}
		else
		{
			C4.write(true);
			out[i] = stop_pwm;
		}

		printf("%d,", out[i]);
	}
	
	printf("      \r");
	last_esc_packet = systimer->gettime();
	rcout.write(out, 0, 4);
	
	return 0;
}


int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	C4.set_mode(MODE_OUT_PushPull);
	C5.set_mode(MODE_OUT_PushPull);
	C4.write(true);
	C5.write(true);
	
	F4Interrupt interrupt;
	interrupt.init(GPIOB, GPIO_Pin_0, interrupt_rising);
	static PPMIN ppm;
	ppm.init(&interrupt);

	//RDP();
	
	F4UART uart1(USART1);
	uart1.set_baudrate(115200);
	
	for(int i=0; i<5; i++)
	{
		C4.write(false);
		systimer->delayms(100);
		C4.write(true);
		systimer->delayms(100);
	}
	
	int64_t last_tx = systimer->gettime();
	

	while(1)
	{
		int16_t pwm_input[8];
		ppm.get_channel_data(pwm_input, 0, 8);
		
		uint8_t data[16] = {0};
		uint16_t *p = (uint16_t*)data;
		p[0] = 0x9b03;
		
		int channel_ids[7] = {1,2,0,3,4,5,6,};
		bool reverts[7] = {true, false, false, true};
		
		for(int i=1; i<8; i++)
			p[i] = tobig( ( convert(pwm_input[i-1], reverts[i-1]) & 0x7ff) | (channel_ids[i-1] << 11) ) ;
		
		if (systimer->gettime() - last_tx > 20000 && pwm_input[2] > 900)
		{
			last_tx = systimer->gettime();
			uart1.write(data, sizeof(data));
		}
		
		handle_esc(&uart1);	
		
	}
}


int main2()
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