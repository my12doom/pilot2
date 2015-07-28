#include <HAL/STM32F4/F4UART.h>
#include <HAL/Interface/ISysTimer.h>
#include <utils/ymodem.h>
#include <utils/param.h>
#include <stdio.h>
#include <Protocol/crc32.h>
#include <string.h>
#include "stm32f4xx_flash.h"

// BSP
using namespace STM32F4;
F4UART uart1(USART1);

extern "C" void USART1_IRQHandler(void)
{
	uart1.USART1_IRQHandler();
}
extern "C" void DMA2_Stream7_IRQHandler()
{
	uart1.DMA2_Steam7_IRQHandler();
}

// constants
const uint32_t ApplicationAddress = 0x8004000;

class ymodem_rec : public ymodem_receiver
{
public:
	ymodem_rec(HAL::IUART*uart):ymodem_receiver(uart)
	{
		pos = 0;
	}
	~ymodem_rec(){}
	virtual int on_event(void *data, int datasize, ymodem_event event)
	{
		//printf("event:%d, %d byte\n", event, datasize);
		if (event == ymodem_file_data)
		{
			for(int i=0; i<datasize/4*4; i+=4)
			{
				FLASH_ProgramWord(ApplicationAddress+pos+i, *(uint32_t*)((uint8_t*)data+i));
				FLASH_WaitForLastOperation();
			}
			
			pos += datasize;
		}
		return 0;
	}
	int pos;
};

// parameter
static param rom_size("size", 0);
static param rom_crc("CRC", 0);

void receive_rom(HAL::IUART *uart)
{
	ymodem_rec rec(uart);
	
	FLASH_Unlock();
	int tmp = ymodem_wait_file_header;
	do
	{
		tmp = rec.run();
	}while(tmp != ymodem_state_error && tmp != ymodem_transition_ended);
	FLASH_Lock();
}

void run_rom();
void erase_rom(HAL::IUART *uart)
{
	uint32_t pages[] =
	{
		FLASH_Sector_1,
		FLASH_Sector_2,
		FLASH_Sector_3,
		FLASH_Sector_4,
		FLASH_Sector_5,
		FLASH_Sector_6,
		FLASH_Sector_7,
		FLASH_Sector_8,
		FLASH_Sector_9,
	};
	
	FLASH_Unlock();
	rom_size = 0;
	rom_crc = 0;
	rom_size.save();
	rom_crc.save();
	
	for(int i=0; i<sizeof(pages)/sizeof(pages[0]); i++)
	{
		char tmp[30];
		sprintf(tmp, "erasing sector %d...", i+1);
		uart->write(tmp, strlen(tmp));
		if (FLASH_EraseSector(pages[i], VoltageRange_3) == FLASH_COMPLETE)
			uart->write("OK\n", 3);
		else
			uart->write("ERROR\n", 6);
	}
	uart->write("DONE\n", 5);
	FLASH_Lock();
}

bool check_rom_crc()
{
	// the CRC and size are stored "as" float, but are indeed binary uint32_t.
	float sizef = rom_size;
	float crcf = rom_crc;
	uint32_t size = *(uint32_t*)&sizef;
	uint32_t crc = *(uint32_t*)&crcf;	
	
	if (size <= 1024 || size > 1024*768)
		return false;
	
	return crc32(0, (uint8_t*)ApplicationAddress, size) == crc;
}

int main()	
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	uart1.set_baudrate(115200);
		
	if (check_rom_crc())
		run_rom();
	
	while(1)
	{
		char tmp[100] = {0};
		if (uart1.readline(tmp, sizeof(tmp)) > 0)
		{
			if (strstr(tmp, "run") == tmp)
			{
				uart1.write("running\n", 8);
				systimer->delayms(200);
				run_rom();
			}
			else if (strstr(tmp, "check") == tmp)
			{
				if (check_rom_crc())
					uart1.write("CRC OK\n", 7);
				else
					uart1.write("CRC FAILED\n", 11);
			}
			else if (strstr(tmp, "romcrc,") == tmp)
			{
				//uart1.write("romcrc only supportted in main ROM\n", 35);
				
								uint32_t size = 0;
				uint32_t crc = 0;
				
				if (sscanf(tmp+7, "%d,%x", &size, &crc) == 2)
				{
					float sizef = *(float*)&size;
					float crcf = *(float*)&crc;
					
					rom_size = sizef;
					rom_crc = crcf;
					
					//FLASH_Unlock();
					//rom_size.save();
					//rom_crc.save();
				}
				
				if (check_rom_crc())
					uart1.write("CRC OK\n", 7);
				else
					uart1.write("CRC FAILED\n", 11);

			}
			
			else if (strstr(tmp, "erase") == tmp)
				erase_rom(&uart1);
			else if (strstr(tmp, "rom") == tmp)
				receive_rom(&uart1);
			else if (strstr(tmp, "hello") == tmp)
				uart1.write("bootloader\n", 11);
		}			
	}
}

void run_rom()
{
	typedef void (*FunVoidType)(void);

    if (((*(vu32*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
    { 
        // TODO: close all BSP
		uart1.destroy();
		NVIC_DisableIRQ(SysTick_IRQn);
		
		FLASH_Lock();

        /* Jump to user application */
        FunVoidType JumpToApplication = (FunVoidType) *(vu32*) (ApplicationAddress + 4);

        /* Initialize user application's Stack Pointer */
        //__MSR_MSP(*(vu32*) ApplicationAddress);
		__set_MSP(*(vu32*) ApplicationAddress);
        JumpToApplication();
		
    }
	
	// reset system if booting application failed
	uart1.write("failed, rebooting\n", 18);
	systimer->delayms(200);
	NVIC_SystemReset();
}


/*
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
*/