#include <HAL/STM32F4/F4UART.h>
#include <HAL/STM32F4/F4VCP.h>
#include <HAL/Interface/ISysTimer.h>
#include <BSP/boards/dev_v4/RGBLED.h>
#include <FileSystem/ff.h>
#include <utils/ymodem.h>
#include <utils/param.h>
#include <stdio.h>
#include <Protocol/crc32.h>
#include <string.h>
#include <stdint.h>
#include "stm32f4xx_flash.h"

// BSP
using namespace STM32F4;
F4UART uart1(USART1);
dev_v2::RGBLED led;

extern "C" void USART1_IRQHandler(void)
{
	uart1.USART1_IRQHandler();
}
extern "C" void DMA2_Stream7_IRQHandler()
{
	uart1.DMA2_Steam7_IRQHandler();
}

// constants
const uint32_t ApplicationAddress = 0x8008000;
float color[5][3] = 
{
	{1,0,0},
	{0,1,0},
	{0,0,1},
	{0.2,0,1},
	{0.2,1,0},
};


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

void run_rom();
void erase_rom(HAL::IUART *uart)
{
	uint32_t pages[] =
	{
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
	
	int sector_count = sizeof(pages)/sizeof(pages[0]);
		
	for(int i=0; i<sector_count; i++)
	{
		led.write(color[i%5][0], color[i%5][1], color[i%5][2]);
		char tmp[30];
		sprintf(tmp, "erasing sector %d...", i+1);
		uart->write(tmp, strlen(tmp));
		if (FLASH_EraseSector(pages[i], VoltageRange_3) == FLASH_COMPLETE)
			uart->write("OK\n", 3);
		else
			uart->write("ERROR\n", 6);
		
	}
	led.write(0,0,0);
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

extern "C" DWORD get_fattime()
{
	return 0;
}

int check_sdcard()
{
	FRESULT res;
	FATFS fs;
	FIL f;
	res = disk_initialize(0) == RES_OK ? FR_OK : FR_DISK_ERR;
	res = f_mount(&fs, "", 0);
	res = f_open(&f, "firmware.yap", FA_OPEN_EXISTING | FA_READ | FA_WRITE);
	
	if (res != FR_OK)
		return -1;
	
	if (f_size(&f) <= 8)
	{
		f_close(&f);
		return -2;
	}
	
	// check "YAP\A" tag and crc
	UINT got = 0;
	DWORD crc = 0;
	char tag[5] = {0};
	int file_rom_size = f_size(&f)-8;
	f_lseek(&f, f_size(&f)-8);
	f_read(&f, tag, 4, &got);
	f_read(&f, &crc, 4, &got);
	if (tag[0] != 'Y' || tag[1] != 'A' || tag[2] != 'P' || tag[3] != ' ')
	{
		f_close(&f);
		return -3;
	}
	
	if (crc == crc32(0, (uint8_t*)ApplicationAddress, file_rom_size))
	{
		f_close(&f);
		return -5;
	}

	// calculate CRC from content
	uint32_t crc_calculated = 0;
	char tmp[1024];
	int left = file_rom_size;
	f_lseek(&f, 0);
	while (left>0)
	{
		if (f_read(&f, tmp, left>sizeof(tmp)?sizeof(tmp):left, &got) != FR_OK)
		{
			f_close(&f);
			return -4;
		}

		crc_calculated = crc32(crc_calculated, tmp, got);
		left -= got;
	}

	if (crc != crc_calculated)
		return -5;


	// erase
	erase_rom(&uart1);

	// flash
	f_lseek(&f, 0);
	left = file_rom_size;
	uint32_t pos = ApplicationAddress;
	FLASH_Unlock();
	led.write(0,0,0);
	while (left>0)
	{
		if (f_read(&f, tmp, left>sizeof(tmp)?sizeof(tmp):left, &got) != FR_OK)
		{
			f_close(&f);
			return -6;
		}

		left -= got;

		for(int i=0; i<got; i+= 4)
			FLASH_ProgramWord(pos+i, *(uint32_t*)((uint8_t*)tmp+i));

		pos += got;
		int i = left / sizeof(tmp) / 10;
		led.write(color[i%5][0], color[i%5][1], color[i%5][2]);
	}
	FLASH_Lock();

	// save
	float sizef = *(float*)&file_rom_size;
	float crcf = *(float*)&crc_calculated;
	
	rom_size = sizef;
	rom_crc = crcf;
	
	FLASH_Unlock();
	rom_size.save();
	rom_crc.save();
	
	f_close(&f);
	//f_unlink("firmware.yap");
	
	return 0;
}

int handle_uart(HAL::IUART &uart1)
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
	
	return 0;
}

int main()	
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	RDP();
	uart1.set_baudrate(115200);
	check_sdcard();
	led.write(0,0,0);
	
	//if (check_rom_crc())
		run_rom();
	
	F4UART vcp(USART1);
	while(1)
	{
		handle_uart(uart1);
		handle_uart(vcp);
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