#include <HAL/STM32F4/F4UART.h>
#include <HAL/STM32F4/F4VCP.h>
#include <HAL/Interface/ISysTimer.h>
#include <BSP/boards/dev_v4/RGBLED.h>
#include <FileSystem/ff.h>
#include <utils/ymodem.h>
#include <utils/param.h>
#include <utils/space.h>
#include <utils/RIJNDAEL.h>
#include <stdio.h>
#include <Protocol/crc32.h>
#include <string.h>
#include <stdint.h>
#include "stm32f4xx_flash.h"

// BSP
using namespace STM32F4;
F4UART uart(USART3);
dev_v2::RGBLED led;

// constants
const uint32_t ApplicationAddress = 0x8008000;
const unsigned char aes_key[32] = {0x85, 0xA3, 0x6C, 0x69, 0x76, 0x6C, 0x61, 0x76, 0xA3, 0x85};
float color[5][3] = 
{
	{1,0,0},
	{0,1,0},
	{0,0,1},
	{0.2,0,1},
	{0.2,1,0},
};
AESCryptor aes;

extern "C" int log_printf(const char*format, ...)
{
	// do nothing
	return 0;
}

class ymodem_rec : public ymodem_receiver
{
public:
	ymodem_rec(HAL::IUART*uart):ymodem_receiver(uart)
	{
		pos = 0;
		first4bytes = 0xffffffff;
	}
	~ymodem_rec(){}	
	virtual int on_event(void *data, int datasize, ymodem_event event)
	{
		//printf("event:%d, %d byte\n", event, datasize);
		if (event == ymodem_file_data)
		{
			for(int i=0; i<datasize/4*4; i+=4)
			{
				// skip the first 4 bytes
				if (pos+i == 0)
				{
					first4bytes = *(uint32_t*)((uint8_t*)data+i);
					continue;
				}
				
				FLASH_ProgramWord(ApplicationAddress+pos+i, *(uint32_t*)((uint8_t*)data+i));
				FLASH_WaitForLastOperation();
			}
			
			pos += datasize;
		}
		return 0;
	}
	void finish()
	{
		if (first4bytes != 0xffffffff)
		{
			FLASH_ProgramWord(ApplicationAddress, first4bytes);
			FLASH_WaitForLastOperation();
		}			
	}
	int pos;
	uint32_t first4bytes;
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
	rec.finish();
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
	led.write(1,1,1);
	uint32_t pages[] =
	{
#ifndef STM32F446RC		
		FLASH_Sector_2,
		FLASH_Sector_3,
		FLASH_Sector_4,
		FLASH_Sector_5,
		FLASH_Sector_6,
		FLASH_Sector_7,
		FLASH_Sector_8,
		FLASH_Sector_9,
#else
		FLASH_Sector_4,
		FLASH_Sector_5,
#endif
	};
	
	FLASH_Unlock();
	rom_size = 0;
	rom_crc = 0;
	rom_size.save();
	rom_crc.save();
	
	// set the "boot" flag and resort parameters
	param bootloader_flag("boot", 1);
	bootloader_flag = 1.0f;
	bootloader_flag.save();
	space_resort();
	
	
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
		uint32_t rom_in_file_crc = 0;

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
	
	// read first 4 bytes and check encrpted tag
	bool encrypted = false;
	unsigned char encrypted_tag[4] = {0x85, 0xA3, 0xA3, 0x85};
	unsigned char normal_first4bytes[4] = {0xC0, 0x41, 0x01, 0x20};
	char tmp[1024];
	f_lseek(&f, 0);
	uint32_t first4bytes = 0;
	f_read(&f, &first4bytes, 4, &got);
	if (memcmp(encrypted_tag, &first4bytes, 4) == 0 && ((file_rom_size-4)%1024)== 0)
	{
		encrypted = true;
	}
	
	// check ROM CRC
	if (encrypted)
	{
		// the last block of 1024 bytes is reserved and currently used for encryption.
		f_lseek(&f, file_rom_size-1024);
		if (f_read(&f, tmp, sizeof(tmp), &got) != FR_OK)
		{
			f_close(&f);
			return -4;
		}
		
		for(int i=0; i<got; i+=16)
			aes.decrypt((unsigned char*)tmp+i, (unsigned char*)tmp+i);

		// check rom crc
		memcpy(&rom_in_file_crc, tmp, 4);
		memcpy(&first4bytes, tmp+4, 4);
		if (rom_in_file_crc == crc32(0, (uint8_t*)ApplicationAddress+4, file_rom_size-1024-4))
		{
			f_close(&f);
			return -5;
		}
	}
	else
	{
		if (crc == crc32(0, (uint8_t*)ApplicationAddress, file_rom_size))
		{
			f_close(&f);
			return -5;
		}
	}

	// calculate CRC from content
	uint32_t crc_calculated = 0;
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
	erase_rom(&uart);

	// flash, skipping the first 4 bytes for failsafe.
	// run_rom() will fail if the first 4 bytes failed and enter bootloader mode.
	// normal first 4 bytes is 0xC0, 0x41, 0x01, 0x20
	// encrypted rom's first 4 bytes is 0x85, 0xA3, 0xA3, 0x85, and size should be N*1024+4
	left = file_rom_size-4;
	f_lseek(&f, 4);
	uint32_t pos = ApplicationAddress+4;
	FLASH_Unlock();
	led.write(0,0,0);
	while (left>0)
	{
		memset(tmp, 0, sizeof(tmp));
		if (f_read(&f, tmp, left>sizeof(tmp)?sizeof(tmp):left, &got) != FR_OK)
		{
			f_close(&f);
			return -6;
		}
		
		if (encrypted)
		{
			// last block is reserved, and currently only for ROM CRC.
			if (left == got)
				break;
			
			for(int i=0; i<got; i+=16)
				aes.decrypt((unsigned char*)tmp+i, (unsigned char*)tmp+i);
		}

		left -= got;

		for(int i=0; i<got; i+= 4)
			FLASH_ProgramWord(pos+i, *(uint32_t*)((uint8_t*)tmp+i));

		pos += got;
		int i = left / sizeof(tmp) / 10;
		led.write(color[i%5][0], color[i%5][1], color[i%5][2]);
	}
	FLASH_ProgramWord(ApplicationAddress, first4bytes);
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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
	PWR_BackupAccessCmd(ENABLE);

	aes.set_key(aes_key, 256);
	RDP();
	check_sdcard();
	led.write(0,0,0);

	void * bkp = (void*)0x40002850;
	if (memcmp(bkp, "hello", 6))
		run_rom();
	
	uart.set_baudrate(57600);
	//F4VCP vcp;
	while(1)
	{
		handle_uart(uart);
		//handle_uart(vcp);
	}
}

void run_rom()
{
	typedef void (*FunVoidType)(void);

    if (((*(vu32*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
    { 
        // TODO: close all BSP
		uart.destroy();
		SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;                    // Disable SysTick
		
		FLASH_Lock();

        /* Jump to user application */
        FunVoidType JumpToApplication = (FunVoidType) *(vu32*) (ApplicationAddress + 4);

		// clear bootloader flag
		*(uint32_t *)0x40002850 = 0;
        /* Initialize user application's Stack Pointer */
        //__MSR_MSP(*(vu32*) ApplicationAddress);
		__set_MSP(*(vu32*) ApplicationAddress);
        JumpToApplication();
		
    }
	
	// reset system if booting application failed
	//uart.write("failed, rebooting\n", 18);
	//systimer->delayms(200);
	//NVIC_SystemReset();
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