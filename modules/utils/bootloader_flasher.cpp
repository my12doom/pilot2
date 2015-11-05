#include "bootloader_flasher.h"
#include <HAL/Interface/Interfaces.h>
#include <FileSystem/ff.h>
#include <Protocol/crc32.h>

int check_and_flash_bootloader(const char *filename)
{
	FRESULT res;
	FATFS fs;
	FIL f;
	res = disk_initialize(0) == RES_OK ? FR_OK : FR_DISK_ERR;
	res = f_mount(&fs, "", 0);
	res = f_open(&f, filename, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
	
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

	f_close(&f);
	
	return 0;
}
