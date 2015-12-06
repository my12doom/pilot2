#include <stddef.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "log.h"

#include <FileSystem/ff.h>
#include <utils/fifo2.h>
#include <Protocol/RFData.h>
#include <protocol/common.h>
#include <HAL/Interface/Interfaces.h>

extern "C"
{
#include "SEGGER_RTT.h"
}

FIL *file = NULL;
FRESULT res;
FATFS fs;
uint32_t lost1 = 0;		// buffer full

__attribute__((section("dma"))) FIFO<16384> buffer;
int last_log_flush_time = -999999;
bool log_ready = false;

extern "C"
{
	#include <HAL/Interface/SDCard.h>

};

int format_sdcard()
{
	res = disk_initialize(0) == RES_OK ? FR_OK : FR_DISK_ERR;
	res = f_mount(&fs, "", 0);
	res = f_mkfs("", 0, 0);
	return 0;
}

int log_init()
{
	//format_sdcard();
	LOGE("sdcard init...");
	FIL f;
	res = disk_initialize(0) == RES_OK ? FR_OK : FR_DISK_ERR;
	res = f_mount(&fs, "", 0);
	res = f_open(&f, "test.bin", FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
	log_ready = res == FR_OK;
	f_close(&f);
	LOGE("%s\r\n", log_ready ? "OK" : "FAIL");
	return 0;
}

int write_to_disk(void *data, int size)
{
	int64_t us = systimer->gettime();

	// fatfs
	{
		if (file == NULL && log_ready)
		{
			static FIL f;
			file = &f;
			char filename[20];
			int done  = 0;
			while(log_ready)
			{
				sprintf(filename, "%04d.dat", done ++);
				FRESULT res = f_open(file, filename, FA_CREATE_NEW | FA_WRITE | FA_READ);
				if (res == FR_OK)
				{
					f_close(file);
					res = f_open(file, filename, FA_OPEN_EXISTING | FA_WRITE | FA_READ);
					LOGE("opened %s for logging\n", filename);
					break;
				}
			}
		}

		if (log_ready && file)
		{
			unsigned int done;
			if (f_write(file, data, size, &done) != FR_OK || done !=size)
			{
				LOGE("\r\nSDCARD ERROR\r\n");
				log_ready = false;
			}
			if (systimer->gettime() - last_log_flush_time > 1000000)
			{
				last_log_flush_time = systimer->gettime();
				f_sync(file);
			}
		}
	}
	if (systimer->gettime() - us > 7000)
	{
		TRACE("log cost %d us  ", int(getus()-us));
		TRACE("  fat R/R:%d/%d\r\n", read_count, write_count);
	}
	
	
	read_count = write_count = 0;

	return 0;
}

int log_flush()
{
	// real saving / sending	
	if (buffer.count() == 0)
		return 1;

	int count = buffer.count();
	uint8_t piece[2048];
	int piece_count = (count) / sizeof(piece);
	for(int i=0; i<piece_count; i++)
	{
		int piece_size = buffer.pop(piece, sizeof(piece));
		write_to_disk(piece, piece_size);
	}

	return 0;
}

int log(const void *data, int size)
{
	int res = buffer.put(data, size);
	
	if (res < 0)
		lost1++;
	
	return 0;
}

int log2(const void *packet, uint16_t tag, uint16_t size)
{
	if (buffer.available() < size+8+4) 
	{
		lost1++;
		return -1;
	}
	
	int64_t timestamp = systimer->gettime();
	timestamp &= ~((uint64_t)0xff << 56);
	timestamp |= (uint64_t)TAG_EXTENDED_DATA << 56;

	buffer.put(&timestamp, 8);
	buffer.put(&tag, 2);
	buffer.put(&size, 2);
	buffer.put(packet, size);

	return 0;
}

int log(const void *packet, uint8_t tag, int64_t timestamp)
{
	timestamp &= ~((uint64_t)0xff << 56);
	timestamp |= (uint64_t)tag << 56;

	rf_data rf;
	rf.time = timestamp;
	memcpy(&rf.data, packet, sizeof(rf.data));

	return log(&rf, sizeof(rf));
}

static time_t _unix_time;
static int64_t sys_time = 0;

// set current UTC.
int log_set_time(time_t unix_time)				
{
	if (sys_time)
		return 0;

	sys_time = systimer->gettime();
	_unix_time = unix_time;

	return 0;
}

DWORD make_fattime(int sec, int min, int hour, int day, int mon, int year)
{
return ((year+1900-1980) << 25)
| ((mon+1) << 21)
| ((day) << 16)
| ((hour) << 11)
| ((min) << 5)
| ((sec) << 1)
;
	
}

extern "C" DWORD get_fattime(void)
{
	if (!sys_time)
		return 0;

	struct tm _tm;
	time_t current_time = _unix_time + (systimer->gettime() - sys_time) / 1000000;
	_tm = *localtime(&current_time);
	
	LOGE("\r%d, %d", current_time, _tm.tm_sec);
	
	return make_fattime(_tm.tm_sec, _tm.tm_min, _tm.tm_hour, _tm.tm_mday, _tm.tm_mon, _tm.tm_year);
}

int log_printf(const char*format, ...)
{
	char buffer[512];
		
	va_list args;
	va_start (args, format);
	int count = vsprintf (buffer,format, args);
	va_end (args);
	
	if (count < 0)
		return count;
	
	printf(buffer);
	SEGGER_RTT_WriteString(0, buffer);
	
	return log2(buffer, TAG_TEXT_LOG, count);
}
