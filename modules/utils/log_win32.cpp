#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <Windows.h>
#include "log.h"

#include <Protocol/RFData.h>
#include <Protocol/common.h>
#include <HAL/Interface/Interfaces.h>

bool storage_ready = true;
int last_log_flush_time = -999999;

bool log_ready()
{
	return storage_ready && last_log_flush_time > systimer->gettime() - 2000000;
}


int format_sdcard()
{
	// TODO

	return 0;
}

int log_init()
{
	// TODO

	return 0;
}

int write_to_disk(void *data, int size)
{
	int64_t us = systimer->gettime();

	// TODO

	return 0;
}

int log_flush()
{
	//TODO

	return 0;
}

int log(const void *data, int size)
{
	// enqueue
	
	return 0;
}

int log_write(const void *data, int size)
{
	return log(data, size);
}


int log2(const void *packet, uint16_t tag, uint16_t size)
{	
	int64_t timestamp = systimer->gettime();
	timestamp &= ~((uint64_t)0xff << 56);
	timestamp |= (uint64_t)TAG_EXTENDED_DATA << 56;

	// enqueue
// 	buffer.put(&timestamp, 8);
// 	buffer.put(&tag, 2);
// 	buffer.put(&size, 2);
// 	buffer.put(packet, size);

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

uint32_t make_fattime(int sec, int min, int hour, int day, int mon, int year)
{
return ((year+1900-1980) << 25)
| ((mon+1) << 21)
| ((day) << 16)
| ((hour) << 11)
| ((min) << 5)
| ((sec) << 1)
;
	
}

extern "C" uint32_t get_fattime(void)
{
	if (!sys_time)
		return 0;

	struct tm _tm;
        /*time_t current_time = _unix_time + (GetTickCount()*1000 - sys_time) / 1000000;
	_tm = *localtime(&current_time);
	
	TRACE("\r%d, %d", current_time, _tm.tm_sec);
	
	return make_fattime(_tm.tm_sec, _tm.tm_min, _tm.tm_hour, _tm.tm_mday, _tm.tm_mon, _tm.tm_year);
        */
        return 0;
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
	
	return log2(buffer, TAG_TEXT_LOG, count);
}

int open_firmware()
{
	return -1;
}
int write_firmware(const void*data, int count)
{
	return -1;
}
int close_and_check_firmware(int trucancate_size)
{
	return -1;
}
