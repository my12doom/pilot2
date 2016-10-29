#include "log.h"
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <utils/fifo2.h>
#include <Protocol/RFData.h>
#include <Protocol/common.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/rk32885.1/ALog.h>

#define LOGFILEPATH "/data/androidUAV.log"

FILE* file = NULL;
FILE* yap_file;
uint32_t lost1 = 0; //buffer full
bool storage_ready = true;
volatile bool buffer_locked = false;

FIFO<16384> buffer;
int last_log_flush_time = -999999;
int file_number = 0;

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
	LOG2("androidUAV:log space init processing...\n");
	yap_file = fopen(LOGFILEPATH,"w+b");
	
	if(!yap_file)
	{
		LOG2("androidUAV:open log file failed\n");
		storage_ready = false;
		return -1;
	}
	storage_ready = true;
	if(storage_ready)
	{
		int size = 0;
		size = fread(&file_number,sizeof(file_number),sizeof(file_number),yap_file);
		if(size != 4)
			file_number = 0;
	}
	return 0;
}

int write_to_disk(void *data, int size)
{
	int64_t us = systimer->gettime();
	int ret = 0;
	if(file == NULL && storage_ready)
	{
		char filename[20];
		int done = file_number;
		memset(filename,0,sizeof(filename));
		while(storage_ready && yap_file)
		{
			sprintf(filename,"/data/%04d.dat",done++);
			file = fopen(filename,"w+b");
			if(file)
			{
				int tmp = 0;
				LOG2("androidUAV:opened %s for logging\n",filename);
				fseek(yap_file,0,SEEK_SET);
				tmp = fwrite(&done,sizeof(done),sizeof(done),yap_file);
				if(tmp != sizeof(done))
				{
					LOG2("androidUAV:write current file number error\n");
				}
				break;
			}
		}
	}
	if(storage_ready && file)
	{
		if((ret = fwrite(data,size,1,file)) != 1)
		{
			LOG2("androidUAV:write file ERROR write ret = %d size %d\n",ret,size);
			storage_ready = false;
		}
		if(systimer->gettime() - last_log_flush_time > 1000000)
		{
			last_log_flush_time = systimer->gettime();
		}
	}
	if(systimer->gettime() - us > 7000)
	{
		TRACE("log cost %d us",int(getus() -us));
	}
	return 0;
}

int log_flush()
{
	if(buffer.count() == 0)
		return 1;
	int count = buffer.count();
	uint8_t piece[2048];
	int piece_count = (count) / sizeof(piece);
	for(int i=0;i<piece_count;i++)
	{
		int piece_size = buffer.pop(piece,sizeof(piece));
		write_to_disk(piece,piece_size);
	}

	return 0;
}

int log(const void *data, int size)
{
	// enqueue
	if(buffer_locked)
		return -1;
	buffer_locked = true;
	int res = buffer.put(data,size);
	buffer_locked = false;
	if(res < 0)
		lost1++;
	return 0;
}

int log_write(const void *data, int size)
{
	return log(data, size);
}


int log2(const void *packet, uint16_t tag, uint16_t size)
{	
	if(buffer_locked)
		return -1;
	buffer_locked = true;
	
	if(buffer.available() < size+8+4)
	{
		lost1++;
		buffer_locked = false;
		return -1;
	}

	int64_t timestamp = systimer->gettime();
	timestamp &= ~((uint64_t)0xff << 56);
	timestamp |= (uint64_t)TAG_EXTENDED_DATA << 56;

	// enqueue
 	buffer.put(&timestamp, 8);
 	buffer.put(&tag, 2);
 	buffer.put(&size, 2);
 	buffer.put(packet, size);

	buffer_locked = false;
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
