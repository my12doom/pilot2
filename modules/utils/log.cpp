#include <stddef.h>
#include <string.h>
#include "log.h"

#include <FileSystem/ff.h>
#include <utils/fifo.h>
#include <Protocol/RFData.h>
#include <protocol/common.h>
#include <HAL/Interface/Interfaces.h>


FIL *file = NULL;
FRESULT res;
FATFS fs;
uint32_t lost1 = 0;		// buffer full
uint32_t lost2 = 0;		// log flush pending

class write_buffer
{
public:
	write_buffer(){byte_count=0;}
	~write_buffer(){}
	int push(const void *data, int size)
	{
		if (size+byte_count > sizeof(buffer))
		{
			lost1 ++;
			return -1;
		}
		
		memcpy(buffer + byte_count, data, size);
		byte_count += size;
		
		return 0;
	}
	void clear()
	{
		byte_count = 0;
	}
	int byte_count;
	char buffer[8192];
};

write_buffer log_buffer;
write_buffer log_buffer2;
write_buffer *plog_buffer = &log_buffer;
int log_pending = 0;
int last_log_flush_time = 0;
bool log_ready;
int LOG_LEVEL = LOG_SDCARD;

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
	if (LOG_LEVEL & LOG_SDCARD)
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
	log_pending = 1;

	write_buffer *writer_buffer = plog_buffer;
	plog_buffer = (plog_buffer == &log_buffer) ? &log_buffer2 : &log_buffer;

	log_pending = 0;

	// real saving / sending
	if (writer_buffer->byte_count == 0)
		return 1;

	write_to_disk(writer_buffer->buffer, writer_buffer->byte_count);
	writer_buffer->clear();

	return 0;
}

int log(const void *data, int size)
{
	if (log_pending)
	{
		lost2 ++;
		return -1;
	}

	return plog_buffer->push(data, size);
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