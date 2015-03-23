#include <stddef.h>
#include <string.h>
#include "log.h"

#include "../fat/ff.h"
#include "../common/fifo.h"
#include "../common/RFData.h"
#include "../common/common.h"
#include "../common/uart4.h"


FIL *file = NULL;
FRESULT res;
FATFS fs;
CircularQueue<rf_data, 256> log_buffer;
CircularQueue<rf_data, 256> log_buffer2;
CircularQueue<rf_data, 256> *plog_buffer = &log_buffer;
int log_pending = 0;
int last_log_flush_time = 0;
bool log_ready;
int LOG_LEVEL = LOG_SDCARD;

int save_log_packet(rf_data &packet)
{
	if (log_pending)
		return -1;

	return plog_buffer->push(packet);
}

extern "C"
{
	#include "../fat/diskio.h"

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

int real_log_packet(void *data, int size)
{
	int64_t us = getus();

#ifdef STM32F4
	// USART, "\r" are escaped into "\r\r"
	if (LOG_LEVEL & LOG_USART1)
	{
		const char *string = (const char*)data;
		char escaped[256];
		int i,j;		// j = escaped size
		for(i=0,j=0; i<size; i++,j++)
		{
			escaped[j] = string[i];
			if (string[i] == '\r')
				escaped[j++] = '\r';
		}

		escaped[j++] = '\r';
		escaped[j++] = '\n';

		UART4_SendPacket(escaped, j);
	}
#endif



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
			if (getus() - last_log_flush_time > 1000000)
			{
				last_log_flush_time = getus();
				f_sync(file);
			}
		}
	}
	if (getus() - us > 7000)
	{
		TRACE("log cost %d us  ", int(getus()-us));
		TRACE("  fat R/R:%d/%d\r\n", read_count, write_count);
	}
	// 	LOGE("\rfat R/R:%d/%d", read_count, write_count);
	// 	if (read_count + write_count > 1)
	// 		LOGE("\r\n");
	read_count = write_count = 0;




	return 0;
}

int log_flush()
{
	log_pending = 1;

	CircularQueue<rf_data, 256> *writer_buffer = plog_buffer;
	plog_buffer = (plog_buffer == &log_buffer) ? &log_buffer2 : &log_buffer;

	log_pending = 0;

	// real saving / sending
	if (writer_buffer->count() == 0)
		return 1;

	// disable USB interrupt to prevent sdcard dead lock
#ifdef STM32F1
	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
#endif
#ifdef STM32F4
	NVIC_DisableIRQ(OTG_HS_IRQn);
	//NVIC_DisableIRQ(OTG_FS_IRQn);
	NVIC_DisableIRQ(OTG_HS_EP1_IN_IRQn);
	NVIC_DisableIRQ(OTG_HS_EP1_OUT_IRQn);
#endif
	__DSB();
	__ISB();

	rf_data packet[256];
	int count = 0;
	while(writer_buffer->pop(&packet[count]) == 0)
		count++;
	real_log_packet(&packet[0], sizeof(rf_data)*count);

	// 	printf("%d\n", sizeof(rf_data)*count);

	// restore USB
#ifdef STM32F1
	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
#endif
#ifdef STM32F4
	NVIC_EnableIRQ(OTG_HS_IRQn);
	NVIC_EnableIRQ(OTG_FS_IRQn);
	NVIC_EnableIRQ(OTG_HS_EP1_IN_IRQn);
	NVIC_EnableIRQ(OTG_HS_EP1_OUT_IRQn);
#endif

	return 0;
}

int log(void *packet, uint8_t tag, int64_t timestamp)
{
	timestamp &= ~((uint64_t)0xff << 56);
	timestamp |= (uint64_t)tag << 56;

	rf_data rf;
	rf.time = timestamp;
	memcpy(&rf.data, packet, sizeof(rf.data));

	return save_log_packet(rf);
}