#pragma once
#include <HAL/rk32885.1/ALog.h>
#include <HAL/Interface/IFIFO.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/msg.h>
#define FIFOPATH "/data/IMU_FIFO"
#define FIFO_CREATE_MODE 0666
#define FIFO_WRITE_MODE (O_APPEND | O_WRONLY)
#define FIFO_READ_MODE (O_RDONLY)
#define IMUDATANUM 4

#pragma pack(1) 

typedef struct imu
{
	uint8_t pack_head_chk[2];
	int64_t time_stamp;
	float array[IMUDATANUM];
	uint16_t crc;
}imu_data_t;
#pragma pack()

namespace androidUAV
{
	class AFIFO : public HAL::IFIFO
	{
		public:
			AFIFO(const char* fifopath=NULL);
			~AFIFO();
			virtual int open(const char* path);
			virtual int close();
			virtual int create(const char* path,int mode);
			virtual int read(void *data,int count);
			virtual int write(void *data,int count);
		protected:
			int fifofd;
			imu_data_t imudata;
	};
}
static uint16_t crc16(const uint8_t* data,uint32_t size);
static uint16_t UpdateCRC16(uint16_t crcIn,uint8_t byte);
