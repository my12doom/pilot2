#include "AIMUFIFO.h"

namespace androidUAV
{
	AFIFO::AFIFO(const char* fifopath):fifofd(0)
	{
		if(!fifopath)
		{
			LOG2("androidUAV:warning fifoPath NULL\n");
		}
		else
		{
			//fifofd = open();
		}
		memset(&imudata,0,sizeof(imu_data_t));
	}
	AFIFO::~AFIFO()
	{
		::close(fifofd);
	}
	int AFIFO::open(const char* path)
	{
		if(!path)
			return -1;
		fifofd = ::open(path,O_RDWR);
		if(fifofd < 0)
		{
			LOG2("androidUAV:open error\n");
		}
		return fifofd;
	}
	int AFIFO::close()
	{
		::close(fifofd);
	}
	int AFIFO::create(const char* path,int mode)
	{
		if(!path)
			return -1;
		fifofd = mkfifo(path,mode);
		return fifofd;
	}
	int AFIFO::read(void *data,int count)
	{
		int ret = 0;
		if(!data)
			return -1;
		memset(&imudata,0,sizeof(imu_data_t));
		
		ret = ::read(fifofd,&imudata,sizeof(imu_data_t));
		
		//ret = ::read(fifofd,data,count);
		return ret;
	}
	int AFIFO::write(void *data,int count)
	{
		if(!data)
			return -1;
		memset(&imudata,0,sizeof(imu_data_t));
		imudata.pack_head_chk[0] = 0xfe;
		imudata.pack_head_chk[1] = ~imudata.pack_head_chk[0];
		imudata.time_stamp = 1234;
		
		memcpy(&imudata.array[0],data,count);
		imudata.crc = crc16((uint8_t *)(&imudata)+2,sizeof(imu_data_t)-4);
		::write(fifofd,&imudata,sizeof(imu_data_t));
		return 0;
	}
	
}
static uint16_t UpdateCRC16(uint16_t crcIn,uint8_t byte)
{
	uint32_t crc = crcIn;
	uint32_t in = byte|0x100;
	do
	{
		crc <<= 1;
		in <<= 1;
		if(in&0x100)
			++crc;
		if(crc&0x10000)
			crc ^= 0x1021;
	}while(!(in&0x10000));
	return crc&0xffffu;
}
static uint16_t crc16(const uint8_t* data,uint32_t size)
{
	uint32_t crc = 0;
	const uint8_t* dataEnd = data+size;
	while(data<dataEnd)
		crc = UpdateCRC16(crc,*data++);
	crc = UpdateCRC16(crc,0);
	crc = UpdateCRC16(crc,0);
	return crc&0xffffu;
}
