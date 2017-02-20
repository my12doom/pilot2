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
			memset(filePath,0,sizeof(filePath));
			memcpy(filePath,fifopath,sizeof(filePath));
		}
		memset(&imudata,0,sizeof(imu_data_t));
	}
	AFIFO::~AFIFO()
	{
		if(fifofd > 0)
			::close(fifofd);
	}
	bool AFIFO::healthy()
	{
		return (fifofd > 0)? 0: -1;
	}
	int AFIFO::open(const char* path,int mode)
	{
		if(!path || is_file_exist(path)<0)
			return -1;
		// open mode :O_NONBLOCK when pip buf is full return -1 immediately 
		fifofd = ::open(path,mode);
		if(fifofd < 0)
		{
			LOG2("androidUAV:open error\n");
			return -1;
		}
		memset(filePath,0,sizeof(filePath));
		memcpy(filePath,path,sizeof(filePath));
		return 0;
	}
	int AFIFO::close()
	{
		::close(fifofd);
		return 0;
	}
	int AFIFO::create(const char* path,int mode)
	{
		if(!path)
			return -1;
		if( is_file_exist(path) == 0)
		{
			fifofd = ::open(path,O_RDWR|O_NONBLOCK);
			LOG2("\nandroidUAV:open an existing fifo %d\n",fifofd);
			if(fifofd > 0)
			{
				memset(filePath,0,sizeof(filePath));
				memcpy(filePath,path,sizeof(filePath));
				return 0;
			}
			return -1;
		}
		else
		{
			int ret = mkfifo(path,mode);
			if(ret == 0)
			{
				fifofd = ::open(path,O_RDWR|O_NONBLOCK);
				memset(filePath,0,sizeof(filePath));
				memcpy(filePath,path,sizeof(filePath));
				return 0;
			}
			else
			{
				LOG2("\nandroidUAV:create fifo error path:%s %d\n",path,fifofd);
				return ret;
			}
		}
		return -1;
	}
	//return 1 no data avaliable,0
	int AFIFO::read(void *data,int size)
	{
		int ret = 0;
		int dataStructLen = sizeof(imu_data_t)*3;
		char buf[dataStructLen];
		int validCount = 0;
		/*if(!data || is_file_exist(filePath)<0)
		{
			if(fifofd > 0)
			{
				::close(fifofd);
				fifofd = 0;
			}
			return -1;
		}*/
		imu_data_t dataTmp;
		memset(&dataTmp,0,sizeof(imu_data_t));
		
		//ret = ::read(fifofd,&dataTmp,sizeof(imu_data_t));
		
		ret = ::read(fifofd,buf,dataStructLen);
		
		if(ret > 0)
		{
			for(int i=0;i<ret;i++)
			{
				if(buf[i] == (~buf[i+1]&0xff))
				{
					if(i+sizeof(imu_data_t)-1 < ret)
					{
						memcpy(&dataTmp,buf+i,sizeof(imu_data_t));
						validCount++;
						i+=sizeof(imu_data_t);
					}
				}
			}
		}
		else if(ret < 0)
		{
			//error occur
			return ret;
		}
		else
		{
			//no data avaliable
			return 1;
		}
		if(validCount)
		{
			if(dataTmp.pack_head_chk[0] != (~(dataTmp.pack_head_chk[1])&0xff) || crc16((uint8_t *)(&dataTmp)+2,sizeof(imu_data_t)-4) != dataTmp.crc)
			{
				//LOG2("androidUAV:data read error\n");
				return -1;
			}
			else
			{
				memcpy(data,&dataTmp,size);
				return 0;
			}
		}
		else
		{
			return 1;
		}
		return 1;
	}
	int AFIFO::write(void *data,int count)
	{
		int ret;
		/*if(!data || is_file_exist(filePath)<0)
		{
			if(fifofd > 0)
			{
				::close(fifofd);
				fifofd = 0;
			}
			return -1;
		}
		else
		{		
			if(fifofd<0 || !fifofd)
			{
				open(filePath,O_RDWR|O_NONBLOCK);
			}
		}*/

		memset(&imudata,0,sizeof(imu_data_t));
		imudata.pack_head_chk[0] = 0xfe;
		imudata.pack_head_chk[1] = ~imudata.pack_head_chk[0];
		imudata.time_stamp = gettime();
		
		memcpy(&imudata.array[0],data,count);
		imudata.crc = crc16((uint8_t *)(&imudata)+2,sizeof(imu_data_t)-4);
		ret = ::write(fifofd,&imudata,sizeof(imu_data_t));
		return (ret == sizeof(imu_data_t))?count:ret;
	}
	int AFIFO::is_file_exist(const char *filePath)
	{
		if(!filePath)
			return -1;
		
		if(access(filePath,F_OK) == 0)
			return 0;
		return -1;
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
static int64_t gettime()
{
	struct timespec tv;
	clock_gettime(CLOCK_MONOTONIC, &tv);
	return (int64_t)((tv.tv_sec) * 1000000 + (tv.tv_nsec)/1000);
}

