#include "ASonar.h"

namespace sensors
{
	ASonar::ASonar()
	{
		exit = 1;
	}
	ASonar::~ASonar()
	{
		
	}
	int ASonar::init(HAL::IUART *uart)
	{
		if(!uart)
			return -1;
		LOG2("androidUAV:init ASonar success\n");
		this->uartSonar = uart;
		
		readThreadId = pthread_create(&tidp,NULL,read_thread,(void *)this);
		
		return 0;
	}
	// return 0 if new data available, 1 if old data, negative for error.
	// unit: meter.
	// timestamp: unit: milli-second, pass NULL or leave it default to ignore it.
	int ASonar::read(float *out, int64_t *timestamp/* = NULL */)
	{
		float data;
		char sonarData[30];
		uint32_t crcData = 0;
		uint32_t crcCalc = -1;
		int ret;
		ret = uartSonar->read(sonarData,sizeof(sonarData));
		if(ret >= 10)//new data here
		{
			int validConut = 0;
			for(int i=0;i<sizeof(sonarData);i++)
			{
				if(sonarData[i] == 0x85 && sonarData[i+1] == 0xa4)
				{
					if(i+2+sizeof(data)+sizeof(crcData) < sizeof(sonarData))
					{
						memcpy(&data,sonarData+i+2,sizeof(data));
						memcpy(&crcData,sonarData+i+2+sizeof(data),sizeof(crcData));
						crcCalc = crc32(0,&data,sizeof(data));
						if(crcData == crcCalc)
						{
							*out = data;
						}
						i+=2+sizeof(data)+sizeof(crcData);
						validConut++;
					}
				}
			}
			if(validConut > 0)
				return 0;
			else
				return 1;
		}
		else if(ret == 0)
		{
			return 1;
		}

		return -1;
	}

	// trigger messuring manually, this is needed by some types of range finder(sonars e.g.)
	int ASonar::trigger()
	{
		return 0;
	}

	// return false if any error/waning
	bool ASonar::healthy()
	{
		return (uartSonar)?0:-1;
	}
	
	void *ASonar::read_thread(void *p)
	{
		ASonar *_this = (ASonar *)p;
		_this->readProcess();
		pthread_exit(0);
		return 0;
	}
	void ASonar::readProcess()
	{
		int ret;
		float data;
		while(exit)
		{
			read(&data);
			//printf("dat %.3f\n",data);
			//uartSonar->write(&sonarData,sizeof(sonarData));
			usleep(20000);
		}
	}
	int ASonar::update_buffer()
	{
		float data;
		char sonarData[30];
		uint32_t crcData = 0;
		uint32_t crcCalc = -1;
		int ret;
		ret = uartSonar->read(sonarData,sizeof(sonarData));
		printf("ret %d\n",ret);
		for(int i=0;i<sizeof(sonarData);i++)
		{
			if(sonarData[i] == 0x85 && sonarData[i+1] == 0xa4)
			{
				memcpy(&data,sonarData+i+2,sizeof(data));
				memcpy(&crcData,sonarData+i+2+sizeof(data),sizeof(crcData));
				crcCalc = crc32(0,&data,sizeof(data));
				if(crcData == crcCalc)
				{
					int64_t timeT = systimer->gettime();
					
					//printf("dat %.3f\n",data);
				}
				i+=2+sizeof(data)+sizeof(crcData);
			}
		}
		return 0;
	}
}
