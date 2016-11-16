#include "AI2C.h"

namespace androidUAV
{
	AI2C::AI2C(const char* i2cPath):i2cFd(0),speed(DEFSPEED)
	{
		if(!i2cPath)
		{
			LOG2("androidUAV:input valid\n");
		}
		else
		{
			if((i2cFd = ::open(i2cPath,O_RDWR)) < 0)
			{
				LOG2("androidUAV:Create i2c dev error\n");
			}
		}
	}
	AI2C::~AI2C()
	{
		if(i2cFd > 0)
		{
			close(i2cFd);
		}
	}
	int AI2C::set_speed(int speed)
	{
		this->speed = DEFSPEED;
		return 0;
	}
	
	int AI2C::read_regs(uint8_t SlaveAddress, uint8_t startRegister, uint8_t*out, int count)
	{
		i2c_io_t read;
		int ret;
		
		memset(&read,0,sizeof(i2c_io_t));
		read.reg = startRegister;
		read.SlaveAddress = SlaveAddress;
		read.cmd = I2CRECV;
		read.count = count;
		read.scal_rate = DEFSPEED;
		ret = ioctl(i2cFd,0,&read);
		if(ret > 0)
		{
			memcpy(out,read.data,count);
			return ret;
		}
		else
		{
			LOG2("read error ret = %d\n",ret);
		}
		return -1;
	}
	int AI2C::write_regs(uint8_t SlaveAddress, uint8_t startRegister, const uint8_t*data, int count)
	{
		i2c_io_t write;
		int ret;
		memset(&write,0,sizeof(i2c_io_t));
		write.reg = startRegister;
		write.SlaveAddress = SlaveAddress;
		memcpy(write.data,data,count);
		write.cmd = I2CSEND;
		write.count = count;
		write.scal_rate = DEFSPEED;
		ret = ioctl(i2cFd,0,&write);

		return ret;
	}

}
