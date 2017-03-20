#include "ARCOUT.h"
namespace androidUAV
{
    ARCOUT::ARCOUT(const char*pwmPath):pwmFd(0)
	{
		memset(pwms,0,sizeof(pwms));
		memset(channel_datas,0,sizeof(channel_datas));
		if(!pwmPath)
		{
			LOG2("androidUAV:invalid parameter in ARCOUT constructor\n");
		}
		else
		{
			pwmFd = open(pwmPath,O_RDWR);
			if(pwmFd < 0)
			{
				LOG2("androidUAV:can not open %s device\n",pwmPath);
			}
			//register pwm here
			for(int i=0;i<PWMNUM;i++)
			{
				pwms[i].pwm_id = i;
				pwms[i].duty_ns = 0;
				pwms[i].period_ns = PWMPERIODNS;
				register_pwm(&pwms[i]);
			}
			//config pwm here
			for(int i=0;i<PWMNUM;i++)
			{
				pwms[i].pwm_id = i;
				pwms[i].duty_ns = 0;
				pwms[i].period_ns = PWMPERIODNS;
				config_pwm(&pwms[i]);
			}
			//enable pwm here
			for(int i=0;i<PWMNUM;i++)
			{
				pwms[i].pwm_id = i;
				pwms[i].duty_ns = 0;
				pwms[i].period_ns = PWMPERIODNS;
				enable_pwm(&pwms[i]);
			}
		}
	}
	ARCOUT::~ARCOUT()
	{
		close(pwmFd);
	}
	int ARCOUT::get_channel_count()
	{
		return MAX_CHANNEL;
	}

	// return num channel written
	// generate an error if index overrun/underrun, and won't update any channel
	// return negative value to indicate an error
	int ARCOUT::write(const int16_t *data, int start_channel, int count)
	{
		if(start_channel<0 || start_channel + count > MAX_CHANNEL)
			return -1;
		for(int i=0;i<count;i++)
		{
			pwms[i+start_channel].pwm_id = i+start_channel;
			pwms[i+start_channel].duty_ns = data[i]*1000;
			channel_datas[i+start_channel] = data[i];
			config_pwm(&pwms[i+start_channel]);
		}
		return 0;
	}

	// return num channel read
	// return any possible read if index overrun
	// return negative value to indicate an error
	int ARCOUT::read(int16_t *out, int start_channel, int max_count)
	{
		if(start_channel<0 || start_channel >= MAX_CHANNEL)
			return 0;
		int count = MAX_CHANNEL - start_channel;
		if(count > max_count)
			count = max_count;
		for(int i=0;i<count;i++)
			out[i] = channel_datas[i+start_channel];
		return count;
	}
	int ARCOUT::register_pwm(pwm_data *data)
	{
		int ret;
		if(!data)
			return -1;
		if((ret = ioctl(pwmFd,PWMREGISTE,data) < 0))
		{
			return ret;
		}
		return 0;
	}
	int ARCOUT::config_pwm(pwm_data *data)
	{
		if(!data)
			return -1;
		if((ioctl(pwmFd,PWMCONFIG,data) < 0))
		{
			//LOG2("error config\n");
			return -1;
		}
		return 0;
	}

	int ARCOUT::enable_pwm(pwm_data *data)
	{
		if(!data)
			return -1;
		if((ioctl(pwmFd,PWMENABLE,data) < 0))
		{
			LOG2("error enable\n");
			return -1;
		}
		return 0;
	}
	int ARCOUT::disable_pwm(pwm_data *data)
	{
		if(!data)
			return -1;
		if((ioctl(pwmFd,PWMDISABLE,data) < 0))
		{
			LOG2("error disable\n");
			return -1;
		}
		return 0;
	}
}
