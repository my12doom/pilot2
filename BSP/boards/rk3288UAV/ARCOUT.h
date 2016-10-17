#pragma once
#include <HAL/Interface/IRCOUT.h>
#include <HAL/rk32885.1/ALog.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <linux/types.h>
#include <termios.h>
#include <errno.h>
#include <pthread.h>
#define MAX_CHANNEL 4
#define PWMNUM 4
#define PWMPERIODNS 2100000
namespace androidUAV
{
    typedef struct pwm_dat
    {
	    int pwm_id;
		int duty_ns;
		int period_ns;
    }pwm_data;
    enum UAVPWMIDS
	{
		UAVPWM0 = 0,
		UAVPWM1 = 1,
		UAVPWM2 = 2,
		UAVPWM3 = 3,
	};
	enum UAVPWMCMD
	{
		PWMCONFIG = 0,
		PWMENABLE = 1,
		PWMDISABLE = 2,
		PWMREGISTE = 3,
	};
	class ARCOUT : public HAL::IRCOUT
	{
	    public:
		    ARCOUT(const char*);
			~ARCOUT();
			// total channel count
			virtual int get_channel_count();

			// return num channel written
			// generate an error if index overrun/underrun, and won't update any channel
			// return negative value to indicate an error
			virtual int write(const int16_t *data, int start_channel, int count);

			// return num channel read
			// return any possible read if index overrun
			// return negative value to indicate an error
			virtual int read(int16_t *out, int start_channel, int max_count);
			int register_pwm(pwm_data *data);
			int config_pwm(pwm_data *data);
			int enable_pwm(pwm_data *data);
			int disable_pwm(pwm_data *data);
	    protected:
			int pwmFd;
			int16_t channel_datas[MAX_CHANNEL];
			pwm_data pwms[PWMNUM];
	};
}
