#include "AUART.h"
#include <HAL/rk32885.1/AGpio.h>
#include <unistd.h>

int speed_arrEuler[] = {B115200,B38400, B19200, B9600, B4800, B2400, B1200, B300, B38400, B19200, B9600, B4800, B2400, B1200, B300};
int name_arrEuler[] = {115200,38400, 19200, 9600, 4800, 2400, 1200, 300, 38400, 19200, 9600, 4800, 2400, 1200, 300};
volatile int cnt = 0;
namespace androidUAV
{
	AUART::AUART(const char* uartPath):start(0),end(0),tx_start(0),tx_end(0),ongoing_tx_size(0),ongoing_rx_start(0),ongoing_rx_size(0)
	{
		exit = false;
		readThreadId = 0;
		policy = get_thread_policy (&attr);
		//Only before pthread_create excuted ,can we set thread parameters
		set_thread_policy(&attr,SCHED_FIFO);
		//update thread pilicy after set thread policy
		policy = get_thread_policy (&attr);
		//set_priority(95);
		pthread_mutex_init(&mutex,NULL);
		if(!uartPath)
		{
			LOG2("androidUAV:Invalid 'uartPath' parameter\n" );
		}
		else
		{
			uartFd = open(uartPath,O_RDWR | O_NOCTTY | O_NDELAY );
			if(uartFd < 0)
			{
				LOG2("androidUAV:oepn uart dev error Path = %s\n",uartPath);
			}
		}
		uartInit(115200);
		exit = true;
		readThreadId = pthread_create(&tidp,NULL,read_thread,(void *)this);
	}
	AUART::~AUART()
	{
		exit = false;
		pthread_join(readThreadId,0);
		pthread_mutex_destroy(&mutex);
		close(uartFd);
	}
	int AUART::uartInit(int baudrate)
	{
		struct termios oldtio,newtio;
		int i;
		if((fcntl(uartFd,F_SETFL,0)) < 0)
		{
			LOG2("fcntl F_SETFL error\n");
		}
		tcgetattr(uartFd,&oldtio);
		newtio = oldtio;
		cfmakeraw(&newtio);

		for(i = 0; i < sizeof(speed_arrEuler) / sizeof(int); i++)
		{
			if(baudrate == name_arrEuler[i])
			{
				cfsetispeed(&newtio, speed_arrEuler[i]);
				cfsetospeed(&newtio, speed_arrEuler[i]);
				break;
			}
		}

		newtio.c_cflag |= (CLOCAL|CREAD);
		newtio.c_cflag &= ~CSIZE;
		newtio.c_cflag |= CS8;
		newtio.c_cflag &= ~PARENB;
		newtio.c_iflag &= ~INPCK;
		newtio.c_cc[VMIN]=0;
		newtio.c_cc[VTIME] = 0;
		tcflush(uartFd,TCIFLUSH);
		tcsetattr(uartFd,TCSANOW,&newtio);
		//fcntl(uartFd,F_SETFL,FNDELAY);
	}
	int AUART::update_buffer()
	{
		int ret = -1;
		int buffer_left = (sizeof(buffer)-buffer_pos)/2;
		if(buffer_left <= 0)
			return 0;
		//if(select(uartFd+1,&rd,NULL,NULL,NULL) < 0)
		{
			//printf("error\n");
		}
		//else
		{
			//pthread_mutex_lock(&mutex);
			ret = ::read(uartFd,buffer+buffer_pos,buffer_left);
			//pthread_mutex_unlock(&mutex);
			if(ret > 0)
			{
				buffer_pos+=ret;
				return ret;
			}
			else
			{

			}
		}
		return ret;
	}
	int AUART::set_baudrate(int baudrate)
	{
		int ret = 0;
		//printf("set %d\n",baudrate);
		ret = uartInit(baudrate);
		return ret;
	}
	int AUART::peak(void *data, int max_count)
	{
		update_buffer();
		if(max_count >= buffer_pos)
			max_count = buffer_pos;
		if(max_count <= 0)
			return 0;
		memcpy(data,buffer,max_count);

		return max_count;
	}
	int AUART::write(const void *data, int count)
	{
		int ret = -1;
		ret = ::write(uartFd,data,count);

		return ret;
	}
	int AUART::flush()
	{
		tcdrain(uartFd);
		//tcflush(uartFd,TCIOFLUSH);
		return 0;
	}
	int AUART::read(void *data, int max_count)
	{
		update_buffer();
		if(max_count >= buffer_pos)
			max_count = buffer_pos;
		if(max_count <= 0)
			return 0;
		memcpy(data,buffer,max_count);
		memmove(buffer,buffer+max_count,buffer_pos-max_count);
		buffer_pos-=max_count;
		return max_count;
	}
	int AUART::readline(void *data, int max_count)
	{
		update_buffer();
		int line_end = -1;
		for(int i=0;i<buffer_pos;i++)
		{
			if(buffer[i] == '\n')
			{
				line_end = i+1;
				break;
			}
		}
		if(line_end <= 0 || line_end > max_count)
			return 0;
		memcpy(data,buffer,line_end);
		memmove(buffer,buffer+line_end,buffer_pos-line_end);
		buffer_pos+=line_end;
		return line_end;
	}
	int AUART::available()
	{
		update_buffer();
		return buffer_pos;
	}
	void *AUART::read_thread(void *p)
	{
		AUART *_this = (AUART *)p;
		_this->readProcess();
		pthread_exit(0);
		return 0;
	}
	void AUART::readProcess()
	{
		int ret;

		FD_ZERO(&rd);
		FD_SET(uartFd,&rd);

		int cnt1 = 0;
		int cnt2 = 0;
		while(exit)
		{

			ret = update_buffer();
			usleep(80);
			/*if(ret > 0)
			{
				cnt+=ret;
				if(cnt>1998)
					printf("cnt = %d ret %d\n",cnt,ret);
			}*/

		}
	}
	void *AUART::send_thread(void *p)
	{
		char send[2] = {0x44};
		AUART *_this = (AUART *)p;
		while(1)
		{
			_this->write(send,1);
			usleep(100);
		}
	}
	int AUART::set_priority(int32_t priority)
	{
		//checkout policy ,only SCHED_FIFO SCHED_RR policies have a sched_priority value (1~99)
		if(policy == SCHED_OTHER)
			return -1;
		int32_t priority_min = 0;
		int32_t priority_max = 0;
		priority_min = sched_get_priority_min(policy);
		priority_max = sched_get_priority_max(policy);
		if(priority<=priority_min || priority>=priority_max)
			return -1;
		set_thread_priority(&attr,&schparam,priority);
		return 0;
	}
}
static int get_thread_policy(pthread_attr_t *attr)
{
	int policy;
	int rs = pthread_attr_getschedpolicy(attr, &policy);
	return policy;
}
static int get_thread_priority(pthread_attr_t *attr,struct sched_param *param)
{
	int rs = pthread_attr_getschedparam (attr, param);
	return param->__sched_priority;
}
static void set_thread_policy(pthread_attr_t *attr,int policy)
{
	int rs = pthread_attr_setschedpolicy (attr, policy);
	get_thread_policy (attr);
}
static void set_thread_priority(pthread_attr_t *attr,struct sched_param *param,int32_t priority)
{
	param->sched_priority = priority;
	pthread_attr_setschedparam(attr,param);
}
