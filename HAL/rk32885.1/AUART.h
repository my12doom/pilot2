#pragma once
#include <HAL/Interface/IUART.h>
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
#include <sched.h>
#include <signal.h>
#define TX_BUFFER_SIZE 200
#define RX_BUFFER_SIZE 1536


namespace androidUAV
{
    class AUART : public HAL::IUART
	{
	    public:
		    AUART(const char*);
			~AUART();
			virtual int set_baudrate(int baudrate);
			virtual int peak(void *data, int max_count);
			virtual int write(const void *data, int count);
			virtual int flush();
			virtual int read(void *data, int max_count);
			virtual int readline(void *data, int max_count);
			virtual int available();

			static void *read_thread(void *p);
			void readProcess();
			int uartInit(int baudrate);
			static void *send_thread(void *p);
	    protected:
			int update_buffer();
			char buffer[20*1024];
			char dataTmp[100];
			fd_set rd;
			int buffer_pos;
	    private:
			int readThreadId;
			int sendThreadID;
			pthread_t tidp;
			pthread_mutex_t mutex;//Non-recursive lock
			bool exit;
			int uartFd;
			struct termios opt;
			int baudrate;
			int tx_start;
			int tx_end;
			int ongoing_tx_size;

			// RX buffer:
			char rx_buffer[RX_BUFFER_SIZE];		// circular buffer
			int start;
			int end;
			int ongoing_rx_start;
			int ongoing_rx_size;
			volatile bool rx_dma_running;

			char tx_buffer[TX_BUFFER_SIZE];
			struct sigaction sigio;
	};
	void signal_handler_IO(int status);
}
