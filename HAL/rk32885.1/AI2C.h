#pragma once
#include <HAL/Interface/II2C.h>
#include <HAL/rk32885.1/ALog.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <stdint.h>

#define I2CDATANUM 14
#define DEFSPEED 400000
typedef struct i2c_io
{
    char reg;
    char data[I2CDATANUM];
    uint8_t SlaveAddress;
    int cmd;
    int count;
    uint32_t scal_rate;
}i2c_io_t;

namespace androidUAV
{
	enum I2CCTL
	{
		I2CSEND = 0,
		I2CRECV = 1,
	};
	class AI2C : public HAL::II2C
	{
		public:
			AI2C(const char*);
			~AI2C();
			virtual int set_speed(int speed);		// speed in hz
			virtual int read_regs(uint8_t SlaveAddress, uint8_t startRegister, uint8_t*out, int count);
			virtual int write_regs(uint8_t SlaveAddress, uint8_t startRegister, const uint8_t*data, int count);
			// low level control
			virtual int start(){return 0;};		// return -1 if bus busy or any error.
			virtual int stop(){return 0;};			// return -1 on any error.
			virtual int send_ack(){return 0;};		// return -1 on any error.
			virtual int send_nak(){return 0;};		// return -1 on any error.
			virtual int wait_ack(){return 0;};		// return 0 on success, -1 on any error, 1 if a nak received.
			virtual int txrx(uint8_t tx = 0xff){return 0;};	// send and receive a byte, to receive from slave, send 0xff.
		protected:
			int i2cFd;
			uint32_t speed;
	};
}
