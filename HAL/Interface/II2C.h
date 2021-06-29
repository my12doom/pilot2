#pragma once

#include <stdint.h>
#include "IGPIO.h"
#include "ISysTimer.h"

namespace HAL
{
	class II2C
	{
	public:
		virtual int set_speed(int speed) = 0;		// speed in hz
		virtual int read_reg(uint8_t SlaveAddress, uint8_t startRegister, uint8_t *out){return read_regs(SlaveAddress, startRegister, out, 1);}
		virtual int read_regs(uint8_t SlaveAddress, uint8_t startRegister, uint8_t*out, int count) = 0;
		virtual int write_reg(uint8_t SlaveAddress, uint8_t Register, uint8_t data){return write_regs(SlaveAddress, Register, &data, 1);}
		virtual int write_regs(uint8_t SlaveAddress, uint8_t startRegister, const uint8_t*data, int count) = 0;

		// low level control
		virtual int start()=0;		// return -1 if bus busy or any error.
		virtual int stop()=0;			// return -1 on any error.
		virtual int send_ack()=0;		// return -1 on any error.
		virtual int send_nak()=0;		// return -1 on any error.
		virtual int wait_ack()=0;		// return 0 on success, -1 on any error, 1 if a nak received.
		virtual int tx(uint8_t tx)=0;	// send a byte
		virtual uint8_t rx()=0;			// receive a byte

		// bus reset
		virtual int reset_bus(){return -2;}// return -1 if bus busy or any error, -2 not supported, 0 on success.
	};

	class I2C_SW : public II2C
	{
	public:
		I2C_SW();
		I2C_SW(IGPIO *SCL, IGPIO *SDA);
		~I2C_SW();
		int init(IGPIO *SCL, IGPIO *SDA);
		virtual int set_speed(int speed);		// speed in hz
		virtual int read_regs(uint8_t SlaveAddress, uint8_t startRegister, uint8_t*out, int count);
		virtual int write_regs(uint8_t SlaveAddress, uint8_t startRegister, const uint8_t*data, int count);

		// low level control
		virtual int start();		// return -1 if bus busy or any error.
		virtual int stop();			// return -1 on any error.
		virtual int send_ack();		// return -1 on any error.
		virtual int send_nak();		// return -1 on any error.
		virtual int wait_ack();		// return 0 on success, -1 on any error, 1 if a nak received.
		virtual int tx(uint8_t tx);	// send a byte
		virtual uint8_t rx();		// receive a byte

		// bus reset
		virtual int reset_bus();		// return -1 if bus busy or any error, -2 not supported, 0 on success.

	protected:
		int m_speed_tick;
		IGPIO *m_SDA;
		IGPIO *m_SCL;
	
		bool SDA_STATE();
		bool SCL_STATE();
		void I2C_Delay();
		uint8_t I2C_Start();
		void I2C_Stop();
		void I2C_SendAck();
		void I2C_SendNoAck();
		uint8_t I2C_WaitAck(void);
		uint8_t I2C_SCLHigh(void);
		void I2C_SendByte(uint8_t Data);
		uint8_t I2C_ReceiveByte(void);


	};
}


