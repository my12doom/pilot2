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


