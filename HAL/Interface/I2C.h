#pragma once

#include <stdint.h>
#include "GPIO.h"
#include "SysTimer.h"

namespace HAL
{
	class I2C
	{
	public:
		virtual int set_speed(int speed) = 0;		// speed in hz
		virtual int read_reg(uint8_t SlaveAddress, uint8_t startRegister, uint8_t *out){return read_regs(SlaveAddress, startRegister, out);}
		virtual int read_regs(uint8_t SlaveAddress, uint8_t startRegister, uint8_t*out, int count) = 0;
		virtual int write_reg(uint8_t SlaveAddress, uint8_t Register, uint8_t data){return write_regs(SlaveAddress, Register, &data, 1);}
		virtual int write_regs(uint8_t SlaveAddress, uint8_t startRegister, const uint8_t*data, int count) = 0;
	};

	class I2C_SW : public I2C
	{
	public:
		I2C_SW(GPIO *SCL, GPIO *SDA);
		~I2C_SW();
		int set_speed(int speed);		// speed in hz
		int read_regs(uint8_t SlaveAddress, uint8_t startRegister, uint8_t*out, int count);
		int write_regs(uint8_t SlaveAddress, uint8_t startRegister, const uint8_t*data, int count);
	protected:
		GPIO *m_SDA;
		GPIO *m_SCL;
	};
}