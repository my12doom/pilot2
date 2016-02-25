#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>

namespace devices
{
	class PCA963x : public devices::ILED, public devices::IRGBLED
	{
	public:
		PCA963x();
		~PCA963x(){}
		
		int init(HAL::II2C *i2c, uint8_t address);

		
		int read_reg(uint8_t reg, void *out, int count);
		int write_reg_core(uint8_t reg, uint8_t data);
		int write_reg(uint8_t reg, uint8_t data);

		// return false if any error/waning
		virtual bool healthy(){return m_healthy;}

		// ILED
		virtual void on();
		virtual void off();
		virtual void toggle();
		virtual bool get(){return false;}
		
		// IRGBLED
		virtual int write(float R, float G, float B);
	protected:

		int init();

		HAL::II2C *i2c;
		uint8_t address;
		bool m_healthy;
	};
}