#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/Interface/IBarometer.h>

namespace sensors
{
	typedef struct 
	{
		uint16_t Gain_stage1:2;
		uint16_t Gain_stage2:3;
		uint16_t Gain_Polarity:1;
		uint16_t Clk_divider:2;
		uint16_t A2D_Offset:3;
		uint16_t OSR_Pressure:3;
		uint16_t OSR_Temp:2;
	}NVM14_Define;

	class NX3A : public devices::IBarometer
	{
	public:
		NX3A();
		~NX3A();
		
		int init(HAL::II2C *i2c, uint8_t address);	// currently only 0x78(7bit)/0xF0(8bit) address is available

		// IBarometer
		virtual bool healthy();
		virtual int read(devices::baro_data *out);

	protected:
		HAL::II2C *i2c;
		uint8_t address;
		int temperature;// = 0;
		int pressure;// = 0;
		int32_t new_temperature;// = 0;
		int64_t last_temperature_time;// = 0;
		int64_t last_pressure_time;// = 0;

		int read(int *data);

		// stupid IIC commands
		int read_status();
		int read_nvm(uint8_t nvm_address, uint16_t *out);
		int write_command(uint8_t cmd, uint16_t cmd_data);
		int read_measurement_data(int32_t *out);
		int wait_chip(int timeout);	// return negative value if timed out.

		int id;
		int version;
		int t1,t2,t3,p1,p2,p3,p4,p5,p6,p7,p8;	// calibration coeff
		int Bshift, Tshift;
		NVM14_Define nvm14;
		bool _healthy;
	};
}