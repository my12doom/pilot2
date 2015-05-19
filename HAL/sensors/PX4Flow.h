#pragma once

#include <stdint.h>
#include <Interfaces.h>
#include <HAL\devices\IFlow.h>
using namespace HAL;
namespace sensors
{
	class PX4Flow :public IFlow
	{	
	private:
		HAL::II2C *I2C;

	public:
		PX4Flow(){}
		~PX4Flow(){}
		virtual int init(HAL::II2C *I2C);
		virtual int read_flow(px4flow_frame *out);
		virtual int read_integral(px4flow_integral_frame *out);
		// return false if any error/waning
		virtual bool healthy();
	};
}