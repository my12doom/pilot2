#include "PX4Flow.h"
int PX4FLOW_ADDRESS = 0x84;
extern int I2C_speed;// = 5;
namespace sensors
{
	int PX4Flow::init(HAL::II2C *I2C)
	{
		this->I2C=I2C;
		px4flow_frame frame= {0};
		I2C->set_speed(8);
		int res = read_flow(&frame);
		//read integral:
		//px4flow_integral_frame integral_frame= {};
		//read_integral(&integral_frame);
		return (res == 0) ? 0 : -1;
	}
	int PX4Flow::read_flow(px4flow_frame *out)
	{
		int result;
		result=I2C->read_regs(PX4FLOW_ADDRESS,0,(uint8_t*)out,sizeof(px4flow_frame));
		return result;
	}
	int PX4Flow::read_integral(px4flow_integral_frame *out)
	{
		//Note: use this u should change this I2C start address
		//I2C->read_regs(PX4FLOW_ADDRESS,0,(uint8_t*)out,sizeof(px4flow_frame));
		return -1;
	}
	// return false if any error/waning
	bool PX4Flow::healthy()
	{
		return init(I2C) == 0;
	}
}
/*

int init_px4flow(void)
{
	px4_frame frame = {0};
	read_px4flow(&frame);
	read_px4flow(&frame);

	return frame.frame_count > 0 ? 0 : -1;
}

int read_px4flow(px4_frame *frame)
{
	volatile int speed = I2C_speed;
	I2C_speed = 10;
	int o = I2C_ReadReg(PX4FLOW_ADDRESS, 0, (uint8_t*)frame, sizeof(px4_frame));
	I2C_speed = speed;
	return o;
}

int read_px4flow_integral(px4_integral_frame *frame)
{
	volatile int speed = I2C_speed;
	I2C_speed = 10;
	int o = I2C_ReadReg(PX4FLOW_ADDRESS, 0, (uint8_t*)frame, sizeof(px4_integral_frame));
	I2C_speed = speed;
	return o;
}

int check_px4flow(void)
{
	return init_px4flow();
}
*/