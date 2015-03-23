#include "px4flow.h"
#include "../common/I2C.h"

int PX4FLOW_ADDRESS = 0x84;
extern int I2C_speed;// = 5;


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
