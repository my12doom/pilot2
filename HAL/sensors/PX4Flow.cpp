#include "PX4Flow.h"
#include <string.h>
#include <Protocol/common.h>



int PX4FLOW_ADDRESS = 0x84;
extern int I2C_speed;// = 5;
namespace sensors
{
	int PX4Flow::init(HAL::II2C *I2C, float cx, float cy)
	{
		this->I2C=I2C;
		this->cx = cx;
		this->cy = cy;


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

		if (out->qual < 0 || out->pixel_flow_x_sum< -1000 || out->pixel_flow_x_sum > 1000 || out->pixel_flow_y_sum <-1000 || out->pixel_flow_y_sum > 1000 || out->ground_distance < 0 || out->ground_distance > 10000 )
		{
			memset(out, 0, sizeof(px4flow_frame));
		}

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

	int PX4Flow::read(sensors::flow_data *out)
	{
		px4flow_frame frame= {0};
		int res = read_flow(&frame);
		if (res != 0)
			return res;

		out->x = frame.pixel_flow_x_sum * cx;
		out->y = frame.pixel_flow_y_sum * cy;
		out->quality = (float)frame.qual / 255;
		out->timestamp = 0;

		return 0;
	}
	int PX4Flow::read(float *out, int64_t *timestamp/* = NULL*/)
	{
		px4flow_frame frame= {0};
		int res = read_flow(&frame);
		if (res != 0)
			return res;
		
		*out = frame.ground_distance / 1000.0f * 1.15f;
		if (timestamp)
			*timestamp = systimer->gettime() - frame.sonar_timestamp * 1000;

		return 0;
	}

}
