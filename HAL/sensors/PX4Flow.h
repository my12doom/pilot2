#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/Interface/IFlow.h>
#include <HAL/Interface/IRangeFinder.h>

namespace sensors
{
	typedef struct px4flow_frame
	{
		uint16_t frame_count;// counts created I2C frames [#frames]
		int16_t pixel_flow_x_sum;// latest x flow measurement in pixels*10 [pixels]
		int16_t pixel_flow_y_sum;// latest y flow measurement in pixels*10 [pixels]
		int16_t flow_comp_m_x;// x velocity*1000 [meters/sec]
		int16_t flow_comp_m_y;// y velocity*1000 [meters/sec]
		int16_t qual;// Optical flow quality / confidence [0: bad, 255: maximum quality]
		uint16_t cmos_version;
		int16_t gyro_y_rate; // latest gyro y rate [rad/sec]
		int16_t gyro_z_rate; // latest gyro z rate [rad/sec]
		uint8_t gyro_range; // gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec] 
		uint8_t sonar_timestamp;// time since last sonar update [milliseconds]
		int16_t ground_distance;// Ground distance in meters*1000 [meters]. Positive value: distance known. Negative value: Unknown distance
	} px4flow_frame;

	typedef struct px4flow_integral_frame
	{
		uint16_t frame_count_since_last_readout;//number of flow measurements since last I2C readout [#frames]
		int16_t pixel_flow_x_integral;//accumulated flow in radians*10000 around x axis since last I2C readout [rad*10000]
		int16_t pixel_flow_y_integral;//accumulated flow in radians*10000 around y axis since last I2C readout [rad*10000]
		int16_t gyro_x_rate_integral;//accumulated gyro x rates in radians*10000 since last I2C readout [rad*10000] 
		int16_t gyro_y_rate_integral;//accumulated gyro y rates in radians*10000 since last I2C readout [rad*10000] 
		int16_t gyro_z_rate_integral;//accumulated gyro z rates in radians*10000 since last I2C readout [rad*10000] 
		uint32_t integration_timespan;//accumulation timespan in microseconds since last I2C readout [microseconds]
		uint32_t sonar_timestamp;// time since last sonar update [microseconds]
		int16_t ground_distance;// Ground distance in meters*1000 [meters*1000]
		int16_t gyro_temperature;// Temperature * 100 in centi-degrees Celsius [degcelsius*100]
		uint8_t quality;// averaged quality of accumulated flow values [0:bad quality;255: max quality]
	} px4flow_integral_frame;

	/*enum px4flow_frame_type
	{
		use_one_shot_frame = 0,
		use_integral_frame = 1,
	};*/


	class PX4Flow :public IFlow, devices::IRangeFinder
	{	
	private:
		HAL::II2C *I2C;
		float cx, cy;		// fov calibration, default values came from HDF-ov7675 offline calibration.

	public:
		PX4Flow(){}
		~PX4Flow(){}
		virtual int init(HAL::II2C *I2C, float cx = 1.0f / 28.0f * 100 * 3.1415926f / 180, float cy = 1.0f / 28.0f * 100 * 3.1415926f / 180);
		// return false if any error/waning
		virtual bool healthy();

		// IFlow
		virtual int read_flow(px4flow_frame *out);
		virtual int read_integral(px4flow_integral_frame *out);
		virtual int read(sensors::flow_data *out);

		// IRangeFinder
		virtual int trigger(){return 0;}		// ignore trigger() since we don't have that I2C command
		virtual int read(float *out, int64_t *timestamp = NULL);

	};
}