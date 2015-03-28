#include "sensors.h"

namespace dev_v1
{
	
mpu6000::mpu6000(sensors::MPU6000 *mpu_device)
{
	mpu = mpu_device;
	m_healthy = true;
}
	

int mpu6000::read(devices::accelerometer_data *out)
{
	short data[7];
	if (mpu->read(data)<0)
		return -1;
	
	out->x = data[0] / 2048.0f;
	out->y = data[1] / 2048.0f;
	out->z = data[2] / 2048.0f;
	out->temperature = data[3] / 340.0f + 36.53f;
	
	return 0;
}

int mpu6000::read(devices::gyro_data *out)
{
	short data[7];
	if (mpu->read(data)<0)
		return -1;
	
	out->x = data[4] * 0.000266316f;		// to radians
	out->y = data[5] * 0.000266316f;
	out->z = data[6] * 0.000266316f;
	out->temperature = data[3] / 340.0f + 36.53f;
	
	return 0;
}

// return false if any error/waning
bool mpu6000::healthy()
{
	return m_healthy;
}
	
}