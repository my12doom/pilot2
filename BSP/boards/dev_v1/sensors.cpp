#include "sensors.h"
#include <math.h>

namespace dev_v1
{
	
mpu6000res::mpu6000res(sensors::MPU6000 *mpu_device)
{
	mpu = mpu_device;
	m_healthy = true;
}
	

int mpu6000res::read(devices::accelerometer_data *out)
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

int mpu6000res::read(devices::gyro_data *out)
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
bool mpu6000res::healthy()
{
	return m_healthy;
}


MS5611res::MS5611res(sensors::MS5611_SPI *ms5611_device)
{
	ms5611 = ms5611_device;
	m_healthy = true;
}
	

int MS5611res::read(devices::baro_data *out)
{
	int data[2];
	int res = ms5611->read(data);
	
	out->pressure = data[0];
	out->temperature = data[1] / 100.0f;

	return 0;
}

// return false if any error/waning
bool MS5611res::healthy()
{
	return m_healthy;
}

HMC5983res::HMC5983res(sensors::HMC5983 *hmc5983)
{
	this->hmc5983 = hmc5983;
	m_healthy = true;
}
	

int HMC5983res::read(devices::mag_data *out)
{
	short data[3];
	if (hmc5983->read(data)<0)
		return -1;
	
	out->x = data[0] * 0.92f;		// to milli-gauss
	out->y = data[1] * 0.92f;
	out->z = data[2] * 0.92f;
	out->temperature = NAN;
	
	return 0;
}

// return false if any error/waning
bool HMC5983res::healthy()
{
	return m_healthy;
}


	
}