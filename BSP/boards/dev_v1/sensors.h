#pragma once

#include <stdint.h>
#include <BSP/devices/sensors/MPU6000.h>
#include <BSP/devices/IAccelerometer.h>
#include <BSP/devices/IGyro.h>
#include <BSP/devices/IMagnetometer.h>

namespace dev_v1
{
	// channel index starts from 0
	class mpu6000 : public devices::IAccelerometer, public devices::IGyro
	{
	public:
		mpu6000(sensors::MPU6000 *mpu_device);
		~mpu6000(){}
		
		virtual int read(devices::accelerometer_data *out);
		virtual int read(devices::gyro_data *out);

		// return false if any error/waning
		virtual bool healthy();
	protected:
		sensors::MPU6000 *mpu;
		bool m_healthy;
	};
}
