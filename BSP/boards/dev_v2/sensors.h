#pragma once

#include <stdint.h>
#include <BSP/devices/sensors/MPU6000.h>
#include <BSP/devices/sensors/HMC5983SPI.h>
#include <BSP/devices/sensors/MS5611_SPI.h>
#include <BSP/devices/IAccelerometer.h>
#include <BSP/devices/IGyro.h>
#include <BSP/devices/IMagnetometer.h>
#include <BSP/devices/IBarometer.h>

namespace dev_v2
{
	class mpu6000res : public devices::IAccelerometer, public devices::IGyro
	{
	public:
		mpu6000res(sensors::MPU6000 *mpu_device);
		~mpu6000res(){}
		
		virtual int read(devices::accelerometer_data *out);
		virtual int read(devices::gyro_data *out);

		// return false if any error/waning
		virtual bool healthy();
	protected:
		sensors::MPU6000 *mpu;
		bool m_healthy;
	};

	class MS5611res : public devices::IBarometer
	{
	public:
		MS5611res(sensors::MS5611_SPI *ms5611_device);
		~MS5611res(){}

		virtual int read(devices::baro_data *out);
		virtual bool healthy();

	protected:
		sensors::MS5611_SPI *ms5611;
		bool m_healthy;
	};

	class HMC5983res : public devices::IMagnetometer
	{
	public:
		HMC5983res(sensors::HMC5983 *hmc5983);
		~HMC5983res(){}

		virtual int read(devices::mag_data *out);
		virtual bool healthy();

	protected:
		sensors::HMC5983 *hmc5983;
		bool m_healthy;
	};
}
