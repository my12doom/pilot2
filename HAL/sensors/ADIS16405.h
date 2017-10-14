#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>

namespace sensors
{
class ADIS16405 : public devices::IGyro, public devices::IAccelerometer
{
public:
	ADIS16405(){}
	~ADIS16405(){}
		
	int init(HAL::IGPIO *sclk, HAL::IGPIO *miso, HAL::IGPIO *mosi, HAL::IGPIO *cs);
		
	// IAccelerometer
	virtual int read(devices::accelerometer_data *out);
	int accelerometer_axis_config(int x, int y, int z, int negtivex, int negtivey, int negtivez);

	// IGyro
	virtual int read(devices::gyro_data *out);
	int gyro_axis_config(int x, int y, int z, int negtivex, int negtivey, int negtivez);

	// return false if any error/waning
	virtual bool healthy(){return m_healthy;}
		
protected:
	HAL::IGPIO *sclk;
	HAL::IGPIO *miso;
	HAL::IGPIO *mosi;
	HAL::IGPIO *cs;
	bool m_healthy;
	int16_t data[12];


	int adis16405_init(void);
	int adis16405_read_register(unsigned char registerAddress, unsigned short *out);
	int adis16405_write_register(unsigned char registerAddress, unsigned short new_value);
	int adis16405_burst_read(int16_t *out, int count = 0);
	int16_t ReadFromADIS16405ViaSpi(unsigned char RegisterAddress);
	uint16_t spi_tx_rx16_burst(uint16_t tx);
	uint16_t spi_tx_rx16(uint16_t tx);


};
}
