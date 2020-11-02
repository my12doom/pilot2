#include "ADXL357.h"
#include <stdio.h>
#include <Protocol/common.h>
#include <string.h>

#define TRACE(...)

#define FAIL_RETURN(x) if((x)<0) return-1

namespace sensors
{

static void swap(void *buf, int size)
{
	char *p = (char*)buf;
	int i;
	for(i=0; i<size/2; i++)
	{
		char t = p[i];
		p[i] = p[size-1-i];
		p[size-1-i] = t;
	}
}

ADXL357::ADXL357()
{
	spi = NULL;
}
int ADXL357::read_reg(uint8_t reg, void *out, int count)
{
	int i;
	uint8_t *p = (uint8_t*)out;

	if (spi)
	{
		systimer->delayus(1);
		CS->write(false);
		systimer->delayus(1);

		uint8_t tx_buf[15] = {(reg << 1) | 1};
		uint8_t rx_buf[15];

		spi->txrx2(tx_buf, rx_buf, count + 1);
		memcpy(out, rx_buf+1, count);

		systimer->delayus(1);
		CS->write(true);
		systimer->delayus(1);
	}
	return 0;
}

int ADXL357::write_reg_core(uint8_t reg, uint8_t data)
{
	if (spi)
	{
		systimer->delayus(1);
		CS->write(false);
		systimer->delayus(1);

		spi->txrx(reg<<1);
		spi->txrx(data);

		systimer->delayus(1);
		CS->write(true);
		systimer->delayus(1);
	}

	return 0;
}

int ADXL357::write_reg(uint8_t reg, uint8_t data)
{
	uint8_t read;
	for(int i=0; i<10; i++)
	{
		read = data - 1;
		write_reg_core(reg, data);
		read_reg(reg, &read, 1);

		if (read == data)
			return 0;
		//else
		//	printf("reg(%02x) error %02x/%02x\n", reg, read, data);
		
		systimer->delayms(10);
	}

	return -1;
}

int ADXL357::init(HAL::ISPI *SPI, HAL::IGPIO *CS)
{	
	this->spi = SPI;
	this->CS = CS;

	spi->set_speed(8000000);
	spi->set_mode(0, 0);
	CS->set_mode(HAL::MODE_OUT_PushPull);
	CS->write(true);

	// default axis
	accelerometer_axis_config(0, 1, 2, 1, 1, 1);
	
	return init();
}

int ADXL357::init()
{
	uint8_t who_am_i = 0;
	int i;

	// ADXL357 register initialization
	TRACE("start ADXL357\r\n");
	uint8_t device_ids[4];
	read_reg(0, device_ids, 4);

	if (device_ids[0] != 0xad || device_ids[1] != 0x1d)
	{
		TRACE("ADXL357 not found\n");
		return -1;
	}

	TRACE("part %02d rev %d detected\n", device_ids[2], device_ids[3]);

	// software reset
	if (device_ids[3] >= 1)		// don't reset for revid <= 1, see datasheet page 40.
	{
		write_reg_core(0x2f, 0x52);
		systimer->delayms(10);
	}

	FAIL_RETURN(write_reg(0x2D, 0x00));		// power on
	FAIL_RETURN(write_reg(0x2C, 0x83));		// +-40g range, high speed spi
	FAIL_RETURN(write_reg(0x29, 1));
	
	systimer->delayms(10);

	m_healthy = true;

	return 0;
}

int ADXL357::accelerometer_axis_config(int x, int y, int z, int negtivex, int negtivey, int negtivez)
{
	axis[0] = x+1;
	axis[1] = y+1;
	axis[2] = z+1;
	negtive[0] = negtivex;
	negtive[1] = negtivey;
	negtive[2] = negtivez;

	return 0;
}

int ADXL357::read(devices::accelerometer_data *out)
{
	spi->set_speed(8000000);
	spi->set_mode(0, 0);

	uint8_t data[11];
	read_reg(0x08, data, 9);

	int16_t data16[3];
	for(int i=0; i<3; i++)
		data16[i] = (data[3*i] << 8) | data[1+3*i];

	out->x = data16[0] * negtive[0] * G_in_ms2 / 800.0f;
	out->y = data16[1] * negtive[1] * G_in_ms2 / 800.0f;
	out->z = data16[2] * negtive[2] * G_in_ms2 / 800.0f;

	int tmp = ((data[0]&0xf) << 8) | data[1];
	out->temperature = (tmp - 1885) / -9.05f + 25.0f;
	
	return 0;
}

}
