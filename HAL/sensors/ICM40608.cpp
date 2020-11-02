#include "ICM40608.h"
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

ICM40608::ICM40608()
{
	i2c = NULL;
	spi = NULL;
}
int ICM40608::read_reg(uint8_t reg, void *out, int count)
{
	int i;
	uint8_t *p = (uint8_t*)out;

	if (spi)
	{
		if(count > 1)
		{
			systimer->delayus(1);
			CS->write(false);
			systimer->delayus(1);

			uint8_t tx_buf[15] = {(reg&0x7f) | 0x80};
			uint8_t rx_buf[15];

			spi->txrx2(tx_buf, rx_buf, (count>14) ? 15 : (count+1));
			memcpy(out, rx_buf+1, count>15 ? 15 : count);

			systimer->delayus(1);
			CS->write(true);
			systimer->delayus(1);
		}
		else
		{
			systimer->delayus(1);
			CS->write(false);
			systimer->delayus(1);

			spi->txrx((reg&0x7f) | 0x80);
			p[0] = spi->txrx(0);
			systimer->delayus(1);
			CS->write(true);
			systimer->delayus(1);
		}
	}

	else if (i2c)
	{
		return i2c->read_regs(address, reg, (uint8_t*)out, count);
	}
	else
	{
		return -1;
	}

	return 0;
}

int ICM40608::write_reg_core(uint8_t reg, uint8_t data)
{
	if (spi)
	{
		systimer->delayus(1);
		CS->write(false);
		systimer->delayus(1);

		spi->txrx(reg);
		spi->txrx(data);

		systimer->delayus(1);
		CS->write(true);
		systimer->delayus(1);
	}

	else if (i2c)
	{
		return i2c->write_reg(address, reg, data);
	}

	else
	{
		return -1;
	}

	return 0;
}

int ICM40608::write_reg(uint8_t reg, uint8_t data)
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

int ICM40608::init(HAL::ISPI *SPI, HAL::IGPIO *CS)
{	
	this->spi = SPI;
	this->CS = CS;
	this->i2c = NULL;
	this->address = 0;

	// ICM40608 can handle 1mhz max for configuration register accessing
	// 20mhz for data output and interrupt accesss
	spi->set_speed(1000000);
	spi->set_mode(1, 1);
	CS->set_mode(HAL::MODE_OUT_PushPull);
	CS->write(true);

	// default axis
	accelerometer_axis_config(0, 1, 2, 1, 1, 1);
	gyro_axis_config(0, 1, 2, 1, 1, 1);
	
	return init();
}

int ICM40608::init(HAL::II2C *i2c, uint8_t address)
{	
	this->spi = NULL;
	this->CS = NULL;
	this->i2c = i2c;
	this->address = address;
	i2c->set_speed(2);

	return init();
}

int ICM40608::init()
{
	uint8_t who_am_i = 0;
	int i;

	// ICM40608 register initialization
	TRACE("start ICM40608\r\n");

	int res = read_reg(0x75, &who_am_i, 1);
	if (who_am_i != 0x39 && who_am_i != 0x47)
	{
		TRACE("ICM40608 not found\n");
		return -1;
	}

	TRACE("ICM%d detected, WHO_AM_I=%x\n", who_am_i == 0x47 ? 42688 : 40608, who_am_i);

	// software reset
	write_reg_core(0x11, 0x01);
	systimer->delayms(10);

	FAIL_RETURN(write_reg(0x4E, 0x1F));		// enable gyro & acc.
	FAIL_RETURN(write_reg(0x4F, 0x03));		// full scale : 2000 degree/s, 8k ODR, ~65.5 LSB/degree/s
	FAIL_RETURN(write_reg(0x50, 0x03));		// full scale : 16g, 8k ODR, 2048 = 1g
 

	TRACE("ICM4xxxx initialized\n", who_am_i);
	
	systimer->delayms(10);

	m_healthy = true;

	return 0;
}

// data[0 ~ 7] :
// accel_x, accel_y, accel_z, raw_temperature, gyro_x, gyro_y, gyro_z
int ICM40608::read(short*data)
{
	int i;
	int result;
	
	if (spi)
	{
		spi->set_speed(20000000);
		spi->set_mode(1, 1);
	}
	
	result = read_reg(0x1D, (uint8_t*)data, 14);
	for(i=0; i<7; i++)
		swap((uint8_t*)&data[i], 2);

	return result;
}

int ICM40608::accelerometer_axis_config(int x, int y, int z, int negtivex, int negtivey, int negtivez)
{
	axis[0] = x+1;
	axis[1] = y+1;
	axis[2] = z+1;
	negtive[0] = negtivex;
	negtive[1] = negtivey;
	negtive[2] = negtivez;

	return 0;
}

int ICM40608::gyro_axis_config(int x, int y, int z, int negtivex, int negtivey, int negtivez)
{
	axis[3] = x+4;
	axis[4] = y+4;
	axis[5] = z+4;
	negtive[3] = negtivex;
	negtive[4] = negtivey;
	negtive[5] = negtivez;

	return 0;
}


int ICM40608::read(devices::accelerometer_data *out)
{
	out->x = data[axis[0]] * negtive[0] * G_in_ms2 / 2048.0f;
	out->y = data[axis[1]] * negtive[1] * G_in_ms2 / 2048.0f;
	out->z = data[axis[2]] * negtive[2] * G_in_ms2 / 2048.0f;
	out->temperature = data[0] / 132.48f + 25.0f;
	
	return 0;
}

int ICM40608::read(devices::gyro_data *out)
{
	if (read(data)<0)
		return -1;
	
	out->x = data[axis[3]] * negtive[3] * 0.00106530f;		// to radians
	out->y = data[axis[4]] * negtive[4] * 0.00106530f;
	out->z = data[axis[5]] * negtive[5] * 0.00106530f;
	out->temperature = data[0] / 132.48f + 25.0f;
	
	return 0;
}


}
