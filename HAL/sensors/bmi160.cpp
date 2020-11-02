#include "bmi160.h"
#include <stdio.h>
#include <Protocol/common.h>
#include <string.h>

#define TRACE(...)

#define FAIL_RETURN(x) if((x)<0) return-1

#define BMI160_GYRO_CONFIG_ADDR              (0x42)
#define BMI160_GYRO_RANGE_ADDR               (0x43)
#define BMI160_ACCEL_CONFIG_ADDR             (0x40)
#define BMI160_ACCEL_RANGE_ADDR              (0x41)
#define BMI160_COMMAND_REG_ADDR              (0x7E)
#define BMI160_GYRO_RANGE_2000_DPS           (0x00)
#define BMI160_GYRO_NORMAL_MODE              (0x15)
#define BMI160_ACCEL_RANGE_16G               (0x0C)
#define BMI160_ACCEL_NORMAL_MODE             (0x11)

namespace sensors
{

BMI160::BMI160()
{
	spi = NULL;
}
int BMI160::read_reg(uint8_t reg, void *out, int count)
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

			spi->txrx2(tx_buf, rx_buf, count+1);
			memcpy(out, rx_buf+1, count);

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

	else
	{
		return -1;
	}

	return 0;
}

int BMI160::write_reg_core(uint8_t reg, uint8_t data)
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

	else
	{
		return -1;
	}

	return 0;
}

int BMI160::write_reg(uint8_t reg, uint8_t data)
{
	uint8_t read;
	for(int i=0; i<10; i++)
	{
		read = data - 1;
		write_reg_core(reg, data);
		systimer->delayms(5);
		read_reg(reg, &read, 1);

		if (read == data)
			return 0;
		//else
		//	printf("reg(%02x) error %02x/%02x\n", reg, read, data);
		
		systimer->delayms(10);
	}

	return -1;
}

int BMI160::init(HAL::ISPI *SPI, HAL::IGPIO *CS)
{
	this->spi = SPI;
	this->CS = CS;

	spi->set_speed(1000000);
	spi->set_mode(1, 1);
	CS->set_mode(HAL::MODE_OUT_PushPull);
	CS->write(true);

	// default axis
	accelerometer_axis_config(0, 1, 2, 1, 1, 1);
	gyro_axis_config(0, 1, 2, 1, 1, 1);
	

	// BMI160 register initialization
	TRACE("start BMI160\r\n");

	uint8_t who_am_i = 0;
	int res = read_reg(0, &who_am_i, 1);
	if (who_am_i != 0xd1)
	{
		TRACE("BMI160 not found\n");
		return -1;
	}	

	TRACE("BMI160 detected, WHO_AM_I=%x\n", who_am_i);

	// software reset
	write_reg_core(0x7e, 0xb6);
	systimer->delayms(15);

	// normal mode for faster register access
	write_reg_core(BMI160_COMMAND_REG_ADDR, BMI160_GYRO_NORMAL_MODE);
	systimer->delayms(80);
	
	// set gyro: 800hz ODR, OSR4 BW, 2000dps range, normal power.
    FAIL_RETURN(write_reg(BMI160_GYRO_CONFIG_ADDR, 0x0b));
    FAIL_RETURN(write_reg(BMI160_GYRO_RANGE_ADDR, BMI160_GYRO_RANGE_2000_DPS));
    write_reg_core(BMI160_COMMAND_REG_ADDR, BMI160_GYRO_NORMAL_MODE);
	systimer->delayms(80);

	// set accel: 800hz ODR, OSDR4 BW, 16G range, normal power.
    FAIL_RETURN(write_reg(BMI160_ACCEL_CONFIG_ADDR, 0x02b));
    FAIL_RETURN(write_reg(BMI160_ACCEL_RANGE_ADDR, BMI160_ACCEL_RANGE_16G));
	write_reg_core(BMI160_COMMAND_REG_ADDR, BMI160_ACCEL_NORMAL_MODE);	
	systimer->delayms(20);

	m_healthy = true;

	return 0;
}

// data[0 ~ 7] :
// accel_x, accel_y, accel_z, raw_temperature, gyro_x, gyro_y, gyro_z
int BMI160::read(short*data)
{
	int i;
	int result;
	
	if (spi)
	{
		spi->set_speed(10000000);
		spi->set_mode(1, 1);
	}
	
	result = read_reg(0x20, (uint8_t*)(data+0), 2);
	result = read_reg(0x0c, (uint8_t*)(&data[1]), 12);

	return result;
}

int BMI160::accelerometer_axis_config(int x, int y, int z, int negtivex, int negtivey, int negtivez)
{
	axis[0] = x+4;
	axis[1] = y+4;
	axis[2] = z+4;
	negtive[0] = negtivex;
	negtive[1] = negtivey;
	negtive[2] = negtivez;

	return 0;
}

int BMI160::gyro_axis_config(int x, int y, int z, int negtivex, int negtivey, int negtivez)
{
	axis[3] = x+1;
	axis[4] = y+1;
	axis[5] = z+1;
	negtive[3] = negtivex;
	negtive[4] = negtivey;
	negtive[5] = negtivez;

	return 0;
}


int BMI160::read(devices::accelerometer_data *out)
{
	out->x = data[axis[0]] * negtive[0] * G_in_ms2 / 2048.0f;
	out->y = data[axis[1]] * negtive[1] * G_in_ms2 / 2048.0f;
	out->z = data[axis[2]] * negtive[2] * G_in_ms2 / 2048.0f;
	out->temperature = data[0] * 0.00195f + 23.0f;
	
	return 0;
}

int BMI160::read(devices::gyro_data *out)
{
	if (read(data)<0)
		return -1;
	
	out->x = data[axis[3]] * negtive[3] * 0.00106530f;		// to radians
	out->y = data[axis[4]] * negtive[4] * 0.00106530f;
	out->z = data[axis[5]] * negtive[5] * 0.00106530f;
	out->temperature = data[0] * 0.00195f + 23.0f;
	
	return 0;
}


}
