#include "MPU6000.h"
#include <stdio.h>
#include <Protocol/common.h>

// Gyro and accelerator registers
#define	SMPLRT_DIV		0x19
#define	MPU9250_CONFIG	0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define EXT_SENS_DATA	0x49
#define	USER_CTRL		0x6A
#define	PWR_MGMT_1		0x6B
#define	PWR_MGMT_2		0x6C
#define	WHO_AM_I		0x75

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

MPU6000::MPU6000()
{
	i2c = NULL;
	spi = NULL;
}
int MPU6000::read_reg(uint8_t reg, void *out, int count)
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

                spi->txrx2(tx_buf, rx_buf, 15);
                memcpy(out, rx_buf+1, 14);


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
                for(i=0; i<count; i++)
                        p[i] = spi->txrx(0);

                //spi->txrx((reg&0x7f) | 0x80);
                //spi->txrx2(NULL,(uint8_t*)out,14);
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

int MPU6000::write_reg_core(uint8_t reg, uint8_t data)
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

int MPU6000::write_reg(uint8_t reg, uint8_t data)
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

int MPU6000::init(HAL::ISPI *SPI, HAL::IGPIO *CS)
{	
	this->spi = SPI;
	this->CS = CS;
	this->i2c = NULL;
	this->address = 0;

	// MPU6000 can handle 1mhz max for configuration register accessing
	// 20mhz for data output and interrupt accesss
        spi->set_speed(1000000);
	spi->set_mode(1, 1);
	CS->set_mode(HAL::MODE_OUT_PushPull);
	CS->write(true);
	
	return init();
}

int MPU6000::init(HAL::II2C *i2c, uint8_t address)
{	
	this->spi = NULL;
	this->CS = NULL;
	this->i2c = i2c;
	this->address = address;
	i2c->set_speed(10);

	return init();
}

int MPU6000::init()
{
	uint8_t who_am_i = 0;
	int i;

	// MPU6000 register initialization
	TRACE("start MPU6000\r\n");
	write_reg_core(PWR_MGMT_1, 0x80);
	systimer->delayms(10);
	write_reg(USER_CTRL, 0x10);
	FAIL_RETURN(write_reg(PWR_MGMT_1, 0x00));
	FAIL_RETURN(write_reg(PWR_MGMT_2, 0x00));
	FAIL_RETURN(write_reg(SMPLRT_DIV, 0x00));
	FAIL_RETURN(write_reg(MPU9250_CONFIG, 1));
	FAIL_RETURN(write_reg(GYRO_CONFIG, 3 << 3));		// full scale : 2000 degree/s, ~65.5 LSB/degree/s
	FAIL_RETURN(write_reg(ACCEL_CONFIG, 0x18));			// full scale : 16g, 2048 = 1g
	FAIL_RETURN(write_reg(0x37, 0x10));
	FAIL_RETURN(write_reg(0x38, 0x01));
	
	int res = read_reg(WHO_AM_I, &who_am_i, 1);
	TRACE("MPU9250 initialized, WHO_AM_I=%x\n", who_am_i);
	res = read_reg(WHO_AM_I, &who_am_i, 1);
	TRACE("MPU9250 initialized, WHO_AM_I=%x\n", who_am_i);
	res = read_reg(WHO_AM_I, &who_am_i, 1);
	TRACE("MPU9250 initialized, WHO_AM_I=%x\n", who_am_i);
		
	for(i=0; i<128; i++)
	{
		uint8_t data;
		read_reg(i, &data, 1);
		TRACE("reg %02x = %02x\n", i, data);
	}
	
	systimer->delayms(10);

	if (who_am_i != 0x68 && who_am_i != 0x71)
		return -1;
	
	m_healthy = true;

	return 0;
}

// data[0 ~ 7] :
// accel_x, accel_y, accel_z, raw_temperature, gyro_x, gyro_y, gyro_z
int MPU6000::read(short*data)
{
	int i;
	int result;
	
	if (spi)
	{
                spi->set_speed(20000000);
		spi->set_mode(1, 1);
	}
	
	result = read_reg(ACCEL_XOUT_H, (uint8_t*)data, 14);
	for(i=0; i<7; i++)
		swap((uint8_t*)&data[i], 2);

	return result;
}

int MPU6000::accelerometer_axis_config(int x, int y, int z, int negtivex, int negtivey, int negtivez)
{
	axis[0] = x;
	axis[1] = y;
	axis[2] = z;
	negtive[0] = negtivex;
	negtive[1] = negtivey;
	negtive[2] = negtivez;

	return 0;
}

int MPU6000::gyro_axis_config(int x, int y, int z, int negtivex, int negtivey, int negtivez)
{
	axis[3] = x+4;
	axis[4] = y+4;
	axis[5] = z+4;
	negtive[3] = negtivex;
	negtive[4] = negtivey;
	negtive[5] = negtivez;

	return 0;
}


int MPU6000::read(devices::accelerometer_data *out)
{
	short data[7];
	if (read(data)<0)
		return -1;
	
	out->x = data[axis[0]] * negtive[0] * G_in_ms2 / 2048.0f;
	out->y = data[axis[1]] * negtive[1] * G_in_ms2 / 2048.0f;
	out->z = data[axis[2]] * negtive[2] * G_in_ms2 / 2048.0f;
	out->temperature = data[3] / 340.0f + 36.53f;
	
	return 0;
}

int MPU6000::read(devices::gyro_data *out)
{
	short data[7];
	if (read(data)<0)
		return -1;
	
	out->x = data[axis[3]] * negtive[3] * 0.00106530f;		// to radians
	out->y = data[axis[4]] * negtive[4] * 0.00106530f;
	out->z = data[axis[5]] * negtive[5] * 0.00106530f;
	out->temperature = data[3] / 340.0f + 36.53f;
	
	return 0;
}


}
