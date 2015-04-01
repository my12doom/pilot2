#include "MPU6000.h"
#include <stdio.h>
#include <stm32f4xx_spi.h>

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

int MPU6000::read_reg(uint8_t reg, void *out, int count)
{
	int i;
	uint8_t *p = (uint8_t*)out;
	systimer->delayus(1);
	CS->write(false);
	systimer->delayus(1);

	spi->txrx((reg&0x7f) | 0x80);
	for(i=0; i<count; i++)
		p[i] = spi->txrx(0);

	systimer->delayus(1);
	CS->write(true);
	systimer->delayus(1);

	return 0;
}

int MPU6000::write_reg_core(uint8_t reg, uint8_t data)
{
	systimer->delayus(1);
	CS->write(false);
	systimer->delayus(1);

	spi->txrx(reg);
	spi->txrx(data);

	systimer->delayus(1);
	CS->write(true);
	systimer->delayus(1);

	return 0;
}

int MPU6000::write_reg(uint8_t reg, uint8_t data)
{
	uint8_t read;
	while(1)
	{
		read = data - 1;
		write_reg_core(reg, data);
		read_reg(reg, &read, 1);

		if (read == data)
			return 0;
		else
			TRACE("reg(%02x) error %02x/%02x\n", reg, read, data);
		
		systimer->delayms(10);
	}
}

int MPU6000::init(HAL::ISPI *SPI, HAL::IGPIO *CS)
{
	uint8_t who_am_i = 0;
	int i;

	this->spi = SPI;
	this->CS = CS;

	// MPU6000 can handle 1mhz max for configuration register accessing
	// 20mhz for data output and interrupt accesss
	SPI->set_speed(1000000);
	SPI->set_mode(1, 1);
	CS->set_mode(HAL::MODE_OUT_PushPull);
	CS->write(true);

	// MPU6000 register initialization
	TRACE("start MPU6000\r\n");
	write_reg_core(PWR_MGMT_1, 0x80);
	systimer->delayms(10);
	//write_reg(USER_CTRL, 0x10);
	write_reg(PWR_MGMT_1, 0x00);
	write_reg(PWR_MGMT_2, 0x00);
	write_reg(SMPLRT_DIV, 0x01);
	write_reg(MPU9250_CONFIG, 0x2);
	write_reg(GYRO_CONFIG, 1 << 3);			// full scale : 500 degree/s, ~65.5 LSB/degree/s
	write_reg(ACCEL_CONFIG, 0x18);			// full scale : 16g, 2048 = 1g
	
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

	if (who_am_i != 0x71)
		return -1;

	// enter SPI high speed mode for data only access
 	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
 	//SPI_Init(SPI2, &SPI_InitStructure);

	//
	
	return 0;
}

// data[0 ~ 7] :
// accel_x, accel_y, accel_z, raw_temperature, gyro_x, gyro_y, gyro_z
int MPU6000::read(short*data)
{
	int i;
	int result;
	spi->set_speed(20000000);
	spi->set_mode(1, 1);

	
	result = read_reg(ACCEL_XOUT_H, (uint8_t*)data, 14);
	for(i=0; i<7; i++)
		swap((uint8_t*)&data[i], 2);

	return result;
}

}
