#include "IST8307A.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <protocol/common.h>

#define FAIL_RETURN(x) if((x)<0) return-1

#define IST8307_REG_WIA				0x00	//Who I am
#define IST8307_REG_INFO			0x01	//More Info
#define IST8307_REG_DATAX			0x03	//Output Value x
#define IST8307_REG_DATAY			0x05	//Output Value y
#define IST8307_REG_DATAZ			0x07	//Output Value z
#define IST8307_REG_STAT1			0x02	//Status register
#define IST8307_REG_STAT2			0x09	//Status register
#define IST8307_REG_CNTRL1			0x0A	//Control setting register 1
#define IST8307_REG_CNTRL2			0x0B	//Control setting register 2
#define IST8307_REG_CNTRL3			0x0D	//Control setting register 3
#define IST8307_REG_OFFSET_START	0xDC	//Offset
#define IST8307_REG_SELECTION_REG   0x42    //Sensor Selection register
#define IST8307_REG_TEST_REG        0x40    //Chip Test register
#define IST8307_REG_TUNING_REG      0x47    //Bandgap Tuning register
#define IST8307_SLA                 0x18    //IST8307 Slave Address
#define IST8307_ODR_MODE            0x01    //Force mode

/*---IST8307 cross-axis matrix Address-----------------danny-----*/
#define IST8307_REG_XX_CROSS_L       0x9C   //cross axis xx low byte
#define IST8307_REG_XX_CROSS_H       0x9D   //cross axis xx high byte
#define IST8307_REG_XY_CROSS_L       0x9E   //cross axis xy low byte
#define IST8307_REG_XY_CROSS_H       0x9F   //cross axis xy high byte
#define IST8307_REG_XZ_CROSS_L       0xA0   //cross axis xz low byte
#define IST8307_REG_XZ_CROSS_H       0xA1   //cross axis xz high byte

#define IST8307_REG_YX_CROSS_L       0xA2   //cross axis yx low byte
#define IST8307_REG_YX_CROSS_H       0xA3   //cross axis yx high byte
#define IST8307_REG_YY_CROSS_L       0xA4   //cross axis yy low byte
#define IST8307_REG_YY_CROSS_H       0xA5   //cross axis yy high byte
#define IST8307_REG_YZ_CROSS_L       0xA6   //cross axis yz low byte
#define IST8307_REG_YZ_CROSS_H       0xA7   //cross axis yz high byte

#define IST8307_REG_ZX_CROSS_L       0xA8   //cross axis zx low byte
#define IST8307_REG_ZX_CROSS_H       0xA9   //cross axis zx high byte
#define IST8307_REG_ZY_CROSS_L       0xAA   //cross axis zy low byte
#define IST8307_REG_ZY_CROSS_H       0xAB   //cross axis zy high byte
#define IST8307_REG_ZZ_CROSS_L       0xAC   //cross axis zz low byte
#define IST8307_REG_ZZ_CROSS_H       0xAD   //cross axis zz high byte

static int16_t min(int16_t a, int16_t b)
{
	return a>b?b:a;
}

namespace sensors
{

// see http://mathworld.wolfram.com/MatrixInverse.html
static int inverse_matrix3x3(const float src[3][3], float dst[3][3])
{
	float det = +src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) 
		-src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0])
		+src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);

	det = 1.0f/det;

	dst[0][0] =  det * (src[1][1]*src[2][2] - src[1][2] * src[2][1]);
	dst[0][1] = det * (src[0][2]*src[2][1] - src[0][1] * src[2][2]);
	dst[0][2] =  det * (src[0][1]*src[1][2] - src[0][2] * src[1][1]);

	dst[1][0] = det * (src[1][2]*src[2][0] - src[1][0] * src[2][2]);
	dst[1][1] =  det * (src[0][0]*src[2][2] - src[0][2] * src[2][0]);
	dst[1][2] = det * (src[0][2]*src[1][0] - src[0][0] * src[1][2]);

	dst[2][0] =  det * (src[1][0]*src[2][1] - src[1][1] * src[2][0]);
	dst[2][1] = det * (src[0][1]*src[2][0] - src[0][0] * src[2][1]);
	dst[2][2] =  det * (src[0][0]*src[1][1] - src[0][1] * src[1][0]);

	return 0;
}

int IST8307A::read_reg(uint8_t reg, void *out, int count)
{
	if (i2c)
		return i2c->read_regs(IST8307_SLA, reg, (uint8_t*)out, count);

	return -1;
}

int IST8307A::write_reg_core(uint8_t reg, uint8_t data)
{
	if (i2c)
		return i2c->write_reg(IST8307_SLA, reg, data);

	return -1;
}

int IST8307A::write_reg(uint8_t reg, uint8_t data)
{
	uint8_t read;
	for(int i=0; i<10; i++)
	{
		read = data - 1;
		write_reg_core(reg, data);
		read_reg(reg, &read, 1);

		if (read == data)
			return 0;
		else
			printf("reg(%02x) error %02x/%02x\n", reg, read, data);
		
		systimer->delayms(10);
	}

	return -1;
}

IST8307A::IST8307A()
{
	i2c = NULL;
}


int IST8307A::init(HAL::II2C *i2c)
{
	this->i2c = i2c;
	i2c->set_speed(20);
	m_healthy = false;
	uint8_t tmp = 0;
	memset(last_data, 0, sizeof(last_data));
	memset(cross_axis_matrix, 0, sizeof(cross_axis_matrix));

	// soft reset
	write_reg_core(IST8307_REG_CNTRL2, 0x1);
	systimer->delayms(10);

	if (read_reg(IST8307_REG_WIA, &tmp, 1) < 0 || tmp != 0xff)
	{
		LOGE("no IST8307A found\n");
		return -1;
	}
		
	FAIL_RETURN(write_reg(IST8307_REG_CNTRL1, 0x0));
	FAIL_RETURN(write_reg(IST8307_REG_TEST_REG, 0x01));
	FAIL_RETURN(write_reg(IST8307_REG_SELECTION_REG, 0x00));

	// cross axis data
	short data[9];
	if (read_reg(IST8307_REG_XX_CROSS_L, data, 18) < 0)
	{
		LOGE("error reading IST8307A cross axis data\n");
		return -1;
	}
	
	// no cross axis data
	if (data[0] == -1)
	{
		cross_axis_matrix[0][0] = 1;
		cross_axis_matrix[1][1] = 1;
		cross_axis_matrix[2][2] = 1;
	}
	else
	{
		float dataf[3][3];
		for(int i=0; i<9; i++)
		{
			dataf[i/3][i%3] = data[i] / 330.0f;
		}

		inverse_matrix3x3(dataf, cross_axis_matrix);
	}

	// start convert
	write_reg_core(IST8307_REG_CNTRL1, IST8307_ODR_MODE);

	m_healthy = true;

	return 0;
}



int IST8307A::read(devices::mag_data *out)
{
	uint8_t stat = 0;
	read_reg(IST8307_REG_STAT1, &stat, 1);

	int v = 0;

	if (stat == 1)
	{
		// data ready, read it and start new one
		short data[3];
		write_reg_core(IST8307_REG_CNTRL1, IST8307_ODR_MODE);
		if (read_reg(IST8307_REG_DATAX, data, 6)<0)
			return -1;

		// cross axis correction
		for(int i=0; i<3; i++)
			last_data[i] = data[0] * cross_axis_matrix[0][i] + data[1] * cross_axis_matrix[1][i] + data[2] * cross_axis_matrix[2][i];
	}
	else
	{
		// no new data, return last data
		v = 1;
	}
	
	
	out->x = last_data[axis[0]] * 3.00f * negtive[0];		// to milli-gauss (1mGauss = 0.1 uT, sensitivity = 3.3 LSB/uT = 0.33LSB/mGauss = 3.00 mGauss/LSB)
	out->y = last_data[axis[1]] * 3.00f * negtive[1];
	out->z = last_data[axis[2]] * 3.00f * negtive[2];
	out->temperature = NAN;
	
	return v;
}

int IST8307A::axis_config(int x, int y, int z, int negtivex, int negtivey, int negtivez)
{
	axis[0] = x;
	axis[1] = y;
	axis[2] = z;
	negtive[0] = negtivex;
	negtive[1] = negtivey;
	negtive[2] = negtivez;

	return 0;
}

// return false if any error/waning
bool IST8307A::healthy()
{
	return m_healthy;
}

}