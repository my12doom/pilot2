#include "IST8307A.h"
#include <math.h>
#include <stdio.h>
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
#define IST8307_SLA                 0x0C    //IST8307 Slave Address

static int16_t min(int16_t a, int16_t b)
{
	return a>b?b:a;
}

namespace sensors
{

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
	m_healthy = false;
	uint8_t tmp = 0;

	// soft reset
	write_reg_core(IST8307_REG_CNTRL2, 0x1);
	systimer->delayms(10);

	// read IST8307A WHO_AM_I register
	if (read_reg(IST8307_REG_WIA, &tmp, 1) < 0 || tmp != 0xff);	// 0x85: place holder..
	{
		LOGE("no IST8307A found\n");
		return -1;
	}
		
	FAIL_RETURN(write_reg(IST8307_REG_CNTRL1, 0x01));
	FAIL_RETURN(write_reg(IST8307_REG_TEST_REG, 0x01));
	FAIL_RETURN(write_reg(IST8307_REG_SELECTION_REG, 0x00));

	m_healthy = true;

	return 0;
}

int IST8307A::read(short*data)
{
	int result = read_reg(IST8307_REG_DATAX, (uint8_t*)data, 6);

	return result;
}


int IST8307A::read(devices::mag_data *out)
{
	short data[3];
	if (read_reg(IST8307_REG_DATAX, (uint8_t*)data, 6)<0)
		return -1;
	
	out->x = data[axis[0]] * 0.03f * negtive[0];		// to milli-gauss (1mGauss = 0.1 uT)
	out->y = data[axis[1]] * 0.03f * negtive[1];
	out->z = data[axis[2]] * 0.03f * negtive[2];
	out->temperature = NAN;
	
	return 0;
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