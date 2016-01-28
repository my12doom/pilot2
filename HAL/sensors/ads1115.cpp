#include "ads1115.h"
#include <string.h>
#include <stdio.h>

#define ADDR1 0x90
#define ADDR2

#define CONVERSION 0
#define CONFIG 1
#define LOW_THRESH 2
#define HIGH_THRESH 3

using namespace HAL;
using namespace sensors;

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

ADS1115::ADS1115()
{

}
ADS1115::~ADS1115()
{

}


int ADS1115::init(HAL::II2C *i2c, int address)
{
	this->address = address;
	this->i2c = i2c;

	memset(&_config, 0, 4);
	i2c->write_reg(0, 0x06, 0);			// 0,0x06,X = reset
	i2c->read_regs(ADDR1, CONFIG, (uint8_t*)&_config, 2);
	
	if (*(unsigned short*)&_config != 0x8385)	// the default config register
		return -1;
	
	return 0;
}

int ADS1115::config(ads1115_speed speed, ads1115_channel channel, ads1115_gain gain, ads1115_mode mode)
{
	i2c->read_regs(ADDR1, CONFIG, (uint8_t*)&_config, 2);
	_config.DR = speed;
	_config.MUX = channel;
	_config.PGA = gain;
	_config.MODE = mode;
	i2c->write_regs(ADDR1, CONFIG, (uint8_t*)&_config, 2);
	
	return 0;
}

int ADS1115::startconvert(void)
{
	i2c->read_regs(ADDR1, CONFIG, (uint8_t*)&_config, 2);
	_config.OS = 1;
	i2c->write_regs(ADDR1, CONFIG, (uint8_t*)&_config, 2);
	
	return 0;
}

int ADS1115::getresult(short *result)		// return -1 if still converting, 0 if conversion completed, further calls return the last conversion result.
{
	_config.MODE = 1;
	_config.OS = 0;
	
	if (i2c->read_regs(ADDR1, CONFIG, (uint8_t*)&_config, 2) < 0)
		return -1;
		
	if (_config.MODE == 1 && _config.OS == 0)
	{
		return -1;
	}

	i2c->read_regs(ADDR1, CONVERSION, (uint8_t*)result, 2);
	swap(result, 2);
	
	return 0;
}

short ADS1115::convert(void)				// a simplfied version which start a new conversion ,wait it to complete and returns the result directly
{
	short v;
	startconvert();
	while (getresult(&v)!=0)
		;
	return v;
}

int ADS1115::new_work(ads1115_speed speed, ads1115_channel channel, ads1115_gain gain, int16_t *out)
{
	ads1115_work *p = &ads1115_works[ads1115_work_count++];
	if (ads1115_work_count >= MAX_ADS1115_WORKS)
		return -1;

	p->speed = speed;
	p->channel = channel;
	p->gain = gain;
	p->out = out;

	if (ads1115_work_count == 1)
	{
		config(speed, channel, gain,mode_singleshot);
		startconvert();
	}

	return 0;
}

int ADS1115::go_on()
{
	int res;
	if (ads1115_work_count == 0)
		return 1;

	res = getresult(ads1115_works[ads1115_work_pos].out);
	if (res == 0)
	{
		ads1115_work *p;
		ads1115_work_pos = (ads1115_work_pos+1)%ads1115_work_count;
		p = &ads1115_works[ads1115_work_pos];
		config(p->speed, p->channel, p->gain, mode_singleshot);
		startconvert();
		return 0;
	}

	return 1;
}
}