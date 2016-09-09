#include "BM1383.h"

using namespace HAL;

#define FAIL_RET(x) if ((x)<0) return -1
#define address 0xBA

namespace sensors
{

BM1383::BM1383()
{
	_healthy = false;
}

BM1383::~BM1383()
{
}

int BM1383::init(HAL::II2C *i2c)
{
	this->i2c = i2c;

	systimer->delayus(100);

	FAIL_RET(i2c->write_reg(address, 0x12, 0x01));		// power up
	systimer->delayus(2000);
	FAIL_RET(i2c->write_reg(address, 0x13, 0x01));		// unreset
	systimer->delayus(100);

	for(int i=0; i<3; i++)
	{
		FAIL_RET(i2c->write_reg(address, 0x14, 0x8A));		// continuous 50ms mode, 16 times average, 37ms measurement time.
		uint8_t confirm = 0;
		i2c->read_reg(address, 0x14, &confirm);
		
		if (confirm == 0x8A)
		{
			_healthy = true;
			return 0;
		}
	}
	
	return -1;
}

int BM1383::read(devices::baro_data *out)
{
	uint8_t data[6] = {0};
	FAIL_RET(i2c->read_regs(address, 0x19, data, 5));
	out->pressure = (data[1] << 14 | (data[2] << 6) | (data[3] >> 2)) / 20.48f;		// TODO; verify this.
	int16_t tmp = (data[4] << 8) | data[5];
	out->temperature = tmp / 32.0f;

	return (data[0]&0x01)?0:1;
}

bool BM1383::healthy()
{
	return _healthy;
}


}