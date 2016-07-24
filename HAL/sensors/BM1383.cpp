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

	FAIL_RET(i2c->write_reg(address, 0x12, 0x10));		// power down
	systimer->delayms(2);
	FAIL_RET(i2c->write_reg(address, 0x13, 0x01));		// reset
	systimer->delayms(2);

	FAIL_RET(i2c->write_reg(address, 0x14, 0x8A));		// continuous 50ms mode, 16 times average, 37ms measurement time.

	return 0;
}

int BM1383::read(devices::baro_data *out)
{
	uint8_t data[3] = {0};
	FAIL_RET(i2c->read_regs(address, 0x1C, data, 3));
	out->pressure = (data[2] >> 2 | (data[0] << 16) | (data[1] << 8)) / 20.48f;		// TODO; verify this.
	out->temperature = 25.0f;

	return 0;
}

bool BM1383::healthy()
{
	return _healthy;
}


}