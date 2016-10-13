#include "SPL06.h"

using namespace HAL;

#define FAIL_RET(x) if ((x)<0) return -1
#define address 0xBA

namespace sensors
{

SPL06::SPL06()
{
	_healthy = false;
}

SPL06::~SPL06()
{
}

int SPL06::init(HAL::II2C *i2c)
{
	this->i2c = i2c;
	
	FAIL_RET(i2c->write_reg(0x77<<1, 6, 0x45));				// 16hz, 32 OverSampling
	FAIL_RET(i2c->write_reg(0x77<<1, 7, 0x45|(0x80)));		// 16hz, 32 OverSampling, use mems sensor
	FAIL_RET(i2c->write_reg(0x77<<1, 8, 0x07));				// background mode.
	FAIL_RET(i2c->write_reg(0x77<<1, 9, 0x0C));				// data shifting
	
	
	// read calibration coeff
	uint8_t tmp[3];
	
	FAIL_RET(i2c->read_regs(0x77<<1, 0x10, tmp, 2));
	c0 = (tmp[0] << 4) | (tmp[1] >> 4);
	c0 |= (c0&0x0800) ? 0xF000 : 0;
	
	FAIL_RET(i2c->read_regs(0x77<<1, 0x11, tmp, 2));
	c1 = ((tmp[0]&0xf) << 8) | (tmp[1]);
	c1 |= (c1&0x0800) ? 0xF000 : 0;
	
	FAIL_RET(i2c->read_regs(0x77<<1, 0x13, tmp, 3));
	c00 = (tmp[0]<<12) | (tmp[1]<<4) | (tmp[2]>>4);
	c00 |= (c00&0x080000) ? 0xFFF00000 : 0;
	
	FAIL_RET(i2c->read_regs(0x77<<1, 0x15, tmp, 3));
	c10 = ((tmp[0]&0xf)<<16) | (tmp[1]<<8) | (tmp[2]);
	c10 |= (c10&0x080000) ? 0xFFF00000 : 0;
	
	FAIL_RET(i2c->read_regs(0x77<<1, 0x18, tmp, 2));
	c01 = (tmp[0] << 8) | (tmp[1]);
	
	FAIL_RET(i2c->read_regs(0x77<<1, 0x1A, tmp, 2));
	c11 = (tmp[0] << 8) | (tmp[1]);
	
	FAIL_RET(i2c->read_regs(0x77<<1, 0x1C, tmp, 2));
	c20 = (tmp[0] << 8) | (tmp[1]);
	
	FAIL_RET(i2c->read_regs(0x77<<1, 0x1E, tmp, 2));
	c21 = (tmp[0] << 8) | (tmp[1]);

	FAIL_RET(i2c->read_regs(0x77<<1, 0x20, tmp, 2));
	c30 = (tmp[0] << 8) | (tmp[1]);
	
	_healthy = true;
	
	return 0;
}

int SPL06::read(devices::baro_data *out)
{
	if (!out)
		return -1;

	uint8_t tmp[3];

	FAIL_RET(i2c->read_regs(0x77<<1, 0, tmp, 3));
	int32_t raw_pressure = (tmp[0]<<16) | (tmp[1]<<8) | (tmp[2]);
	raw_pressure |= (raw_pressure & 0x800000) ? 0xFF000000 : 0;
	
	FAIL_RET(i2c->read_regs(0x77<<1, 3, tmp, 3));
	int32_t raw_temp = (tmp[0]<<16) | (tmp[1]<<8) | (tmp[2]);
	raw_temp |= (raw_temp & 0x800000) ? 0xFF000000 : 0;

	float fTsc = raw_temp / 516096.0f;
	
	float fPsc = raw_pressure / 516096.0f;
	float qua2 = c10 + fPsc * (c20 + fPsc* c30);
	float qua3 = fTsc * fPsc * (c11 + fPsc * c21);
	
	out->temperature =  c0 * 0.5 + c1 * fTsc;
	out->pressure = c00 + fPsc * qua2 + fTsc * c01 + qua3;

	return 0;
}

bool SPL06::healthy()
{
	return _healthy;
}


}