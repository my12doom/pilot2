#include "SPL06.h"

using namespace HAL;

#define FAIL_RET(x) if ((x)<0) return -1

namespace sensors
{

SPL06::SPL06()
{
	_healthy = false;
}

SPL06::~SPL06()
{
}

int SPL06::init(HAL::II2C *i2c, int address)
{
	this->i2c = i2c;
	this->address = address;
	uint8_t tmp[3];
	
	// SPL06
	FAIL_RET(i2c->write_reg(address, 6, 0x45));				// 16hz, 32 OverSampling
	FAIL_RET(i2c->write_reg(address, 7, 0x45|(0x80)));		// 16hz, 32 OverSampling, use mems sensor
	FAIL_RET(i2c->write_reg(address, 8, 0x07));				// background mode.
	FAIL_RET(i2c->write_reg(address, 9, 0x0C));				// data shifting
	

	// DPS310
	FAIL_RET(i2c->write_reg(address, 0x0C, 0x09));				// reset
	systimer->delayms(100);
	// unlock address 0x62
	FAIL_RET(i2c->write_reg(address, 0x0e, 0xa5));
	FAIL_RET(i2c->write_reg(address, 0x0f, 0x96));
	// Then update high gain value for Temperature
	FAIL_RET(i2c->write_reg(address, 0x62, 0x02));
	// lock back the location 0x62
	FAIL_RET(i2c->write_reg(address, 0x0e, 0x00));
	FAIL_RET(i2c->write_reg(address, 0x0f, 0x00));

	FAIL_RET(i2c->read_reg(address, 0x28, tmp));
	systimer->delayms(60);
	FAIL_RET(i2c->write_reg(address, 7, 0x45|(tmp[0]&0x80)));	// 16hz, 32 OverSampling
	FAIL_RET(i2c->write_reg(address, 6, 0x45));					// 16hz, 32 OverSampling
	FAIL_RET(i2c->write_reg(address, 8, 0x07));					// background mode.
	FAIL_RET(i2c->write_reg(address, 9, 0x0C));					// data shifting
	
	// read calibration coeff
	
	FAIL_RET(i2c->read_regs(address, 0x10, tmp, 2));
	c0 = (tmp[0] << 4) | (tmp[1] >> 4);
	c0 |= (c0&0x0800) ? 0xF000 : 0;
	
	FAIL_RET(i2c->read_regs(address, 0x11, tmp, 2));
	c1 = ((tmp[0]&0xf) << 8) | (tmp[1]);
	c1 |= (c1&0x0800) ? 0xF000 : 0;
	
	FAIL_RET(i2c->read_regs(address, 0x13, tmp, 3));
	c00 = (tmp[0]<<12) | (tmp[1]<<4) | (tmp[2]>>4);
	c00 |= (c00&0x080000) ? 0xFFF00000 : 0;
	
	FAIL_RET(i2c->read_regs(address, 0x15, tmp, 3));
	c10 = ((tmp[0]&0xf)<<16) | (tmp[1]<<8) | (tmp[2]);
	c10 |= (c10&0x080000) ? 0xFFF00000 : 0;
	
	FAIL_RET(i2c->read_regs(address, 0x18, tmp, 2));
	c01 = (tmp[0] << 8) | (tmp[1]);
	
	FAIL_RET(i2c->read_regs(address, 0x1A, tmp, 2));
	c11 = (tmp[0] << 8) | (tmp[1]);
	
	FAIL_RET(i2c->read_regs(address, 0x1C, tmp, 2));
	c20 = (tmp[0] << 8) | (tmp[1]);
	
	FAIL_RET(i2c->read_regs(address, 0x1E, tmp, 2));
	c21 = (tmp[0] << 8) | (tmp[1]);

	FAIL_RET(i2c->read_regs(address, 0x20, tmp, 2));
	c30 = (tmp[0] << 8) | (tmp[1]);
	
	_healthy = true;
	
	return 0;
}

	uint8_t tmp[3];
int SPL06::read(devices::baro_data *out)
{
	if (!out)
		return -1;


	FAIL_RET(i2c->read_regs(address, 0, tmp, 3));
	int32_t raw_pressure = (tmp[0]<<16) | (tmp[1]<<8) | (tmp[2]);
	raw_pressure |= (raw_pressure & 0x800000) ? 0xFF000000 : 0;
	
	FAIL_RET(i2c->read_regs(address, 3, tmp, 3));
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