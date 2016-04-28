#include "TLC59208F.h"
#include <stdlib.h>
#include <string.h>

#include <Protocol/common.h>

namespace dev_v2
{

#define FAIL_RET(x) if((x) < 0 ) return -1
	
TLC59208F::TLC59208F()
:i2c(NULL)
{
}

int TLC59208F::init(HAL::II2C *i2c, uint8_t address, uint8_t channel_map[8])
{
	if (!i2c)
		return -1;

	this->address = address;
	this->i2c = i2c;
	memcpy(this->channel_map, channel_map, 8);

	FAIL_RET(i2c->write_reg(address, 0, 1));
	FAIL_RET(i2c->write_reg(address, 0xc, 0xff));
	FAIL_RET(i2c->write_reg(address, 0xd, 0xff));
	for(int i=0; i<8; i++)
		FAIL_RET(i2c->write_reg(address, 2+i, 0));

	return 0;
}

int TLC59208F::write(float R, float G, float B)
{
	R = limit(R*255, 0, 255);
	G = limit(G*255, 0, 255);
	B = limit(B*255, 0, 255);
	
	for(int i=0; i<8; i++)
	{
		if (channel_map[i] == 0)
			i2c->write_reg(address, 2+i, R);
		if (channel_map[i] == 1)
			i2c->write_reg(address, 2+i, G);
		if (channel_map[i] == 2)
			i2c->write_reg(address, 2+i, B);
	}
	return 0;
}

}
