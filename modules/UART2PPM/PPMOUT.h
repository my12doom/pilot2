#pragma once

#include <stdint.h>


// channel index starts from 0
class PPMOUT
{
public:
	PPMOUT();
	~PPMOUT(){}

	int write(const int16_t *data, int count);
	int enable();
	int disable();
	int cb();

protected:
	int send();
	int sending;
	int16_t sending_data[10];
	int16_t channel_data[10];
	int channel_count;
};
