#pragma once

#include <stdint.h>
#include <HAL/Interface/IRCOUT.h>

// channel index starts from 0
class PPMOUT : public HAL::IRCOUT
{
public:
	PPMOUT();
	~PPMOUT(){}

	virtual int write(const int16_t *data, int count, int start_channel);
	virtual int read(int16_t *data, int count, int start_channel);
	virtual int get_channel_count();
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
