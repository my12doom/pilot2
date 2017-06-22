#pragma once

#include <stdint.h>
#include <HAL/Interface/IRCOUT.h>

// channel index starts from 0
class BrushedOut : public HAL::IRCOUT
{
public:
	BrushedOut();
	~BrushedOut(){}

	virtual int write(const int16_t *data, int count, int start_channel);
	virtual int read(int16_t *data, int count, int start_channel);
	virtual int get_channel_count();
	int enable();
	int disable();
	int cb();

protected:
	int16_t channel_data[4];
	bool armed;
	bool m3positive;
};
