#include "Sonar.h"
#include <math.h>
#include <stdio.h>

#define TIMEOUT	59000		// 59ms, ~10 meter
#define DEADBAND 0		// 2ms, ~34cm
#define MIN_PULSE_COUNT 5
#define PULSE_TIMEOUT 50	// 30us max pulse interval, 2x 

namespace sensors
{
	Sonar::Sonar()
	{
		send_time = 0;
		echo_confirmed = false;
	}
	int Sonar::init(HAL::IGPIO *tx, HAL::IGPIO *level)
	{
		this->tx = tx;
		this->level = level;
		
		if (tx)
			tx->set_mode(HAL::MODE_OUT_PushPull);
		if (level)
			level->set_mode(HAL::MODE_OUT_OpenDrain);
		
		return tx && level ? 0 : -1;
	}
	// return 0 if new data available, 1 if old data, negative for error.
	// unit: meter.
	int Sonar::read(float *out, int64_t *timestamp/* = NULL*/)
	{
		*out = distance;
		
		if (fresh_data)
		{
			if (timestamp) *timestamp = send_time;
			fresh_data = false;
			return 0;
		}
		
		return 1;
	}

	// trigger messuring manually, this is needed by some types of range finder(sonars e.g.)
	int Sonar::trigger()
	{
		// sonic wave still flying?
		if (systimer->gettime() < send_time+TIMEOUT/* && !echo_confirmed*/)
			return 1;
		
		// process last result
		if (!echo_confirmed)
		{
			distance = 0;
			fresh_data = true;
		}
		
		// reset variables
		send_time = systimer->gettime();
		echo_confirmed = false;
		pulse_counter = 0;
		first_pulse_time = last_pulse_time = -PULSE_TIMEOUT;
		distance = 0;
		
		// pull level shifter low
		level->write(false);
		
		// send 8 pulses
		for(int i=0; i<8; i++)
		{
			tx->write(true);
			systimer->delayus(12);	// this might need some tuning
			tx->write(false);
			systimer->delayus(12);
		}
		
		// release level shifter
		level->write(true);
		
		return 0;
	}
	
	void Sonar::echo()
	{
		if (echo_confirmed)
			return;
		
		// time since sending
		int t = systimer->gettime() - send_time;
		
		// deadband
		if (t<DEADBAND)
			return;
		
		// pulse timeout
		if (t-last_pulse_time > PULSE_TIMEOUT)
		{
			pulse_counter = 0;
			first_pulse_time = t;
		}
		else
		{
			pulse_counter++;
		}
		last_pulse_time = t;
		
		// confirmed ?
		if (pulse_counter >= MIN_PULSE_COUNT)
		{
			fresh_data = true;
			echo_confirmed = true;
			distance = first_pulse_time*0.000001f * 340/2;
		}
	}
	
	// return false if any error/waning
	bool Sonar::healthy()
	{
		return tx && level ? 0 : -1;
	}
}