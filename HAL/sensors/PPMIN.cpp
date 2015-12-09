#include "PPMIN.h"
#include <string.h>

static int min(int a, int b)
{
	return a > b ? b : a;
}
static int max(int a, int b)
{
	return a > b ? a : b;
}
int sensors::PPMIN::handle_ppm(int64_t now)
{
	float delta = 0;
	if (last_high_tim < 0)
	{
		last_high_tim = now;
		return 0;
	}
	
	if (now > last_high_tim)
		delta = now - last_high_tim;
	else
		delta = now + 60000 - last_high_tim;

	last_high_tim = now;
	
	if (delta > 2100)
	{
		ppm_channel_count = ppm_channel_id;
		ppm_channel_id = 0;
		//printf("        %.0f    \r", delta);
	}
	else if (ppm_channel_id < sizeof(rc_input)/sizeof(rc_input[0]))
	{
		rc_input[ppm_channel_id] = delta;
		rc_static[0][ppm_channel_id] = min(rc_static[0][ppm_channel_id], rc_input[ppm_channel_id]);
		rc_static[1][ppm_channel_id] = max(rc_static[1][ppm_channel_id], rc_input[ppm_channel_id]);
		//printf("%d,", rc_input[ppm_channel_id]);

		rc_update[ppm_channel_id] = systimer->gettime();

		// swap 4&5 channel
		if (ppm_channel_id == 3)
			ppm_channel_id = 5;
		else if (ppm_channel_id == 5)
			ppm_channel_id = 4;
		else if (ppm_channel_id == 4)
			ppm_channel_id = 6;
		else
			ppm_channel_id++;
	}
	return 0;
}

sensors::PPMIN::PPMIN()
:last_high_tim(-1)
,ppm_channel_id(0)
,ppm_channel_count(0)
{
}

// total channel count
int sensors::PPMIN::get_channel_count()
{
	return ppm_channel_count;
}

// return num channel written to out pointer
int sensors::PPMIN::get_channel_data(int16_t *out, int start_channel, int max_count)
{
	int count = min(ppm_channel_count - start_channel, max_count);
	memcpy(out, rc_input + start_channel, count * sizeof(int16_t));
	
	return count;
}

// return num channel written to out pointer
int sensors::PPMIN::get_channel_update_time(int64_t *out, int start_channel, int max_count)
{
	int count = min(ppm_channel_count - start_channel, max_count);
	memcpy(out, rc_update + start_channel, count * sizeof(int64_t));
	
	return count;
}

// statistics functions is mainly for RC calibration purpose.
int sensors::PPMIN::get_statistics_data(int16_t *min_out, int16_t *max_out, int start_channel, int max_count)
{
	int count = min(ppm_channel_count - start_channel, max_count);
	memcpy(min_out, rc_static[0] + start_channel, count * sizeof(int16_t));
	memcpy(max_out, rc_static[1] + start_channel, count * sizeof(int16_t));
	
	return count;
}
int sensors::PPMIN::reset_statistics()
{
	int i;
	for(i=0; i<8; i++)
	{
		rc_static[0][i] = 32767;			// min
		rc_static[1][i] = 0;				// max
	}

	return 0;
}
bool sensors::PPMIN::init(HAL::IInterrupt *interrupt)
{
	interrupt->set_callback(cb_entry, this);
	
	return true;
}
void sensors::PPMIN::cb_entry(void *parameter, int flags)
{
	PPMIN * _this = (PPMIN*)parameter;
	_this->handle_ppm(systimer->gettime());
}

