#pragma once
#include <HAL/Interface/II2C.h>

typedef enum
{
	speed_8sps = 0,
	speed_16sps = 1,
	speed_32sps = 2,
	speed_64sps = 3,
	speed_128sps = 4,
	speed_250sps = 5,
	speed_475sps = 6,
	speed_860sps = 7,

} ads1115_speed;

typedef enum
{
	channnel_AIN0_AIN1 = 0,		// differential
	channnel_AIN0_AIN3 = 1,
	channnel_AIN1_AIN3 = 2,
	channnel_AIN2_AIN3 = 3,
	channnel_AIN0 = 4,			// single ends
	channnel_AIN1 = 5,
	channnel_AIN2 = 6,
	channnel_AIN3 = 7,
} ads1115_channel;

typedef enum
{
	gain_6V = 0,	// +-6.144V
	gain_4V = 1,	// +-4.096V
	gain_2V = 2,	// +-2.048V
	gain_1V = 3,	// +-1.024V
	gain_512 = 4,	// +-0.512V
	gain_256 = 5,	// +-0.256V
	gain_256_2 = 6,	// +-0.256V
	gain_256_3 = 7,	// +-0.256V
} ads1115_gain;

typedef enum
{
	mode_continuous = 0,
	mode_singleshot = 1,
} ads1115_mode;


typedef struct
{
	unsigned MODE:1;
	unsigned PGA:3;
	unsigned MUX:3;
	unsigned OS:1;


	unsigned COMP_QUE:2;
	unsigned COMP_LAT:1;
	unsigned COMP_POL:1;
	unsigned COMP_MODE:1;
	unsigned DR:3;

} config_value;


typedef struct
{
	ads1115_speed speed;
	ads1115_channel channel;
	ads1115_gain gain;
	int16_t *out;
}ads1115_work;

namespace sensors
{

#define MAX_ADS1115_WORKS 8
class ADS1115
{
public:
	ADS1115();
	~ADS1115();
	int init(HAL::II2C *i2c, int address);
	int config(ads1115_speed speed, ads1115_channel channel, ads1115_gain gain, ads1115_mode mode);
	int startconvert(void);
	int getresult(short *result);		// return -1 if still converting, 0 if conversion completed or continuous mode, further calls return the last conversion result.
	short convert(void);				// a simplfied version which start a new conversion ,wait it to complete and returns the result directly

	// batch worker
	int new_work(ads1115_speed speed, ads1115_channel channel, ads1115_gain gain, int16_t *out);
	int go_on(void);

protected:
	ads1115_work ads1115_works[MAX_ADS1115_WORKS];
	int ads1115_work_pos/* = 0 */;
	int ads1115_work_count/* = 0 */;
	HAL::II2C *i2c;
	int address;
	config_value _config;
};
};