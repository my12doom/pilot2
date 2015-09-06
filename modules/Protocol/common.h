#ifndef __COMMON_H__
#define __COMMON_H__
#include <math.h>
#include <stdio.h>
#include <stdint.h>

#ifndef WIN32
#else
typedef unsigned char uint8_t;
#endif

#define QUADCOPTER_THROTTLE_RESERVE 0.15f
#define PI 3.14159265f
#define LOGE printf
#define TRACE(...) 
#define LOG_NRF 1
#define LOG_SDCARD 2
#define LOG_USART1 4
#define LOG_USART2 8
#define RC_DEAD_ZONE 0.04f
#define G_in_ms2 9.8065f			// gravity in m/s^2
#define countof(x) (sizeof(x)/sizeof(x[0]))
#define CRASH_TILT_IMMEDIATE	1
#define CRASH_COLLISION_IMMEDIATE	2

static const char * critical_error_desc[] = 
{
	"error_gyro",
	"error_accelerometer",
	"error_magnet",
	"error_baro",
	"error_RC",
	"error_GPS",
	"error_MAX",
};

// currently only HARD limit is used, PARTIAL limit is only a place holder.
enum actuator_limit
{
	LIMIT_NONE 				= 0,
	LIMIT_NEGATIVE_PARTIAL	= 1 << 0,
	LIMIT_NEGATIVE_HARD		= 1 << 1,
	LIMIT_POSITIVE_PARTIAL	= 1 << 2,
	LIMIT_POSITIVE_HARD		= 1 << 3,
	LIMIT_NEGAITVE_MASK		= 0x3,
	LIMIT_POSITIVE_MASK		= 0xC,
};


enum critical_error
{
	error_gyro = 1,
	error_accelerometer = 2,
	error_magnet = 4,
	error_baro = 8,
	error_RC = 16,
	error_GPS = 32,
	error_MAX,
} ;

enum fly_mode
{
	initializing,
	manual,
	acrobatic,
	fly_by_wire,
	quadcopter,
	_shutdown,
	_rc_fail,
	acrobaticV,
};

enum copter_mode
{
	invalid,	// not even in copter mode
	basic,
	althold,
	poshold,
	acro,
	bluetooth,	// bluetooth RC
	optical_flow,	// optical demo from legacy AP code
	RTL,
};

static void swap(void *p, int size)
{
	int i;
	uint8_t *pp = (uint8_t*)p;
	uint8_t tmp;
	for(i=0; i<size/2; i++)
	{
		tmp = pp[i];
		pp[i] = pp[size-1-i];
		pp[size-1-i] = tmp;
	}
}

static float limit(float v, float low, float high)
{
	if (v < low)
		return low;
	if (v > high)
		return high;
	return v;
}

static float radian_add(float a, float b)
{
	a += b;
	if (a>=PI)
		a -= 2*PI;
	if (a<-PI)
		a += 2*PI;
	
	return a;
}

#ifndef __cplusplus
#define abs fabs
#endif

// a & b : -PI ~ PI
// return a - b
static float radian_sub(float a, float b)
{
	float v1 = a-b;
	float v2 = a+2*PI-b;
	float v3 = a-2*PI-b;
	
	v1 = fabs(v1)>fabs(v2) ? v2 : v1;
	return fabs(v1)>fabs(v3) ? v3 : v1;
}

#endif
