// complemetary filter for accelerometer - GPS fusion
// caller is responsible for providing accel in lon/lat axis.
// for performance reason we don't use degree for geographic position unit, which have only ~0.1 meter un-guaranteed resolution with double variable
// and doube variable is not an option for performance and compatibility reasons.
// so we use integer variable and 1/COORDTIMES degree as unit.
// int32_t can provide ~0.01meter resolution, int64_t is even better.

#pragma once

#include <stdint.h>
#include <Protocol/common.h>
#include <utils/fifo.h>

#if 1
#define COORDTIMES 1000000000
#define COORDTYPE int64_t
#else
#define COORDTIMES 10000000
#define COORDTYPE int32_t
#endif

#ifdef WIN32
typedef long long int64_t;
#endif

// struct definitions
typedef struct
{
	COORDTYPE longtitude;		// unit: 1/COORDTIMES degree
	COORDTYPE latitude;			// unit: 1/COORDTIMES degree
	double vlongtitude;		// unit: 1/COORDTIMES degree/s
	double vlatitude;		// unit: 1/COORDTIMES degree/s
	int64_t time;
} position;

typedef struct
{
	double longtitude;		// unit: meter
	double latitude;			// unit: meter
	double vlongtitude;		// unit: meter/s
	double vlatitude;		// unit: meter/s
	int64_t time;
} position_meter;

class pos_estimator
{
public:
	pos_estimator();
	~pos_estimator();
	int reset();		// mainly for after GPS glitch handling
	int set_home(COORDTYPE lat, COORDTYPE lon);
	int update_accel(double accel_lat, double accel_lon, int64_t timestamp);			// unit: meter/s
	int update_gps(COORDTYPE lat, COORDTYPE lon, float hdop, int64_t timestamp);					// unit: degree
	void set_gps_latency(int new_latency){latency = new_latency;}

	position get_estimation();
	position_meter get_estimation_meter(){return meter;}
	position_meter get_raw_meter(){return meter_raw;}
	position get_home(){return home;}
	bool healthy();
	
	double error_lon;
	double error_lat;
	float error_lon_meter;
	float error_lat_meter;
	bool home_set;
	bool gps_healthy;
	double abias_lon;
	double abias_lat;
protected:

	double pbias_lon;
	double pbias_lat;

	double longtitude_to_meter;
	double latitude_to_meter;

	int64_t last_accel_update;
	int64_t last_gps_update;
	int latency;

	position est;
	position_meter meter;
	position_meter meter_raw;
	position home;

	CircularQueue<position, 20> history_pos;
	int64_t last_history_push;
};