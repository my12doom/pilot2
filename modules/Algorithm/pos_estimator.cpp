#include "pos_estimator.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

double _time_constant_xy = 2.5f;
double _k1_xy = 3 / _time_constant_xy;
double _k2_xy = 3 / (_time_constant_xy*_time_constant_xy);
double _k3_xy = 1 / (_time_constant_xy*_time_constant_xy*_time_constant_xy);

#ifndef PI
#define PI 3.14159265
#endif
#define history_interval 50000		// 50ms, 20 point supports 1000ms max delay

pos_estimator::pos_estimator()
{
	latency = 400000;
	reset();
}

pos_estimator::~pos_estimator()
{

}

int pos_estimator::reset()		// mainly for after GPS glitch handling
{
	abias_lon = 0;
	abias_lat = 0;
	pbias_lon = 0;
	pbias_lat = 0;
	error_lon = 0;
	error_lat = 0;
	home_set = false;
	healthy = false;
	last_accel_update = 0;
	last_gps_update = 0;
	last_history_push = 0;

	memset(&home, 0, sizeof(home));
	history_pos.clear();

	return 0;
}

int pos_estimator::set_home(COORDTYPE lat, COORDTYPE lon)
{
	return 0;
}

int pos_estimator::update_accel(double accel_lat, double accel_lon, int64_t timestamp)			// unit: meter/s
{
	if (!home_set)
		return -1;

	double dt = (timestamp - last_accel_update)/1000000.0f;
	last_accel_update = timestamp;
	if (dt <=0 || dt > 1)
		return 0;

	accel_lat /= latitude_to_meter;
	accel_lon /= longtitude_to_meter;

	abias_lon += error_lon * _k3_xy * dt;		// accel bias
	abias_lat += error_lat * _k3_xy * dt;		// accel bias

	est.vlongtitude += error_lon * _k2_xy * dt;
	est.vlatitude += error_lat * _k2_xy * dt;

	pbias_lon += error_lon * _k1_xy * dt;
	pbias_lat += error_lat * _k1_xy * dt;

	double v_increase_lon = (accel_lon + abias_lon) * dt;
	double v_increase_lat = (accel_lat + abias_lat) * dt;

	est.longtitude += ((COORDTYPE)(est.vlongtitude) + v_increase_lon*0.5f) * dt;
	est.latitude += ((COORDTYPE)(est.vlatitude) + v_increase_lat*0.5f) * dt;

	est.vlongtitude += v_increase_lon;
	est.vlatitude += v_increase_lat;
	est.time = timestamp;
	
	meter.longtitude = (est.longtitude + (COORDTYPE)(pbias_lon) - home.longtitude) * longtitude_to_meter;
	meter.latitude = (est.latitude + (COORDTYPE)(pbias_lat) - home. latitude) * latitude_to_meter;
	meter.vlongtitude = (est.vlongtitude) * longtitude_to_meter;
	meter.vlatitude = (est.vlatitude) * latitude_to_meter;

	// push data to buffer for later use
	if (timestamp - last_history_push > history_interval)
	{
		last_history_push = timestamp;
		history_pos.push(est);
	}

	return 0;
}

int pos_estimator::update_gps(COORDTYPE lat, COORDTYPE lon, float hdop, int64_t timestamp)			// unit: degree
{
	if (!home_set || !healthy && hdop < 2.5f)
	{
		if (!home_set)
		{
			reset();
			home.latitude = lat;
			home.longtitude = lon;
			home_set = true;
		}
		healthy = true;
		est.latitude = lat;
		est.longtitude = lon;
	}

	if (hdop > 3.5f)
	{
		if (timestamp - last_gps_update > 8000000)
			healthy = false;

		return -1;
	}

	last_gps_update = timestamp;

	// TODO: ublox GPS has ~400ms latency, calculate position error to history data.
	position _history_pos = est;

	while(history_pos.count() > 0 && latency > 0)
	{
		position peek;
		history_pos.peek(0, &peek);

		if (peek.time < timestamp - latency || peek.time > timestamp)
			history_pos.pop_n(1);
		else
		{
			_history_pos = peek;
			break;
		}
	}

	// only position error is calculated, estamation is only updated in update_accel() since accel update in every cycle
	error_lon = lon - (_history_pos.longtitude + (COORDTYPE)(pbias_lon));
	error_lat = lat - (_history_pos.latitude + (COORDTYPE)(pbias_lat));

	// update ratios
	longtitude_to_meter = (40007000.0f/COORDTIMES/360*cos(est.latitude* PI / 180/COORDTIMES));
	latitude_to_meter = (40007000.0f/COORDTIMES/360);

	// update raw meter
	meter_raw.longtitude = (lon - home.longtitude) * longtitude_to_meter;
	meter_raw.latitude = (lat - home. latitude) * latitude_to_meter;
	meter_raw.vlongtitude = 0;
	meter_raw.vlatitude = 0;


	return 0;
}

position pos_estimator::get_estimation()
{
	position o = 
	{
		est.longtitude + (COORDTYPE)(pbias_lon),
		est.latitude + (COORDTYPE)(pbias_lat),
		est.vlongtitude,
		est.vlatitude,
	};
	return o;
}