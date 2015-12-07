#pragma once

#include <stdint.h>

enum data_publish_types
{
	data_publish_imu = 1,
	data_publish_flow = 2,	
	data_publish_baro = 4,
	data_publish_gps = 8,

	data_publish_binary = 0x8000,
};

typedef struct
{
	uint32_t timestamp;		// all time unit is microsecond (μs).
	int16_t acc[3];			// all acceleration unit is milli-meter per sencond per second(mm/s^2)
	int16_t gyro[3];		// all angular velocity unit is centi-degree degress per second (0.01°/s)
	int16_t acc2[3];
	int16_t gyro2[3];
	int16_t mag[3];			// all magnet unit is milli-gauss
}usb_imu_data;

typedef struct 
{
	uint32_t timestamp;		// all time unit is microsecond (μs).
	int16_t flow[2];		// unit: pixel
	int16_t sonar;			// unit: milli-meter
} usb_flow_data;

typedef struct 
{
	uint32_t timestamp;		// all time unit is microsecond (μs).
	int pressure;			// unit: pascal
	int16_t temperature;	// unit: centi-degree celsius (0.01°C)
} usb_baro_data;

typedef struct
{
	uint32_t timestamp;		// all time unit is microsecond (μs).
	double longitude;
	double latitude;
	int16_t altitude;		// unit: centi-meter (cm).
	int16_t hdop;			// unit: 0.01	
	int16_t speed_over_ground;	// unit: center-meter per second (cm/s)
	int16_t heading;			// 
} usb_gps_data;
