#pragma once
#include <stdint.h>

namespace devices
{
	typedef struct
	{
		int timestamp;					// unix timestamp
		double longitude;				// longitude in degree
		double latitude;				// latitude in degree
		float speed;					// unit: meter/s
		float altitude;					// meter
		float direction;				// Track angle in degrees True, 0-360 degree, 0: north, 90: east, 180: south, 270: west, 359:almost north
		uint16_t DOP[3];				// DOP[3]: PDOP, HDOP, VOP, unit base: 0.01
		uint8_t satelite_in_view;
		uint8_t satelite_in_use;
		uint8_t declination; 			// Magnetic variation in 0.01 degrees (Easterly var. subtracts from true course)
		unsigned sig				: 4;// GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive)
		unsigned fix				: 4;// Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D)
	} gps_data;

	typedef struct
	{
		int timestamp;					// unix timestamp
		int microseconds;				// fraction part of time, in microseconds.
		double longitude;				// longitude in degree
		double latitude;				// latitude in degree
		float altitude;					// altitude above mean sea level, in meter
		float velocity[3];				// velocity over ground, north, east, down, in meter/s
		float accuracy_position[3];		// accuracy estimation of position, north, east, down, in meter, 1 sigma.
		float accuracy_velocity[3];		// accuracy estimation of velocity, in meter/s, 1 sigma.
		float DOP[3];					// DOP[3]: PDOP, HDOP, VOP
		uint8_t satelite_in_view;
		uint8_t satelite_in_use;
		uint8_t declination;			// Magnetic variation in 0.01 degrees (Easterly var. subtracts from true course)
		uint8_t fix;					// Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D)
	} gps_data2;

	class IGPS
	{
	public:
		// return 0 if new data available, 1 if old data, negative for error.
		virtual int read(gps_data *out) = 0;

		// return 0 if new data available, 1 if old data, negative for error.
		// default implementation: -1 = not implemented
		virtual int read(gps_data2 *out){return -1;}

		// return false for hardware error(still return true for signal lost)
		virtual bool healthy() = 0;
	};
}