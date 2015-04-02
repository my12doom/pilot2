#include "ahrs2.h"
#include <math.h>
#include <Protocol/common.h>
#include <utils/vector.h>

float roll;
float pitch;
float yaw_mag;
float yaw_gyro;


vector accel_ef;
float accelz_mwc = 0;
vector estAccGyro = {0};			// for roll & pitch
vector estMagGyro = {0};			// for yaw
vector estGyro = {0};				// for gyro only yaw, yaw lock on this
vector groundA;						// ground accerometer vector
vector groundM;						// ground magnet vector
static vector gyro_bias;
static float mag_tolerate = 0.25f;

int ahrs_mwc_update(vector gyro, vector accel, vector mag, float dt)
{
	vector delta_rotation = gyro;
	vector_add(&gyro, &gyro_bias);
	vector_multiply(&delta_rotation, dt);
	vector_rotate(&estGyro, delta_rotation.array);
	vector_rotate(&estAccGyro, delta_rotation.array);
	vector_rotate(&estMagGyro, delta_rotation.array);


	// apply CF filter for Mag : 0.5hz low pass for mag
	float alpha = dt / (dt + 1.0f/(2*3.1415926 * 0.5f));
	float alpha2 = dt / (dt + 1.0f/(2*3.1415926 * 0.15f));

	vector mag_delta = estMagGyro;
	vector_sub(&mag_delta, &mag);

	// apply CF filter for Mag if delta is acceptable
	float mag_diff = fabs(vector_length(&mag) / vector_length(&estMagGyro) - 1.0f);
	if ( mag_diff < mag_tolerate)
	{
		vector mag_f = mag;
		vector_multiply(&mag_f, alpha2);
		vector_multiply(&estMagGyro, 1-alpha2);
		vector_add(&estMagGyro, &mag_f);
		mag_tolerate = 0.25f;
	}
	else
	{
		mag_tolerate += 0.05f * dt;
		TRACE("warning: possible magnetic interference");
	}

	// apply CF filter for Acc if g force is acceptable
	float acc_g = vector_length(&accel);
	if (acc_g > 0.90f && acc_g < 1.10f)
	{
		// 0.05 low pass filter for acc reading
		const float RC = 1.0f/(2*3.1415926 * 0.05f);
		float alpha = dt / (dt + RC);


		vector acc_f = accel;
		vector_multiply(&acc_f, alpha);
		vector_multiply(&estAccGyro, 1-alpha);
		vector_add(&estAccGyro, &acc_f);
	}
	else
	{
		TRACE("rapid movement (%fg, angle=%f)", acc_g, acos(vector_angle(&estAccGyro, &acc)) * 180 / PI );
	}

	float dot = accel.V.x * estAccGyro.V.x + accel.V.y * estAccGyro.V.y + accel.V.z * estAccGyro.V.z;
	dot /= vector_length(&estAccGyro);
	accelz_mwc = G_in_ms2 * (dot-1);
	// calculate attitude, unit is radian, range +/-PI
	roll = radian_add(atan2(estAccGyro.V.x, estAccGyro.V.z), PI);
	pitch = atan2(estAccGyro.V.y, (estAccGyro.V.z > 0 ? 1 : -1) * sqrt(estAccGyro.V.x*estAccGyro.V.x + estAccGyro.V.z * estAccGyro.V.z));
	pitch = radian_add(pitch, PI);
	vector estAccGyro16 = estAccGyro;
	vector_divide(&estAccGyro16, 4);
	float xxzz = (estAccGyro16.V.x*estAccGyro16.V.x + estAccGyro16.V.z * estAccGyro16.V.z);
	float G = sqrt(xxzz+estAccGyro16.V.y*estAccGyro16.V.y);
	yaw_mag = atan2(estMagGyro.V.x * estAccGyro16.V.z - estMagGyro.V.z * estAccGyro16.V.x ,
		((estMagGyro.V.x * estAccGyro16.V.x + estMagGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y  - estMagGyro.V.y * xxzz )/G);
	yaw_gyro = atan2(estGyro.V.x * estAccGyro16.V.z - estGyro.V.z * estAccGyro16.V.x,
		((estGyro.V.x * estAccGyro16.V.x + estGyro.V.z * estAccGyro16.V.z) *estAccGyro16.V.y - estGyro.V.y * xxzz )/G);

	accel_ef = accel;
	float attitude[3] = {roll, pitch, radian_add(yaw_mag, PI)};
	vector_rotate2(&accel_ef, attitude);

	TRACE("\raccel_ef:%.1f, %.1f, %.1f / %.1f,%.1f,%.1f, accelz:%.2f/%.2f, yaw=%.2f", accel_earth_frame_mwc.V.x*9.8f, accel_earth_frame_mwc.V.y*9.8f, accel_earth_frame_mwc.V.z*9.8f,
		accel_earth_frame.V.x, accel_earth_frame.V.y, accel_earth_frame.V.z, accelz,
		accelz_mwc, (accel_earth_frame_mwc.V.z+2085)/2085*9.80, yaw_est*180/PI);

	return 0;
}
int ahrs_mwc_init(vector gyro_bias, vector accel, vector mag)
{
	::gyro_bias = gyro_bias;
	estAccGyro = accel;
	estGyro= estMagGyro = mag;
	vector_multiply(&::gyro_bias, -1);

	return 0;
}