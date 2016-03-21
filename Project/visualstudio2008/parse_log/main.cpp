#include <stdio.h>
#include <conio.h>
#include <stdint.h>
#include <protocol/RFData.h>
#include <Algorithm/ekf_estimator.h>
#include <Algorithm/ahrs.h>
#include <HAL/Interface/IGPS.h>
#include <algorithm/pos_estimator2.h>

#define PI 3.1415926

int main(int argc, char* argv[])
{
	if (argc < 2)		
	{
		printf("usage: parse_log.exe 0001.dat\npress any key to exit...");
		getch();
		return -2;
	}

	char out_name[300];
	sprintf(out_name, "%s.log", argv[1]);

	FILE *in = fopen(argv[1], "rb");
	FILE *out = fopen(out_name, "wb");

	if (!in)
	{
		printf("failed opening %s\n", argv[1]);
		return -1;
	}

	if (!out)
	{
		printf("failed opening %s\n", out_name);
		return -1;
	}

	int64_t time = 0;
	ppm_data ppm = {0};
	gps_data gps;
	devices::gps_data gps_extra = {0};
	imu_data imu = {0};
	sensor_data sensor = {0};
	quadcopter_data quad = {0};
	quadcopter_data2 quad2 = {0};
	quadcopter_data5 quad5 = {0};
	bool sensor_valid = false;
	bool imu_valid = false;
	bool gps_valid = false;
	bool home_set = false;
	double home_lat = 0;
	double home_lon = 0;
	double lat_meter = 0;
	double lon_meter = 0;
	double speed_north = 0;
	double speed_east = 0;

	ekf_estimator ekf_est;
	pos_estimator2 pos2;

	while (fread(&time, 1, 8, in) == 8)
	{
		uint8_t tag = time >> 56;
		uint16_t tag_ex;
		time = time & ~((uint64_t)0xff << 56);
		char data[65536];
		int size = 24;

		if (tag == TAG_EXTENDED_DATA)		// extended variable length data packets
		{
			if (fread(&tag_ex, 1, 2, in) != 2)
				break;
			if (fread(&size, 1, 2, in) != 2)
				break;
			if (fread(data, 1, size, in) != size)
				break;
		}
		else
		{
			size = 24;
			if (fread(data, 1, 24, in) != 24)
				break;
		}

		// handle packet data here..
		if (tag == TAG_PPM_DATA)
		{
			memcpy(&ppm, data, 24);
		}
		if (tag_ex == TAG_EXTRA_GPS_DATA)
		{
			memcpy(&gps_extra, data, size);
		}
		if (tag == TAG_QUADCOPTER_DATA2)
		{
			memcpy(&quad2, data, size);
		}
		if (tag == TAG_QUADCOPTER_DATA5)
		{
			memcpy(&quad5, data, size);
		}
		if (tag == TAG_QUADCOPTER_DATA)
		{
			memcpy(&quad, data, sizeof(quad));
		}
		else if (tag == TAG_SENSOR_DATA)
		{
			sensor_valid = true;
			memcpy(&sensor, data, sizeof(sensor));
			int throttle = (ppm.out[0] + ppm.out[1] + ppm.out[2] + ppm.out[3])/4;
			static int power_id = 0;
			if (power_id++ % 10 == 0)
				fprintf(out, "CURR, %d, %d, %f, %f, %d, %d\r\n", int(time/1000), throttle, sensor.voltage/1000.0f, sensor.current/1000.0f, 0, 0);
		}
		else if (tag == TAG_IMU_DATA)
		{
			memcpy(&imu, data, sizeof(imu));

			static int64_t last_time = 0;
			float dt = (time - last_time)/1000000.0f;
			last_time = time;

			float acc[3] = {-imu.accel[0]/100.0f, -imu.accel[1]/100.0f, -imu.accel[2]/100.0f};
			float mag[3] = {-imu.mag[0]/10.0f, -imu.mag[1]/10.0f, imu.mag[2]/10.0f};
			float gyro[3] = {sensor.gyro[0]*PI/18000, sensor.gyro[1]*PI/18000, sensor.gyro[2]*PI/18000};

			float q0 = quad5.q[0];
			float q1 = quad5.q[1];
			float q2 = quad5.q[2];
			float q3 = quad5.q[3];

			float pitch_cf = -asinf(2.f * (q1*q3 - q0*q2));
			float roll_cf = atan2f(2.f * (q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3);;
			float yaw_cf = atan2f(2.f * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);		//! Yaw, 0 = north, PI/-PI = south, PI/2 = east, -PI/2 = west


			if (!imu_valid && sensor_valid)
			{
				imu_valid = true;

				NonlinearSO3AHRSinit(acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], gyro[0], gyro[1], gyro[2]);
				ekf_est.init(acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], gyro[0], gyro[1], gyro[2]);
			}

			if (imu_valid && sensor_valid && gps_valid && dt < 1)
			{

				// lon/lat to meter
				double lon = gps.longitude / 10000000.0f;
				double lat = gps.latitude / 10000000.0f;
				double latitude_to_meter = 40007000.0f/360;
				double longtitude_to_meter = 40007000.0f/360*cos(lat * PI / 180);
				if (!home_set && gps.DOP[1] < 150)
				{
					home_lon = lon;
					home_lat = lat;
					home_set = true;
				}

				// CF
				float acc_gps_bf[3] = {0};
				float factor = 1.0f;
				float factor_mag = 1.0f;
				NonlinearSO3AHRSupdate(
					acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], gyro[0], gyro[1], gyro[2],
					0.15f*factor, 0.0015f, 0.15f*factor_mag, 0.0015f, dt,
					acc_gps_bf[0], acc_gps_bf[1], acc_gps_bf[2]);


				// EKF
				EKF_U ekf_u;
				EKF_Mesurement ekf_mesurement;

				ekf_u.accel_x=-acc[0];
				ekf_u.accel_y=-acc[1];
				ekf_u.accel_z=-acc[2];
				ekf_u.gyro_x=gyro[0];
				ekf_u.gyro_y=gyro[1];
				ekf_u.gyro_z=gyro[2];
				ekf_mesurement.Mag_x=-mag[0];
				ekf_mesurement.Mag_y=-mag[1];
				ekf_mesurement.Mag_z=mag[2];

				ekf_mesurement.Pos_Baro_z=quad2.altitude_baro_raw/100.0f;

				if (home_set)
				{
				float yaw_gps = gps.direction * 2 * PI / 360.0f;
				speed_north = cos(yaw_gps) * gps.speed / 100.0f;
				speed_east = sin(yaw_gps) * gps.speed / 100.0f;

				lon_meter = (lon - home_lon) * longtitude_to_meter;
				lat_meter = (lat - home_lat) * latitude_to_meter;
				}

				if(gps_extra.position_accuracy_horizontal < 10 && home_set /*&& time > 500000000*/)
				{
					ekf_mesurement.Pos_GPS_x=lat_meter;
					ekf_mesurement.Pos_GPS_y=lon_meter;
					ekf_mesurement.Vel_GPS_x=speed_north;
					ekf_mesurement.Vel_GPS_y=speed_east;
					ekf_est.set_mesurement_R(0.0005 * gps_extra.position_accuracy_horizontal * gps_extra.position_accuracy_horizontal, 0.02 * gps_extra.velocity_accuracy_horizontal * gps_extra.velocity_accuracy_horizontal);
// 					ekf_est.set_mesurement_R(0.001 * gps_extra.position_accuracy_horizontal * gps_extra.position_accuracy_horizontal, 0.01 * gps_extra.velocity_accuracy_horizontal * gps_extra.velocity_accuracy_horizontal);
// 					ekf_est.set_mesurement_R(1e-3,8e-2);
				}
				else
				{
					ekf_mesurement.Pos_GPS_x=0;
					ekf_mesurement.Pos_GPS_y=0;
					ekf_mesurement.Vel_GPS_x=0;
					ekf_mesurement.Vel_GPS_y=0;
 					ekf_est.set_mesurement_R(1E20,5);
				}

				ekf_est.update(ekf_u,ekf_mesurement, dt);
				pos2.update(quad5.q, acc, gps_extra, quad2.altitude_baro_raw/100.0f, dt);
			}


			static bool fmt = false;
			if (!fmt)
			{
				fmt = true;

				fprintf(out, "ArduCopter V3.1.5 (3c57e771)\r\n");
				fprintf(out, "Free RAM: 990\r\n");
				fprintf(out, "MoMo\r\n");
				fprintf(out, "FMT, 9, 23, CURR, IhIh, TimeMS,ThrOut,Volt,Curr\r\n");
				fprintf(out, "FMT, 9, 23, ATT_OFF_CF, IhIh, TimeMS,Roll,Pitch,Yaw\r\n");
				fprintf(out, "FMT, 9, 23, ATT_ON_CF, IhIh, TimeMS,RollOnCF,PitchOnCF,YawOnCF\r\n");
				fprintf(out, "FMT, 9, 23, ATT_OFF_EKF, IhIh, TimeMS,Roll_EKF,Pitch_EKF,Yaw_EKF\r\n");
				fprintf(out, "FMT, 9, 23, COVAR_OFF_EKF, IhIhhhh, TimeMS,q0,q1,q2,q3,PN,PE\r\n");
				fprintf(out, "FMT, 9, 23, ATT_ON, IhIh, TimeMS,RollOn,PitchOn,YawOn\r\n");
				fprintf(out, "FMT, 9, 23, POSITION, IhIhhhhhhh, TimeMS,N_GPS,E_GPS,N_EKF,E_EKF,VE_EKF,VE_RAW,POS_ACC,VEL_ACC,HDOP\r\n");
				fprintf(out, "FMT, 9, 23, LatLon, IhIhhh, TimeMS,Lat,Lon,RPOS,RVEL\r\n");

				fprintf(out, "FMT, 9, 23, ALT, Ihhhhhhh, TimeMS,BARO,ALT_EKF,ALT_ON, ALT_OFF2\r\n");
				fprintf(out, "FMT, 9, 23, IMU, Ihhhhhh, TimeMS,ACCX,ACCY,ACCZ,GYROX,GYROY,GYROZ\r\n");
				fprintf(out, "FMT, 9, 23, POS2, Ihhhhhhhhhhhh, TimeMS,POSN,POSE,POSD,VELN,VELE,VELD, abiasx, abiasy, abiasz, vbiasx, vbiasy, vbiasz\r\n");
				fprintf(out, "FMT, 9, 23, ACC_NED, Ihhh, TimeMS,ACC_N, ACC_E, ACC_D\r\n");
			}


			static int att_id = 0;
			if (att_id++ % 10 == 0 && imu_valid && sensor_valid && gps_valid)
			{
				ekf_est.ekf_result.roll;
				fprintf(out, "ATT_ON, %d, %f, %f, %f, %d, %d\r\n", int(time/1000), quad.angle_pos[0]/100.0f, quad.angle_pos[1]/100.0f, quad.angle_pos[2]/100.0f, 0, 0);
				fprintf(out, "ATT_OFF_CF, %d, %f, %f, %f, %d, %d\r\n", int(time/1000), euler[0] * 180 / PI, euler[1] * 180 / PI, euler[2] * 180 / PI, 0, 0);
				fprintf(out, "ATT_ON_CF, %d, %f, %f, %f, %d, %d\r\n", int(time/1000), roll_cf * 180 / PI, pitch_cf * 180 / PI, yaw_cf * 180 / PI, 0, 0);
				fprintf(out, "ATT_OFF_EKF, %d, %f, %f, %f, %d, %d\r\n", int(time/1000), ekf_est.ekf_result.roll * 180 / PI, ekf_est.ekf_result.pitch * 180 / PI, ekf_est.ekf_result.yaw * 180 / PI, 0, 0);

				fprintf(out, "COVAR_OFF_EKF, %d, %f, %f, %f, %f, %f, %f, %d, %d\r\n", int(time/1000), ekf_est.P[6*14], ekf_est.P[7*14], ekf_est.P[8*14], ekf_est.P[9*14], sqrt(ekf_est.P[0*14]), sqrt(ekf_est.P[4*14]), 0, 0);
				fprintf(out, "IMU, %d, %f, %f, %f, %f, %f, %f\r\n", int(time/1000), acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
				if(ekf_est.ekf_is_ready())
				{
					fprintf(out, "POSITION, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d, %d\r\n", int(time/1000), lat_meter, lon_meter, ekf_est.ekf_result.Pos_x, ekf_est.ekf_result.Pos_y, ekf_est.ekf_result.Vel_y, speed_east, gps_extra.position_accuracy_horizontal, gps_extra.velocity_accuracy_horizontal, gps.DOP[1]/100.0f, 0, 0);
					fprintf(out, "ALT, %d, %f, %f, %f, %f\r\n", int(time/1000), quad2.altitude_baro_raw/100.0f, ekf_est.ekf_result.Pos_z, quad2.altitude_kalman/100.0f, pos2.x[2]);
					fprintf(out, "POS2, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n", int(time/1000), pos2.x[0], pos2.x[1], pos2.x[2], pos2.x[3], pos2.x[4], pos2.x[5], pos2.x[6], pos2.x[7], pos2.x[8], pos2.x[9], pos2.x[10], pos2.x[11]);
					fprintf(out, "ACC_NED,%d,%f,%f,%f\r\n", int(time/1000), pos2.acc_ned[0], pos2.acc_ned[1], pos2.acc_ned[2]);
				}
				if (home_set)
				fprintf(out, "LatLon, %d, %f, %f, %f, %f\r\n", int(time/1000), gps_extra.latitude, gps_extra.longitude, 0.0005 * gps_extra.position_accuracy_horizontal * gps_extra.position_accuracy_horizontal, 0.02 * gps_extra.velocity_accuracy_horizontal * gps_extra.velocity_accuracy_horizontal);
			}

		}

		else if (tag == TAG_GPS_DATA)
		{
			gps_valid = true;
			memcpy(&gps, data, size);
		}





	}

	fclose(in);
	fclose(out);

	return 0;
}