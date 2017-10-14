#include <stdio.h>
#include <conio.h>
#include <stdint.h>
#include <protocol/RFData.h>
#include <Algorithm/ekf_estimator.h>
#include <Algorithm/ahrs.h>
#include <HAL/Interface/IGPS.h>
#include <algorithm/pos_estimator2.h>
#include <algorithm/ekf_ahrs.h>
#include <algorithm/EKFINS.h>
#include <Windows.h>
#include <math/LowPassFilter2p.h>
#include <math/median_filter.h>
#include <math/quaternion.h>
#include <algorithm/motion_detector.h>
#include <Algorithm/battery_estimator.h>
#include <HAL/sensors/PX4flow.h>

using namespace sensors;

#define PI 3.1415926
math::LowPassFilter2p lpf2p[3];//
math::LowPassFilter2p lpf_ned[3];
MedianFilter<7> mf[2];

motion_detector motion_acc;
motion_detector motion_gyro;
static float last_valid_sonar = 0.0f;

int compare_sat(const void *v1, const void*v2)
{
	SAT *p1 = (SAT*) v1;
	SAT *p2 = (SAT*) v2;

	return p1->cno < p2->cno ? 1 : -1;
}

float weights[] = 
{
	0.0631297580000000,
	0.0654184550000000,
	0.111688605000000,
	0.152938067000000,
	0.190829723000000,
	0.218888877000000,
	0.231410108000000,
	0.224725340000000,
	0.197890975000000,
	0.153378354000000,
	0.0965476750000000,
	0.0347751970000000,
	-0.0239214040000000,
	-0.0723772250000000,
	-0.105478283000000,
	-0.121162892000000,
	-0.120233530000000,
	-0.106095926000000,
	-0.0837842500000000,
	-0.0504882480000000,
	-0.0580793770000000,
};


float sq(float v)
{
	return v*v;
}

int quat_test()
{
	float q_t[4];
	float q[4];
	float q_error[4];

	float euler_t[3] = {4*PI/180, -90 * PI / 180, 0*PI/3};
	float euler[3] = { 3*PI/180, 0 * PI / 180, 0*PI/3};
	float body_frame_error[3];

	RPY2Quaternion(euler, q);
	RPY2Quaternion(euler_t, q_t);

	quat_inverse(q_t);
	quat_mult(q_t, q, q_error);
	quat_inverse(q_error);
	Quaternion2BFAngle(q_error, body_frame_error);

	float euler2[3];
	Quaternion2RPY(q_t, euler2);
	float q_t2[4];
	RPY2Quaternion(euler2, q_t2);

	float q1 = q_error[0];
	float q2 = q_error[1];
	float q3 = q_error[2];
	float q4 = q_error[3];

	//float roll = atan2f(2.0f*(q1*q2 + q3*q4), 1 - 2.0f*(q2*q2 + q3*q3));
	float len = sqrt(sq(q2)+sq(q3)+sq(q4));
	if (len >= 1.0e-12f) {
		float invl = 1.0f/len;
		float qp = radian_add(2.0f * atan2f(len,q1), 0);
		q2 *= qp * invl;
		q3 *= qp * invl;
		q4 *= qp * invl;
	}


	return 0;
}

int test_batt2()
{
	FILE * f = fopen("J:\\0011.dat.txt.csv", "rb");

	if (!f)
		return -1;

	FILE * o = fopen("Z:\\bat0.csv", "wb");
	if (!o)
	{
		fclose(f);
		return -2;
	}

	fprintf(o, "t,vpredict,vint,v,P,res,I\r\n");
	char tmp[200];
	fgets(tmp, 200, f);
	float t = 0;
	float last_t = 0;
	float v1,v2,v3,I,mah,bal;

	battery_estimator est[3];

	while(fscanf(f, "%f,%f,%f,%f,%f,%f,%f", &t, &v1,&v2,&v3,&I,&mah,&bal) == 7)		
	{
		float dt = t - last_t;
		last_t = t;

		float Icell[3] = {-I,-I,-I};
		if (bal >= 0)
		{
			if (bal == 0)
				Icell[0] += v1/20.0;
			if (bal == 1)
				Icell[1] += v2/20.0;
			if (bal == 4)
				Icell[2] += v3/20.0;
		}

		est[0].update(v2, Icell[1], dt);
// 		est[1].update(v2, -I, dt);
// 		est[2].update(v3, -I, dt);

		fprintf(o, "%.3f,%.3f,%.3f,%.3f,%.6f,%.3f,%.3f\n", t, est[0].hx1, est[0].get_internal_voltage(), v2, sqrt(est[0].P[3]), est[0].get_internal_resistance()*1000, Icell[1]);
	};

	printf("mah=%.3f\n", est[0].get_mah_consumed());

	fclose(f);
	fclose(o);
	return 0;
}


int main(int argc, char* argv[])
{

	lpf2p[0].set_cutoff_frequency(1000, 60);
	lpf2p[1].set_cutoff_frequency(1000, 60);
	lpf2p[2].set_cutoff_frequency(1000, 60);
	lpf_ned[0].set_cutoff_frequency(200, 5);
	lpf_ned[1].set_cutoff_frequency(200, 5);
	lpf_ned[2].set_cutoff_frequency(200, 5);

	double scaling = (double)1013.26 / 1013.25;
	double temp = ((float)40) + 273.15f;
	double a_raw_altitude = 153.8462f * temp * (1.0f - exp(0.190259f * log(scaling)));
	battery_estimator batt_est;

	motion_acc.set_threshold(1);
	motion_gyro.set_threshold(5*PI/180);

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
	FILE *excel = fopen("Z:\\flow.csv", "wb");
	if (excel)
		fprintf(excel, "t,x,y\r\n");

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

	float az = 0;
	float az_raw = 0;

	int64_t time = 0;
	ppm_data ppm = {0};
	gps_data gps = {0};
	devices::gps_data gps_extra = {0};
	gps_extra.position_accuracy_horizontal = 1000;
	imu_data imu = {0};
	sensor_data sensor = {0};
	quadcopter_data quad = {0};
	quadcopter_data2 quad2 = {0};
	quadcopter_data3 quad3 = {0};
	quadcopter_data4 quad4 = {0};
	quadcopter_data5 quad5 = {0};
	pilot_data pilot = {0};
	pilot_data2 pilot2 = {0};
	bool sensor_valid = false;
	bool sensor2_valid = false;
	bool imu_valid = false;
	bool gps_valid = true;
	bool home_set = false;
	double home_lat = 0;
	double home_lon = 0;
	double lat_meter = 0;
	double lon_meter = 0;
	double speed_north = 0;
	double speed_east = 0;
	float on_pos2[13] = {0};
	px4flow_frame frame = {0};
	rc_mobile_data mobile = {0};
	float gyro[3] = {0};
	float att_dbg[4] = {0};

	int ssss = sizeof(frame);
	ekf_estimator ekf_est;
	pos_estimator2 pos2;
	ekf_ahrs ahrs;
	EKFINS ins;
	SAT sats[40];
	SAT_header sat={0};
	float avg_cno = 0;
	float top6_cno = 0;
	posc_ext_data posc = {0};
	float batt_state_on[2] = {0};
	float flow_on[3] = {0};
	float flow_on_mf[3] = {0};

	while (fread(&time, 1, 8, in) == 8)
	{
		uint8_t tag = time >> 56;
		uint16_t tag_ex = 0xffff;
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
		if (tag == TAG_MOBILE_DATA)
		{
			memcpy(&mobile, data, sizeof(mobile));
		}

		if (tag_ex == 17 && size == 8)
		{
			memcpy(batt_state_on, data, 8);
		}


		if (tag_ex == TAG_EXTRA_GPS_DATA)
		{
			memcpy(&gps_extra, data, size);
		}
		if (tag_ex == TAG_ATTITUDE_CONTROLLER_DATA)
		{
			if (size == 16)
				memcpy(&att_dbg, data, size);
		}
		if (tag == TAG_PX4FLOW_DATA)
		{
			memcpy(&frame, data, sizeof(frame));
		}
		if (tag == TAG_QUADCOPTER_DATA2)
		{
			memcpy(&quad2, data, size);
		}
		if (tag == TAG_PILOT_DATA)
		{
			memcpy(&pilot, data, sizeof(pilot));
		}
		if (tag == TAG_PILOT_DATA2)
		{
			memcpy(&pilot2, data, sizeof(pilot));
		}
		if (tag == TAG_QUADCOPTER_DATA3)
		{
			memcpy(&quad3, data, size);
		}
		if (tag == TAG_QUADCOPTER_DATA4)
		{
			memcpy(&quad4, data, sizeof(quad4));
		}
		if (tag == TAG_QUADCOPTER_DATA5)
		{
			memcpy(&quad5, data, size);
		}
		if (tag == TAG_QUADCOPTER_DATA)
		{
			memcpy(&quad, data, sizeof(quad));
		}
		if (tag_ex == TAG_POS_ESTIMATOR2 && (size == 48 || size == 52) )
		{
			memcpy(on_pos2, data, size);
		}
		if (tag_ex == TAG_POSC_DATA)
		{
			memcpy(&posc, data, size);
		}
		if (tag_ex == TAG_FLOW)
		{
			memcpy(flow_on, data, size);

			flow_on_mf[0] = mf[0].apply(flow_on[0]);
			flow_on_mf[1] = mf[1].apply(flow_on[1]);
		}
		if (tag_ex == 5)
		{
			sensor2_valid = true;

			int16_t *p = (int16_t*) data;
			for(int i=0; i<3; i++)
				gyro[i] = lpf2p[i].apply(p[i+3]) * PI / 1800;

			az_raw = -p[2]/100.0f;
		}

		else if (tag_ex == TAG_UBX_SAT_DATA)
		{
			memcpy(&sat, data, size);
			memcpy(sats, data+8, size-8);
			qsort(sats, sat.num_sat_visible, sizeof(SAT), compare_sat);

			avg_cno = 0;
			int used_count = 0;
			for(int i=0; i<sat.num_sat_visible; i++)
			{
				if (sats[i].flags & 0x8)
				{
					avg_cno += sats[i].cno;
					used_count++;
				}
			}

			avg_cno /= used_count;

			if (sat.num_sat_visible >=6)
				top6_cno = (sats[0].cno + sats[1].cno + sats[2].cno + sats[3].cno + sats[4].cno + sats[5].cno)/6.0;
			else
				top6_cno = 0;
		}
		else if (tag == TAG_SENSOR_DATA)
		{
			memcpy(&sensor, data, sizeof(sensor));
			sensor_valid = true;

			if (time > 5000000)
			{
				batt_est.update(sensor.voltage/1000.0f, sensor.current/1000.0f, 0.005f);
				int throttle = (ppm.out[0] + ppm.out[1] + ppm.out[2] + ppm.out[3])/4;
				static int power_id = 0;
				if (power_id++ % 10 == 0)
					fprintf(out, "CURR, %d, %d, %f, %f, %f, %f, %f,%d,\r\n", int(time/1000), throttle, sensor.voltage/1000.0f, sensor.current/1000.0f, imu.temperature/1.0f, batt_state_on[0], batt_state_on[1]*1000, pilot.mah_consumed);
			}

			static int ll = 0;
			static FILE * bias = fopen("temp-bias.csv", "wb");
			if (ll ++ % 25 == 0)
			fprintf(bias, "%f,%f,%f,%f,%f\n", time/1000000.0f, (float)sensor.gyro[0]/100.0f, (float)sensor.gyro[1]/100.0f, 
				(float)sensor.gyro[2]/100.0f, (sensor.temperature1+10000)/100.0f);
		}
		else if (tag == TAG_IMU_DATA)
		{
			memcpy(&imu, data, sizeof(imu));

			static int64_t last_time = 0;
			float dt = (time - last_time)/1000000.0f;
			last_time = time;

			if (dt > 0.007)
				printf("dt=%f @ %f\n", dt, time/1000000.0f);

			float acc[3] = {-imu.accel[0]/100.0f, -imu.accel[1]/100.0f, -imu.accel[2]/100.0f};
			float mag[3] = {-imu.mag[0]/10.0f, -imu.mag[1]/10.0f, imu.mag[2]/10.0f};
 			float gyro[3] = {sensor.gyro[0]*PI/18000, sensor.gyro[1]*PI/18000, sensor.gyro[2]*PI/18000};

			static FILE * facc = NULL;
			if (!facc)
			{
				facc = fopen("Z:\\acc.csv", "wb");
				if (facc)
					fprintf(facc, "x,y,z\r\n");
			}
			vector va = {acc[0], acc[1], acc[2]};
			vector vg = {gyro[0], gyro[1], gyro[2]};
			vector la;
			vector lg;
			int lca = motion_acc.get_average(&la);
			int lcg = motion_gyro.get_average(&lg);
			motion_acc.new_data(va);
			motion_gyro.new_data(vg);

			bool still = motion_acc.get_average(NULL)>100 && motion_gyro.get_average(NULL)>100;
			static bool last_still = false;
			if (last_still && !still)
			{
				printf("still : %.4f, %.4f, %.4f, %d\n", lg.array[0], lg.array[1], lg.array[2], lca);
				
				if (facc)
					fprintf(facc, "%.4f, %.4f, %.4f\r\n", la.array[0], la.array[1], la.array[2]);				
			}
			last_still = still;

			float q0 = quad5.q[0];
			float q1 = quad5.q[1];
			float q2 = quad5.q[2];
			float q3 = quad5.q[3];

			float pitch_cf = -asinf(2.f * (q1*q3 - q0*q2));
			float roll_cf = atan2f(2.f * (q2*q3 + q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3);;
			float yaw_cf = atan2f(2.f * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);		//! Yaw, 0 = north, PI/-PI = south, PI/2 = east, -PI/2 = west
			float roll_raw = atan2(-acc[1], -acc[2]) * 180 / 3.14159;
			float pitch_raw = atan2(acc[0], (-acc[2] > 0 ? 1 : -1) * sqrt(acc[1]*acc[1] + acc[2]*acc[2])) * 180 / 3.14159;
			float magX = mag[0] * cos(roll_cf) + mag[1] * sin(roll_cf) * sin(pitch_cf) + mag[2] * cos(roll_cf) * sin(pitch_cf);

			float magY = mag[1] * cos(roll_cf) - mag[2] * sin(roll_cf);

			float yaw_raw = atan2f(-magY, magX) * 180 / PI;

			//

// 			if (time < 9500000)
// 				continue;

// 			if (time > 215000000)
// 			{
// 				gyro[0] += 5.5 * PI / 180 * min(time - 215000000, 60000000) / 60000000;
// 				acc[2] += 1.5* min(time - 215000000, 60000000) / 60000000;
// 			}


			if (!imu_valid && sensor_valid && sensor2_valid)
			{
				imu_valid = true;

				NonlinearSO3AHRSinit(acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], gyro[0], gyro[1], gyro[2]);
				ekf_est.init(acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], gyro[0], gyro[1], gyro[2]);
			}

			if (imu_valid && sensor_valid && sensor2_valid && gps_valid && dt < 1)
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

				// experimental EKF
				ahrs.update(acc, gyro, mag, dt, true);

				// EKF
				EKF_U ekf_u;
				EKF_Mesurement ekf_mesurement = {0};

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

				devices::gps_data gps_extra2 = gps_extra;

// 					gps_extra2.position_accuracy_horizontal = 15;
				if (time > 183000000 && time < 200000000)
				{
// 					gps_extra2.latitude += (time/1000000.0f - 200) * 5 * (360/40007000.0f);
				}

				if (time > 200000000)
				{
// 					gps_extra2.latitude += 205 * (360/40007000.0f);
// 					gps_extra2.longitude += 205 * (360/40007000.0f);

				}


				memcpy(pos2.gyro, gyro, sizeof(gyro));
// 				memcpy(&pos2.frame, &frame, sizeof(frame));
				float baro = quad2.altitude_baro_raw/100.0f;

				static int last_state = 0;
				sensors::flow_data fdata = {flow_on_mf[0], flow_on_mf[1], flow_on[2]};

				float sonar = quad3.ultrasonic / 1000.0f;
 				pos2.update(quad5.q, acc, gps_extra2,fdata, sonar, baro, dt, pilot.fly_mode, true, still);
				if (last_state != pos2.state())
				{
					last_state = pos2.state();
					printf("pos2: %d\n", last_state);
					printf("pos2: %d\n", last_state);
				}


				//ins.update(gyro, acc, mag, gps_extra, *(sensors::px4flow_frame*)&frame, baro, dt, ppm.out[0]>=1240, quad2.airborne);

				if (time > 12000000)
				{

					float P[7];
					for(int p=0; p<7; p++)
						P[p] = ahrs.P[p*8];

					printf("");
				}
			}


			static bool fmt = false;
			if (!fmt)
			{
				fmt = true;
				fprintf(out, "ArduCopter V3.1.5 (3c57e771)\r\n");
				fprintf(out, "Free RAM: 990\r\n");
				fprintf(out, "MoMo\r\n");
				fprintf(out, "FMT, 9, 23, CURR, IhIhh, TimeMS,ThrOut,Volt,Curr,Temp,EstVol,EstRes,mah\r\n");
				fprintf(out, "FMT, 9, 23, FLOW, Ihhh, TimeMS,flow_x,flow_y,quality\r\n");
				fprintf(out, "FMT, 9, 23, ATT_OFF_CF, IhIh, TimeMS,Roll,Pitch,Yaw\r\n");
				fprintf(out, "FMT, 9, 23, ATT_DBG, IhIh, TimeMS,pitch_t,pitch_rate_t,pitch,pitch_rate,I_pitch\r\n");
				fprintf(out, "FMT, 9, 23, ATT_ON_CF, IhIh, TimeMS,RollOnCF,PitchOnCF,YawOnCF\r\n");
				fprintf(out, "FMT, 9, 23, ATT_OFF_EKF, IhIh, TimeMS,Roll_EKF,Pitch_EKF,Yaw_EKF\r\n");
				fprintf(out, "FMT, 9, 23, COVAR_OFF_EKF, IhIhhhh, TimeMS,q0,q1,q2,q3,PN,PE\r\n");
				fprintf(out, "FMT, 9, 23, ATT_ON, IhIh, TimeMS,RollOn,PitchOn,YawOn,RollDes,PitchDes,YawDes,RollRate,PitchRate,YawRate,RollRateDes,PitchRateDes,YawRateDes\r\n");
				fprintf(out, "FMT, 9, 23, POSITION, Ihhhhhhhhh, TimeMS,N_GPS,E_GPS,N_EKF,E_EKF,VE_EKF,VE_RAW,POS_ACC,VEL_ACC,HDOP\r\n");
				fprintf(out, "FMT, 9, 23, LatLon, IhIhhh, TimeMS,Lat,Lon,RPOS,RVEL,NSAT,AVGCNO,TOP6CNO\r\n");

				fprintf(out, "FMT, 9, 23, ALT, Ihhhhhhh, TimeMS,BARO,ALT_EKF,ALT_ON, ALT_DST,Sonar,DesSonar,CRate,DesCRate\r\n");
				fprintf(out, "FMT, 9, 23, IMU, Ihhhhhh, TimeMS,ACCX,ACCY,ACCZ,GYROX,GYROY,GYROZ, ACCZ_RAW,ACCZ_FIR\r\n");
				fprintf(out, "FMT, 9, 23, POS2, Ihhhhhhhhhhhh, TimeMS,POSN,POSE,POSD,VELN,VELE,VELD, abiasx, abiasy, abiasz, vbiasx, vbiasy, vbiasz, sonar_surface, rawn, rawe,flowx,flowy,preX,preY,hx,hy,hz\r\n");
				fprintf(out, "FMT, 9, 23, ON_POS2, Ihhhhhhhhhhhh, TimeMS,POSN,POSE,POSD,VELN,VELE,VELD, abiasx, abiasy, abiasz, vbiasx, vbiasy, vbiasz, rawn, rawe\r\n");
				fprintf(out, "FMT, 9, 23, ACC_NED, Ihhh, TimeMS,ACC_N, ACC_E, ACC_D, ACC_TOTAL, ACC_N_LPF, ACC_E_LPF, ACC_D_LPF\r\n");
				fprintf(out, "FMT, 9, 23, AHRS_SEKF, Ihhh, TimeMS,ahrs_roll, ahrs_pitch, ahrs_yaw, ahrs_gyro_bias[0], ahrs_gyro_bias[1], ahrs_gyro_bias[2]\r\n");
				fprintf(out, "FMT, 9, 23, ATT_OFF_INS, Ihhh, TimeMS,ins_roll, ins_pitch, ins_yaw, ins_gyro_bias[0], ins_gyro_bias[1], ins_gyro_bias[2], ins_acc_bias[0], ins_acc_bias[1], ins_acc_bias[2], ins_n, ins_e, ine_d, ins_vn, ins_ve, inv_vd, ins_vbn, ins_vbe, ins_vbd\r\n");
				fprintf(out, "FMT, 9, 23, ATT_RAW, Ihhh, TimeMS, roll_raw, pitch_raw, yaw_raw\r\n");
				fprintf(out, "FMT, 9, 23, P, Ihhhhhhhhh, TimeMS, p[0], p[1], p[2], p[3], p[4], p[5], p[6], pmax, plen\r\n");
				fprintf(out, "FMT, 9, 23, P_INS, Ihhhhhhhhh, TimeMS, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15], vbias[0],vbias[1],vbias[2]\r\n");
				fprintf(out, "FMT, 9, 23, P_EKF, Ihhhhhhhhh, TimeMS, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15], vbias[0],vbias[1],vbias[2]\r\n");
				fprintf(out, "FMT, 9, 23, Motor, Ihhhh, TimeMS, m[0], m[1], m[2], m[3]\r\n");
				fprintf(out, "FMT, 9, 23, GPS, Ihhhhh, TimeMS, v[0], v[1], v[2], HDOP, acc_horizontal, acc_vel_hori, NSat, v_horizontal\r\n");
				fprintf(out, "FMT, 9, 23, VerticalLoop, Ihhhhh, TimeMS,desZAcc,DesCLimb,ZAcc,Climb\r\n");
				fprintf(out, "FMT, 9, 23, PosControl, Ihhhhhhhh, TimeMS,Pos[0],Pos[1],velocity[0],velocity[1],setpoint[0],setpoint[1],velocity_setpoint[0],velocity_setpoint[1],accel_taget[0],accel_taget[1], I[0], I[1]\r\n");
				fprintf(out, "FMT, 9, 23, POS2_P, Ihhhhhhhh, TimeMS,p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7],p[8],p[9],p[10],p[11],p[12]\r\n");
				fprintf(out, "FMT, 9, 23, RC, Ihhhhhh, TimeMS, RC[1], RC[2], RC[3], RC[4], RC[5], RC[6]\r\n");
				fprintf(out, "FMT, 9, 23, APP, Ihhhhhh, TimeMS, latency, RC[1], RC[2], RC[3], RC[4]\r\n");
				fprintf(out, "FMT, 9, 23, V_BF, Ihhhhhh, TimeMS, v_bf[0], v_bf[1], v_bf[2],baro-sonar, baro_comp, baro_comped\r\n");
			}



			if (frame.ground_distance)
				last_valid_sonar = frame.ground_distance /1000.0f * 1.15f;
			float acc_ned_lpf[3];
			for(int i=0; i<3; i++)
				acc_ned_lpf[i] = lpf_ned[i].apply(pos2.acc_ned[i]);

			static int att_id = 0;
			if (att_id++ % 5 == 0 && imu_valid && sensor_valid && gps_valid)
			{
				float euler_sekf[3];
				ahrs.get_euler(euler_sekf);
				fprintf(out, "AHRS_SEKF, %d, %f, %f, %f, %f, %f, %f, %d, %d\r\n", int(time/1000), euler_sekf[0] * 180 / PI, euler_sekf[1] * 180 / PI, euler_sekf[2] * 180 / PI, -ahrs.x[4] * 180 / PI, -ahrs.x[5] * 180 / PI, -ahrs.x[6] * 180 / PI, 0, 0);
				float euler_ins[3];
				ins.get_euler(euler_ins);
				fprintf(out, "ATT_OFF_INS, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f,%f,%f\r\n", int(time/1000), euler_ins[0] * 180 / PI, euler_ins[1] * 180 / PI, euler_ins[2] * 180 / PI,
					-ins.x[4] * 180 / PI, -ins.x[5] * 180 / PI, -ins.x[6] * 180 / PI, ins.x[13], ins.x[14], ins.x[15], ins.x[7], ins.x[8], ins.x[9], ins.x[10], ins.x[11], ins.x[12], ins.x[16], ins.x[17], ins.x[18]);
				fprintf(out, "ATT_RAW, %d, %f, %f, %f\r\n", int(time/1000), roll_raw, pitch_raw, yaw_raw);
				float plen = 0;
				float pmax = 0;
				for(int i=0; i<ahrs.P.m; i++)
				{
					plen += ahrs.P[i*8];
					pmax = max(pmax, ahrs.P[i*8]);
				}
				plen = sqrt(plen);
				pmax = sqrt(pmax);



				fprintf(out, "P, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n", int(time/1000), sqrt(ahrs.P[0*8]), sqrt(ahrs.P[1*8]), sqrt(ahrs.P[2*8]), sqrt(ahrs.P[3*8]), sqrt(ahrs.P[4*8]), sqrt(ahrs.P[5*8]), sqrt(ahrs.P[6*8]), pmax, plen);
				fprintf(out, "P_INS, %d", int(time/1000));
				for(int i=0; i<ins.P.m; i++)
					fprintf(out,",%f", sqrt(ins.P[(ins.P.m+1)*i]));
				fprintf(out, "\r\n");

				fprintf(out, "P_EKF, %d", int(time/1000));
				for(int i=0; i<13; i++)
					fprintf(out,",%f", sqrt(ekf_est.P[14*i]));
				fprintf(out, "\r\n");

				fprintf(out, "ATT_ON, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n", int(time/1000), quad.angle_pos[0]/100.0f, quad.angle_pos[1]/100.0f, quad.angle_pos[2]/100.0f, quad.angle_target[0]/100.0f, quad.angle_target[1]/100.0f, quad.angle_target[2]/100.0f, quad.speed[0]/100.0f, quad.speed[1]/100.0f, quad.speed[2]/100.0f, quad.speed_target[0]/100.0f, quad.speed_target[1]/100.0f, quad.speed_target[2]/100.0f);
				fprintf(out, "ATT_OFF_CF, %d, %f, %f, %f, %d, %d\r\n", int(time/1000), euler[0] * 180 / PI, euler[1] * 180 / PI, euler[2] * 180 / PI, 0, 0);
				fprintf(out, "ATT_DBG, %d, %f, %f, %f, %f, %f\r\n", int(time/1000), att_dbg[0] * 180 / PI, att_dbg[1] * 180 / PI, att_dbg[2] * 180 / PI, att_dbg[3]*180/PI, pilot2.I[1]*PI/18000);
				fprintf(out, "ATT_ON_CF, %d, %f, %f, %f, %d, %d\r\n", int(time/1000), roll_cf * 180 / PI, pitch_cf * 180 / PI, yaw_cf * 180 / PI, 0, 0);
				fprintf(out, "ATT_OFF_EKF, %d, %f, %f, %f, %d, %d\r\n", int(time/1000), ekf_est.ekf_result.roll * 180 / PI, ekf_est.ekf_result.pitch * 180 / PI, ekf_est.ekf_result.yaw * 180 / PI, 0, 0);

				fprintf(out, "COVAR_OFF_EKF, %d, %f, %f, %f, %f, %f, %f, %d, %d\r\n", int(time/1000), ekf_est.P[6*14], ekf_est.P[7*14], ekf_est.P[8*14], ekf_est.P[9*14], sqrt(ekf_est.P[0*14]), sqrt(ekf_est.P[4*14]), 0, 0);
				fprintf(out, "IMU, %d, %f, %f, %f, %f, %f, %f,%f,%f\r\n", int(time/1000), acc[0], acc[1], acc[2], gyro[0]*180/PI, gyro[1]*180/PI, gyro[2]*180/PI, az_raw, az);
// 				if(ekf_est.ekf_is_ready())
				{
					fprintf(out, "POSITION, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d, %d\r\n", int(time/1000), lat_meter, lon_meter, ekf_est.ekf_result.Pos_x, ekf_est.ekf_result.Pos_y, ekf_est.ekf_result.Vel_y, speed_east, gps_extra.position_accuracy_horizontal, gps_extra.velocity_accuracy_horizontal, gps.DOP[1]/100.0f, 0, 0);
					fprintf(out, "ALT, %d, %f, %f, %f, %f,%f,%f,%f,%f\r\n", int(time/1000), quad2.altitude_baro_raw/100.0f, ekf_est.ekf_result.Pos_z, quad2.altitude_kalman/100.0f, quad3.altitude_target/100.0f, quad3.ultrasonic/1000.0f, quad4.sonar_target/100.0f, quad2.climb_rate_kalman/100.0f, quad3.climb_target/100.0f);
					fprintf(out, "POS2, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f,%f,%f, %f, %f,%f,%f,%f,%f,%f,%f\r\n", int(time/1000), pos2.x[0], pos2.x[1], pos2.x[2], pos2.x[3], pos2.x[4], pos2.x[5], pos2.x[6], pos2.x[7], pos2.x[8], pos2.x[9], pos2.x[10], pos2.x[11], pos2.x[12], pos2.gps_north, pos2.gps_east, pos2.vx, pos2.vy, pos2.predict_flow[0], pos2.predict_flow[1], (float)pos2.local[0], (float)pos2.local[1], (float)pos2.local[2]);
					fprintf(out, "POS2_P, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\r\n", int(time/1000), float(pos2.P[0*14]), float(pos2.P[1*14]), float(pos2.P[2*14]), float(pos2.P[3*14]), float(pos2.P[4*14]), float(pos2.P[5*14]), float(pos2.P[6*14]), float(pos2.P[7*14]), float(pos2.P[8*14]), float(pos2.P[9*14]), float(pos2.P[10*14]), float(pos2.P[11*14]), float(pos2.P[12*14]));
					fprintf(out, "ON_POS2, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f,%f,%f\r\n", int(time/1000), on_pos2[0], on_pos2[1], on_pos2[2], on_pos2[3], on_pos2[4], on_pos2[5], on_pos2[6], on_pos2[7], on_pos2[8], on_pos2[9], on_pos2[10], on_pos2[11], pos2.gps_north, pos2.gps_east);
					fprintf(out, "ACC_NED,%d,%f,%f,%f, %f,%f,%f,%f\r\n", int(time/1000), pos2.acc_ned[0], pos2.acc_ned[1], pos2.acc_ned[2], sqrt(pos2.acc_ned[0]*pos2.acc_ned[0] + pos2.acc_ned[1]*pos2.acc_ned[1] + pos2.acc_ned[2]*pos2.acc_ned[2]),
						acc_ned_lpf[0], acc_ned_lpf[1], acc_ned_lpf[2]);
				}
				if (home_set)
				fprintf(out, "LatLon, %d, %f, %f, %f, %f, %d, %f, %f\r\n", int(time/1000), gps_extra.latitude, gps_extra.longitude, 0.0005 * gps_extra.position_accuracy_horizontal * gps_extra.position_accuracy_horizontal, 0.02 * gps_extra.velocity_accuracy_horizontal * gps_extra.velocity_accuracy_horizontal, gps.satelite_in_use, avg_cno, top6_cno);
				fprintf(out, "Motor,%d,%d,%d,%d,%d\r\n", int(time/1000), ppm.out[0], ppm.out[1], ppm.out[2], ppm.out[3]);
				fprintf(out, "GPS,%d,%f,%f,%f,%f,%f,%f,%d,%f\r\n", int(time/1000), speed_north, speed_east, gps_extra.climb_rate, gps_extra.DOP[1]/100.0f, gps_extra.position_accuracy_horizontal, gps_extra.velocity_accuracy_horizontal, gps_extra.satelite_in_use, sqrt(speed_north*speed_north+speed_east*speed_east));
				fprintf(out, "VerticalLoop,%d,%f,%f,%f,%f\r\n", int(time/1000), (float)quad3.accel_target/100.0f, (float)quad3.climb_target/100.0f, pos2.acc_ned[2], pos2.x[5]);
				fprintf(out, "PosControl,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n", int(time/1000), posc.pos[0], posc.pos[1], posc.velocity[0], posc.velocity[1], posc.setpoint[0], posc.setpoint[1],posc.velocity_setpoint[0], posc.velocity_setpoint[1], posc.accel_target[0], posc.accel_target[1], (float)posc.pid[0][1], (float)posc.pid[1][1]);
				fprintf(out, "RC,%d,%d,%d,%d,%d,%d,%d\r\n", int(time/1000), ppm.in[0], ppm.in[1], ppm.in[2], ppm.in[3], ppm.in[4], ppm.in[5]);
				fprintf(out, "APP,%d,%d,%d,%d,%d,%d,%d\r\n", int (time/1000), mobile.latency, mobile.channel[0], mobile.channel[1], mobile.channel[2], mobile.channel[3]);
				fprintf(out, "V_BF,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", int (time/1000), pos2.v_bf[0], pos2.v_bf[1], pos2.v_bf[2], quad2.altitude_baro_raw/100.0f - last_valid_sonar, pos2.baro_comp, quad2.altitude_baro_raw/100.0f - pos2.baro_comp);
				fprintf(out, "FLOW,%d,%.3f,%.3f,%.3f\r\n", int (time/1000), flow_on[0], flow_on[1], flow_on[2]);

				if(excel)
				{
					fprintf(excel, "%f,%.2f,%.2f,%.2f,%.2f\r\n", time/1000000.0f, pos2.v_bf[0], pos2.v_bf[1], pos2.v_bf[2], quad2.altitude_baro_raw/100.0f - pos2.last_valid_sonar);
				}
			}

		}

		else if (tag == TAG_GPS_DATA)
		{
			gps_valid = true;
			memcpy(&gps, data, size);
		}





	}

	for(int i=0; i<sat.num_sat_visible; i++)
		printf("sat:%d, %ddb, residual:%d\n", sats[i].gnss_id, sats[i].cno, sats[i].residual);

	fclose(in);
	fclose(out);
	if(excel)
	fclose(excel);

	return 0;
}