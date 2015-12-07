#include <stdio.h>
#include <conio.h>
#include <stdint.h>
#include<string.h>
#include <Protocol/RFData.h>
#define PI 3.141592654
int main(int argc, char* argv[])
{
	if (argc < 2)		
	{
		printf("usage: parse_log.exe 0001.dat\npress any key to exit...");
		getch();
		return -2;
	}

	char out_name[300];
	sprintf(out_name, "%s.csv", argv[1]);

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

	//fprintf(out, "ArduCopter V3.1.5 (3c57e771)\r\n");
	//fprintf(out, "Free RAM: 990\r\n");
	//fprintf(out, "MoMo\r\n");
	//fprintf(out, "FMT, 9, 23, CURR, IhIh, TimeMS,ThrOut,Volt,Curr\r\n");
	fprintf(out, "time,gx,gy,gz,ax,ay,az,mx,my,mz,gps_x,gpsy,baro,vx,vy,raw_roll,raw_pitch,raw_yaw,ekf_roll,ekf_pitch,ekf_yaw,ekf_Px,ekf_Py,ekf_Pz,ekf_vx,ekf_vy,ekf_vz\r\n");

	int64_t time = 0;
	ppm_data ppm;
	imu_data imu;
	sensor_data sensor;
	quadcopter_data quad;
	quadcopter_data2 quad2;
	ekf_data ekf;
	while (fread(&time, 1, 8, in) == 8)
	{
		uint8_t tag = time >> 7*8;//The Highest byte of eight byte is tag
		uint16_t tag_ex=-1;
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
		if(tag == TAG_SENSOR_DATA)
		{
			memcpy(&sensor,data,24);
		}
		if (tag == TAG_IMU_DATA)
		{
			memcpy(&imu,data,sizeof(imu));
			//fprintf(out,"%f,%f,%f,%f,%f,%f,%f,%f,%f\n",imu.gyro[0]* PI/18000,imu.gyro[1]* PI/18000,imu.gyro[2]* PI/18000,imu.accel[0]/100.0,imu.accel[1]/100.0,imu.accel[2]/100.0,imu.mag[0]/10.0,imu.mag[1]/10.0,imu.mag[2]/10.0);
			
		}
		if(tag == TAG_QUADCOPTER_DATA2)
		{
			memcpy(&quad2,data,24);
		}
		if(tag_ex == TAG_EKF_DATA)
		{
			memcpy(&ekf,data,sizeof(ekf));
		}
		if (tag == TAG_QUADCOPTER_DATA )
		{
			float a=0;
			memcpy(&quad,data,24);
			fprintf(out,"%f,",time);
			fprintf(out,"%f,%f,%f,%f,%f,%f,%f,%f,%f,",sensor.gyro[0]* PI/18000,sensor.gyro[1]* PI/18000,sensor.gyro[2]* PI/18000,imu.accel[0]/100.0,imu.accel[1]/100.0,imu.accel[2]/100.0,sensor.mag[0]/10.0,sensor.mag[1]/10.0,sensor.mag[2]/10.0);
			fprintf(out,"%f,%f,%f,%f,%f,",ekf.raw_postion_n/100.0f,ekf.raw_postion_e/100.0f,quad2.altitude_baro_raw/100.0f,ekf.raw_speed_n/1000.0f,ekf.raw_speed_e/1000.0f);//To-Do:fill gps data
			fprintf(out,"%f,%f,%f,",quad.angle_pos[0]*(PI/18000),quad.angle_pos[1]*(PI/18000),quad.angle_pos[2]*(PI/18000));
			fprintf(out,"%f,%f,%f,",ekf.angle_pos[0]*(PI/18000),ekf.angle_pos[1]*(PI/18000),ekf.angle_pos[2]*(PI/18000));
			fprintf(out,"%f,%f,%f,%f,%f,%f\r\n",ekf.ekf_postion[0]/100.0f,ekf.ekf_postion[1]/100.0f,ekf.ekf_postion[2]/100.0f,ekf.ekf_speed[0]/1000.0f,ekf.ekf_speed[1]/1000.0f,ekf.ekf_speed[2]/1000.0f);
		}
		else if (tag == TAG_SENSOR_DATA)
		{

			/*
			sensor_data &sensor = *(sensor_data*)data;
			int throttle = (ppm.out[0] + ppm.out[1] + ppm.out[2] + ppm.out[3])/4;
			static int power_id = 0;
			if (power_id++ % 10 == 0)
				fprintf(out, "CURR, %d, %d, %f, %f, %d, %d\r\n", int(time/1000), throttle, sensor.voltage/1000.0f, sensor.current/1000.0f, 0, 0);
				*/
		}
	}
	fclose(in);
	fclose(out);

	return 0;
}