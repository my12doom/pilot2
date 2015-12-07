#include <iostream>
#include "ekf_estimator.h"
#include "init_quaternion_by_euler.h"
#include "init_ekf_matrix.h"
#include "INS_SetState.h"
#include "LinearFG.h"
#include "RungeKutta.h"
#include "INS_CovariancePrediction.h"
#include "INS_Correction.h"
#include "quaternion_to_euler.h"

//caculate loop time
#include "time.h"
using namespace std;

typedef struct{
	float time,gx,gy,gz,ax,ay,az,mx,my,mz,gpsn,gpse,gpsd,gpsvn,gpsve,baro,roll,pitch,yaw,ekf_roll,ekf_pitch,ekf_yaw;
}log_type;
int main(int argc, char* argv[])
{
	ekf_estimator ekf_estimator;
	cout<<"################ EKF offline simulation ################"<<endl;
	FILE* fp=fopen("I:\\0002.DAT.log","rb");
	if(fp!=NULL) 
		fprintf(stdout,"###### Open log success\n");
	else
	{
		fprintf(stderr,"file open error\n");
		getchar();
		return -1;
	}
	char data_title[300];
	fgets(data_title,sizeof(data_title),fp);
	fprintf(stdout,"###### Flight log type: \n%s",&data_title);
	fgets(data_title,sizeof(data_title),fp);//dummy line 
	float time,gx,gy,gz,ax,ay,az,mx,my,mz,gpsn,gpse,gpsd,gpsvn,gpsve,baro,roll,pitch,yaw,ekf_roll,ekf_pitch,ekf_yaw;
	float gyro_zerobias_x,gyro_zerobias_y,gyro_zerobias_z;
	//Read one line to initallize quaternion and store gyro zero bias
	fscanf(fp,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",&time,&gyro_zerobias_x,&gyro_zerobias_y,&gyro_zerobias_z,&ax,&ay,&az,&mx,&my,&mz,&gpsn,&gpse,&gpsd,&gpsvn,&gpsve,&baro,&roll,&pitch,&yaw,&ekf_roll,&ekf_pitch,&ekf_yaw);
	fprintf(stdout,"###### Gyro Zero Bias:\nGyro_zerobias_x:%.4f   Gyro_zerobias_x:%.4f   Gyro_zerobias_x:%.4f\n",gyro_zerobias_x,gyro_zerobias_y,gyro_zerobias_z);
	//fprintf(stderr,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",time,gyro_zerobias_x,gyro_zerobias_y,gyro_zerobias_z,ax,ay,az,mx,my,mz,gpsn,gpse,gpsd,gpsvn,gpsve,baro,roll,pitch,yaw);
	
	
	ekf_estimator.init(-ax,-ay,-az,-mx,-my,mz,gyro_zerobias_x,gyro_zerobias_y,gyro_zerobias_z);

	
	float dT=0.003f;
	fprintf(stderr,"Init dT: %.4fms\n",dT);

	float result_roll, result_pitch, result_yaw;

	//Create a file to log ekf result
	FILE * fp_log=fopen("Z:\\ekf_result.csv","wb");
	fprintf(fp_log,"time,Pn,Pe,Pd,Vn,Ve,Vd,Roll,Pitch,Yaw,sqrt_variance_roll,sqrt_variance_pitch,sqrt_variance_yaw,is_convergence\n");
	
	//Iteration
	long log_record=0;

	log_type * data= new log_type[105536];
	fprintf(stdout,"###### Load flight data to memory wait ......\n");
	while (!feof(fp))
	{
		//Read one record
		fscanf(fp,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",&data[log_record].time,&data[log_record].gx,&data[log_record].gy,&data[log_record].gz,&data[log_record].ax,&data[log_record].ay,&data[log_record].az,&data[log_record].mx,&data[log_record].my,&data[log_record].mz,&data[log_record].gpsn,&data[log_record].gpse,&data[log_record].gpsd,&data[log_record].gpsvn,&data[log_record].gpsve,&data[log_record].baro,&data[log_record].roll,&data[log_record].pitch,&data[log_record].yaw,&data[log_record].ekf_roll,&data[log_record].ekf_pitch,&data[log_record].ekf_yaw);
		//fprintf(stdout,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",time,gx,gy,gz,ax,ay,az,mx,my,mz,gpsn,gpse,gpsd,gpsvn,gpsve,baro,roll,pitch,yaw);
		log_record++;
	}
	fprintf(stdout,"###### Load finish!\n");
	fprintf(stdout,"###### EKF Loop Iteration...\n");
	fprintf(stdout,"###### Log ekf type:\ntime,Pn,Pe,Pd,Vn,Ve,Vd,Roll,Pitch,Yaw,sqrt_variance_roll,sqrt_variance_pitch,sqrt_variance_yaw,is_convergence\n");
	//caculate loop time
	clock_t start,end;  
	start = clock();

	//EKF estimator update
	EKF_U ekf_u;
	EKF_Mesurement ekf_mesurement;
	for(int 
		i=0;i<log_record;i++)
	{
		//fprintf(stdout,"%d\n",i);
		ekf_u.accel_x=data[i].ax;
		ekf_u.accel_y=data[i].ay;
		ekf_u.accel_z=data[i].az;
		ekf_u.gyro_x=data[i].gx;
		ekf_u.gyro_y=data[i].gy;
		ekf_u.gyro_z=data[i].gz;
		ekf_mesurement.Mag_x=data[i].mx;
		ekf_mesurement.Mag_y=data[i].my;
		ekf_mesurement.Mag_z=data[i].mz;
		ekf_mesurement.Pos_GPS_x=0;//data[i].gpsn;
		ekf_mesurement.Pos_GPS_y=0;//data[i].gpse;
		ekf_mesurement.Pos_Baro_z=data[i].baro;
		ekf_mesurement.Vel_GPS_x=0;//data[i].gpsvn;
		ekf_mesurement.Vel_GPS_y=0;//data[i].gpsve;

		ekf_estimator.update(ekf_u,ekf_mesurement,0.003);

		quaternion_to_euler(1,ekf_estimator.ekf_result.q0,ekf_estimator.ekf_result.q1,ekf_estimator.ekf_result.q2,ekf_estimator.ekf_result.q3,&result_roll,&result_pitch,&result_yaw);

		//Write result to ekf_result.csv
		fprintf(fp_log,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",time,ekf_estimator.ekf_result.Pos_x,ekf_estimator.ekf_result.Pos_y,ekf_estimator.ekf_result.Pos_z,ekf_estimator.ekf_result.Vel_x,ekf_estimator.ekf_result.Vel_y,ekf_estimator.ekf_result.Vel_z,ekf_estimator.ekf_result.roll,ekf_estimator.ekf_result.pitch,ekf_estimator.ekf_result.yaw,ekf_estimator.sqrt_variance[0],ekf_estimator.sqrt_variance[1],ekf_estimator.sqrt_variance[2],ekf_estimator.ekf_is_convergence);
	}
	end = clock();  
	fprintf(stdout,"###### Caculate Algorithm cycletime\n",log_record);
	fprintf(stdout,"Record number:%d\n",log_record);
	fprintf(stdout,"Total time is: %.3fs\tCycletime is: %.3fms\n",((double)end-start)/CLK_TCK,((double)end-start)/CLK_TCK/log_record*1000);  
	fprintf(stdout,"###### Finish!\n\nPress Enter to end ...\n",log_record);

	fclose(fp_log);
	fclose(fp);
	getchar();
	
}
