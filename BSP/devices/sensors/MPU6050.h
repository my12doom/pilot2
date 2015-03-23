#ifndef __MPU6050_H__
#define __MPU6050_H__

#ifdef __cplusplus
extern "C" {
#endif

// call initI2C before this
int init_MPU6050(void);
int check_MPU6050(void);		// check for MPU6050 healthy, and replug and init it if possible
								// return : 0 if hardware error happend, 
								//			-1 if hardware error resolved
								//			1 if everything OK

// data[0 ~ 7] :
// accel_x, accel_y, accel_z, raw_temperature, gyro_x, gyro_y, gyro_z
int read_MPU6050(short*data);	
#ifdef __cplusplus
}
#endif

#endif
