#ifndef __HMC5883_H__
#define __HMC5883_H__

#ifdef __cplusplus
extern "C" {
#endif

// call initI2C() and init_MPU6050() before this
int init_HMC5883(void);
int read_HMC5883(short*data);
int check_HMC5883(void);			// check for HMC5883 healthy, re-plug and init it if possible
								// return : 0 if hardware error happend, 
								//			-1 if hardware error resolved
								//			1 if everything OK

#ifdef __cplusplus
}
#endif

#endif
