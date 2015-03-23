#ifndef __HMC5983_H__
#define __HMC5983_H__

#ifdef __cplusplus
extern "C" {
#endif

// call initI2C() and init_MPU6050() before this
int init_HMC5983(void);
int read_HMC5983(short*data);
int check_HMC9883(void);			// check for HMC5883 healthy, re-plug and init it if possible
								// return : 0 if hardware error happend, 
								//			-1 if hardware error resolved
								//			1 if everything OK

#ifdef __cplusplus
}
#endif

#endif
