#ifndef __MS5611_H__
#define __MS5611_H__

#ifdef __cplusplus
extern "C" {
#endif

int init_MS5611(void);
int read_MS5611(int *data);
int check_MS5611(void);			// check for MS5611 healthy, re-plug and init it if possible
								// return : 0 if hardware error happend, 
								//			-1 if hardware error resolved
								//			1 if everything OK


#ifdef __cplusplus
}
#endif

#endif
