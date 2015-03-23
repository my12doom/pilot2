#ifndef __HP203B_H__
#define __HP203B_H__

#ifdef __cplusplus
extern "C" {
#endif

int init_hp203b(void);
int read_hp203b(int *data);
int check_hp203b(void);			// check for MS5611 healthy, re-plug and init it if possible
								// return : 0 if hardware error happend, 
								//			-1 if hardware error resolved
								//			1 if everything OK


#ifdef __cplusplus
}
#endif

#endif
