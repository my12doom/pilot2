#ifndef __BMP085_H__
#define __BMP085_H__

#ifdef __cplusplus
extern "C" {
#endif

int init_BMP085(void);
int read_BMP085(int *data);

#ifdef __cplusplus
}
#endif

#endif
