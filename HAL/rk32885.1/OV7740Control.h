#ifndef _OV7740_CONTROL
#define _OV7740_CONTROL

#include <stdio.h>
#include <fcntl.h>

#include "OV7740Registers.h"
#include "cameraParameter.h"
#include "camsys_head.h"

#define CAMSYS_MARVIN   "/dev/camsys_marvin"

int write_I2C_cam(int cam_fd,uint8_t slaveAddr,uint8_t regAddr,uint8_t data,uint8_t data_size,uint8_t reg_size);
int read_I2C_cam(int cam_fd,uint8_t slaveAddr,uint8_t regAddr,uint8_t *data,uint8_t data_size,uint8_t reg_size);
int initOV7740();
#endif
