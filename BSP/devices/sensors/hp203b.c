#include "MS5611.h"
#include "../common/I2C.h"
#include "../common/common.h"

// registers of the device
#define ADDR   0xEE
#define _WRITE_REG 0xC0
#define _READ_REG 0x80

#define SOFT_RESET 0x06
#define ADC_CVT 0x40
#define READ_PT 0x10
#define READ_AT 0x11
#define READ_P 0x30
#define READ_A 0x31
#define READ_T 0x32
#define ANALOG_CALIBRATION 0x28

#define ALF_OFF_LSB 0x00
#define ALF_OFF_MSB 0x01
#define INT_SRC 0x0D

#define T_RDY 0x10
#define PA_RDY 0x20

typedef enum
{
	OSR4096 = 0<<2,
	OSR2048 = 1<<2,
	OSR1024 = 2<<2,
	OSR512 = 3<<2,
	OSR256 = 4<<2,
	OSR128 = 5<<2,
} HP203_OSR;

typedef enum
{
	CHANNEL_PRESSURE_AND_TEMPERATURE = 0,
	TEMPERATURE = 2,
} HP203_CHANNEL;

static int64_t last_convert = 0;

// call initI2C before this
int init_hp203b(void)
{
	int r;
	if (I2C_WriteReg(ADDR, SOFT_RESET, 0) < 0)
		return -1;
	delayms(100);
	
	r = I2C_WriteReg(ADDR, ADC_CVT + OSR4096 + CHANNEL_PRESSURE_AND_TEMPERATURE, 0);
	last_convert = getus();
	
	return 0;
}

int check_hp203b(void)
{
	return 1;
}

int read_hp203b(int *data)
{
	int rtn = 1;
	uint8_t tmp[8] = {0};

	// check if data ready
	I2C_ReadReg(ADDR, INT_SRC, tmp, 1);
	LOGE("tmp=%02x\n", tmp[0]);
	if (tmp[0] == 0x40)
	{
		// read out data
// 		I2C_ReadReg(ADDR, READ_P, tmp+1, 3);
// 		tmp[0] = tmp[1] & 0x80 ? 0xff : 0x00;
// 		data[0] = *(int*)tmp;
// 
// 		I2C_ReadReg(ADDR, READ_T, tmp+1, 3);
// 		tmp[0] = tmp[1] & 0x80 ? 0xff : 0x00;
// 		data[1] = *(int*)tmp;
// 
// 		// start new data converting
		I2C_WriteCmd(ADDR, ADC_CVT + OSR4096 + CHANNEL_PRESSURE_AND_TEMPERATURE);
		last_convert = getus();

		delayms(100);

		I2C_ReadReg(ADDR, READ_PT, tmp+2, 4);
		data[0] = *(int*)tmp;
		tmp[1] = tmp[2] & 0x80 ? 0xff : 0x00;
		data[1] = *(int*)(tmp+1);

	}
	else
	{
// 		// time out, start new data converting
// 		if (getus() - last_convert > 100000)
// 		{
// 			I2C_WriteReg(ADDR, ADC_CVT + OSR4096 + CHANNEL_PRESSURE_AND_TEMPERATURE, 0);
// 			last_convert = getus();
// 		}
// 		return -1;
	}
	
	return 0;
}
