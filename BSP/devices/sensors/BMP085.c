#include "BMP085.h"
#include "../common/I2C.h"
#include "../common/common.h"

// registers of the device
#define	BMP085_SlaveAddress 0xee
#define OSS 0				// Oversampling Setting
#define SAMPLEING_TIME 5000	// 5000us, 5ms, modify this to OSS setting

short ac1;
short ac2; 
short ac3; 
unsigned short ac4;
unsigned short ac5;
unsigned short ac6;
short b1; 
short b2;
short mb;
short mc;
short md;

// temp variables
long x1, x2, b5, b6, x3, b3, p;
unsigned long b4, b7;



static int temperature = 0;
static int pressure = 0;
static int new_temperature = 0;
static int64_t last_temperature_time = 0;
static int64_t last_pressure_time = 0;
static int rawTemperature = 0;
static int rawPressure = 0;

static short I2C_Double_Read(unsigned char slave_address, unsigned char resigter_address, int16_t *data)
{
	unsigned char tmp[2];
	if (I2C_ReadReg(slave_address, resigter_address, tmp, 2) < 0)
		return -1;

	*data = (tmp[0] << 8) | tmp[1];

	return 0;
}

#define RET_IF(x) if((x)<0) return -1

// call initI2C before this
int init_BMP085(void)
{
	RET_IF(I2C_Double_Read(BMP085_SlaveAddress, 0xAA, &ac1));
	RET_IF(I2C_Double_Read(BMP085_SlaveAddress, 0xAC, &ac2));
	RET_IF(I2C_Double_Read(BMP085_SlaveAddress, 0xAE, &ac3));
	RET_IF(I2C_Double_Read(BMP085_SlaveAddress, 0xB0, &ac4));
	RET_IF(I2C_Double_Read(BMP085_SlaveAddress, 0xB2, &ac5));
	RET_IF(I2C_Double_Read(BMP085_SlaveAddress, 0xB4, &ac6));
	RET_IF(I2C_Double_Read(BMP085_SlaveAddress, 0xB6, &b1));
	RET_IF(I2C_Double_Read(BMP085_SlaveAddress, 0xB8, &b2));
	RET_IF(I2C_Double_Read(BMP085_SlaveAddress, 0xBA, &mb));
	RET_IF(I2C_Double_Read(BMP085_SlaveAddress, 0xBC, &mc));
	RET_IF(I2C_Double_Read(BMP085_SlaveAddress, 0xBE, &md));
	
	return 0;
}


int read_BMP085(int *data)
{
	int rtn = 1;
	uint8_t tmp[2];

	do
	{
		if (new_temperature == 0 && last_temperature_time == 0 && last_pressure_time == 0)
		{
			if (I2C_WriteReg(BMP085_SlaveAddress, 0xF4, 0x2E) < 0)
				break;
			last_temperature_time = getus();
		}
		
		if (getus() - last_temperature_time >  SAMPLEING_TIME && new_temperature == 0)
		{
			if (I2C_ReadReg(BMP085_SlaveAddress, 0xF6, tmp, 2) < 0)
				break;
		
			rawTemperature = ((int)tmp[0] << 8) | (int)tmp[1];
			//DeltaTemp = rawTemperature - (((int32_t)refdata[4]) << 8);
			//new_temperature = ((1<<EXTRA_PRECISION)*2000l + ((DeltaTemp * refdata[5]) >> (23-EXTRA_PRECISION))) / ((1<<EXTRA_PRECISION));
			x1 = ((long)rawTemperature - ac6) * ac5 >> 15;
			x2 = ((long) mc << 11) / (x1 + md);
			b5 = x1 + x2;
			new_temperature = ((b5 + 8) * 10) >> 4;


			if (I2C_WriteReg(BMP085_SlaveAddress, 0xF4, 0x34 + (OSS << 6)) < 0)
				break;

			
			last_pressure_time = getus();
		}

		if (getus() - last_pressure_time >  SAMPLEING_TIME && last_pressure_time > 0)
		{
			if (I2C_ReadReg(BMP085_SlaveAddress, 0xF6, tmp, 2) <0)
				break;
			
			rawPressure = ((int)tmp[0] << 8) + (int)tmp[1];

			b6 = b5 - 4000;
			x1 = (b2 * (b6 * b6 >> 12)) >> 11;
			x2 = ac2 * b6 >> 11;
			x3 = x1 + x2;
			b3 = (((long)ac1 * 4 + x3) + 2)/4;
			x1 = ac3 * b6 >> 13;
			x2 = (b1 * (b6 * b6 >> 12)) >> 16;
			x3 = ((x1 + x2) + 2) >> 2;
			b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;
			b7 = ((unsigned long) rawPressure - b3) * (50000 >> OSS);
			if( b7 < 0x80000000)
				p = (b7 * 2) / b4 ;
			else  
				p = (b7 / b4) * 2;
			x1 = (p >> 8) * (p >> 8);
			x1 = (x1 * 3038) >> 16;
			x2 = (-7357 * p) >> 16;
			pressure = p + ((x1 + x2 + 3791) >> 4);
			temperature = new_temperature;

			





			new_temperature = 0;
			last_temperature_time = 0;
			last_pressure_time = 0;
			rtn = 0;
		}
	} while (0);
	
	data[0] = pressure;
	data[1] = temperature;
	return rtn;
}
