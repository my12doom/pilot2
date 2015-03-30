#include "HMC5983SPI.h"
#include <math.h>
#include <stdio.h>

#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

#define LOGE printf
#define TRACE printf

static int16_t min(int16_t a, int16_t b)
{
	return a>b?b:a;
}

static void swap(void *buf, int size)
{
	char *p = (char*)buf;
	int i;
	for(i=0; i<size/2; i++)
	{
		char t = p[i];
		p[i] = p[size-1-i];
		p[size-1-i] = t;
	}
}

namespace sensors
{

int HMC5983::read_reg(uint8_t reg, void *out, int count)
{
	int i;
	uint8_t *p = (uint8_t*)out;
	CS->write(false);
	

	spi->txrx( (reg) | 0x80 | (count > 1 ? 0x40 : 0));
	for(i=0; i<count; i++)
		p[i] = spi->txrx(0);

	
	CS->write(true);

	return 0;
}

int HMC5983::write_reg_core(uint8_t reg, uint8_t data)
{
	int count = 1;
	CS->write(false);
	

	spi->txrx(reg | (count > 1 ? 0x40 : 0));
	spi->txrx(data);

	
	CS->write(true);

	return 0;
}

int HMC5983::write_reg(uint8_t reg, uint8_t data)
{
	uint8_t read;
	while(1)
	{
		read = data - 1;
		write_reg_core(reg, data);
		read_reg(reg, &read, 1);
		
		if (read == data)
			return 0;
		else
			LOGE("reg error %08x/%08x", read, data);
	}
}

HMC5983::HMC5983()
{
}

int HMC5983::init(HAL::ISPI *SPI, HAL::IGPIO *CS)
{
	int i;
	int j;
	short data[3];
	uint8_t identification[3] = {0};

	this->spi = SPI;
	this->CS = CS;
	SPI->set_mode(1,1);
	CS->set_mode(HAL::MODE_OUT_PushPull);
	CS->write(true);

	for(i=0; i<3; i++)
	{
		
		CS->write(false);
		
		spi->txrx(0x00);
		
		spi->txrx(0x33);
		
		CS->write(true);
		
				
		CS->write(false);
		
		spi->txrx(0x80);
		
		spi->txrx(0x00);
		
		CS->write(true);
		
		systimer->delayms(10);
	}

	// HMC5983 initialization
	for(i=0; i<3; i++)
		read_reg(0x0a+i, identification+i, 1);
	//write_reg(HMC58X3_R_CONFB, 0x40);  //Set the Gain
	//read_reg(HMC58X3_R_CONFB, identification, 2);
	//write_reg(HMC58X3_R_CONFB, 0x20);  //Set the Gain
	//read_reg(HMC58X3_R_CONFB, identification, 2);
	if (identification[0] != 'H' || identification[1] != '4' || identification[2] != '3')
	{
		LOGE("HMC5983 not found\n");
		//return -2;
	}

	write_reg(HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS);	// Reg A DOR=0x010 + MS1,MS0 set to pos bias
	write_reg(HMC58X3_R_CONFB, 0x40);  //Set the Gain
	write_reg(HMC58X3_R_MODE, 1);
				// Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
											// The new gain setting is effective from the second measurement and on.
	read_reg(0x03, (uint8_t*)data, 6);
	
	for(j=0; j<10; j++)
	{
		write_reg(HMC58X3_R_MODE, 1);
		
		read_reg(0x03, (uint8_t*)data, 6);
		for(i=0; i<3; i++)
		{
			swap(&data[i], 2);
		}
		
		if (-(1<<12) >= min(data[0],min(data[1],data[2])))
		{
			LOGE("mag saturation detected\n");
			//return -1;
		}
	}
	
	write_reg(HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
	write_reg(HMC58X3_R_MODE, 1);
	
	read_reg(0x03, (uint8_t*)data, 6);
	
	for(j=0; j<10; j++)
	{
		write_reg(HMC58X3_R_MODE, 1);
		
		read_reg(0x03, (uint8_t*)data, 6);
				
		if (-(1<<12) >= min(data[0],min(data[1],data[2])))
		{
			LOGE("mag saturation detected\n");
			//return -1;
		}
	}
		
	write_reg(HMC58X3_R_CONFA, 0x70); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
	write_reg(HMC58X3_R_CONFB, 0x20); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
	write_reg(HMC58X3_R_MODE, 0x00);
	
	return 0;
}

int HMC5983::read(short*data)
{
	int i;
	int result = read_reg(0x03, (uint8_t*)data, 6);
	for(i=0; i<3; i++)
		swap((uint8_t*)&data[i], 2);
	
	return result;
}

}