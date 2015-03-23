#include "HMC5883.h"
#include "../common/common.h"
#include <math.h>
#include "../common/mcu.h"

#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

static int16_t min(int16_t a, int16_t b)
{
	return a>b?b:a;
}

static void CSLow()
{
	//LOGE("Low\n");
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);
	
}

static void CSHigh()
{
	//LOGE("High\n\n");
	GPIO_SetBits(GPIOA,GPIO_Pin_15);
	
}

static uint8_t txrx(uint8_t tx)
{
	uint8_t rx;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, tx);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	rx = SPI_I2S_ReceiveData(SPI1);

	
	//LOGE("%02x - %02x\n", tx, rx);
	return rx;
}

static int read_reg(uint8_t reg, void *out, int count)
{
	int i;
	uint8_t *p = (uint8_t*)out;
	CSLow();
	

	txrx( (reg) | 0x80 | (count > 1 ? 0x40 : 0));
	for(i=0; i<count; i++)
		p[i] = txrx(0);

	
	CSHigh();

	return 0;
}

static int write_reg_core(uint8_t reg, uint8_t data)
{
	int count = 1;
	CSLow();
	

	txrx(reg | (count > 1 ? 0x40 : 0));
	txrx(data);

	
	CSHigh();

	return 0;
}

static int write_reg(uint8_t reg, uint8_t data)
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

int init_HMC5983(void)
{
	int i;
	int j;
	short data[3];
	uint8_t identification[3] = {0};

	// SPI & GPIO Init
	SPI_InitTypeDef  SPI_InitStructure = {0};
	GPIO_InitTypeDef GPIO_InitStructure;

#ifdef STM32F1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F4
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
#endif

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	// configure SCK,MISO,MOSI (GPIOA^5,GPIOA^6,GPIOA^7)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
#ifdef STM32F1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
#endif
#ifdef STM32F4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
#endif
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// configure CS (GPIOA15)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef STM32F1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
#endif
#ifdef STM32F4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
#endif
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);

	for(i=0; i<3; i++)
	{
		
		CSLow();
		
		txrx(0x00);
		
		txrx(0x33);
		
		CSHigh();
		
				
		CSLow();
		
		txrx(0x80);
		
		txrx(0x00);
		
		CSHigh();
		
		delayms(10);
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

int check_HMC5983(void)			// check for HMC5883 healthy, re-plug and init it if possible
								// return : 0 if hardware error happend, 
								//			-1 if hardware error resolved
								//			1 if everything OK
{
	return 1;
}

int read_HMC5983(short*data)
{
	//LOGE("READ\n");
	int i;
	int result = read_reg(0x03, (uint8_t*)data, 6);
	for(i=0; i<3; i++)
	{
		swap((uint8_t*)&data[i], 2);
	}
	
	return result;
}
