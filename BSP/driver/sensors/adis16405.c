#include "adis16405.h"
#include "../common/mcu.h"

#define SET_CS()		GPIO_SetBits(GPIOA, GPIO_Pin_15)	//PA.4->/CS
#define CLR_CS()		GPIO_ResetBits(GPIOA,GPIO_Pin_15);

#define	SET_SCL()		GPIO_SetBits(GPIOA, GPIO_Pin_5)	//PA.5->SCLK
#define	CLR_SCL()		GPIO_ResetBits(GPIOA, GPIO_Pin_5)

#define SET_SDO()		GPIO_SetBits(GPIOA, GPIO_Pin_7)	 //PA.7->DIN（从的入）
#define CLR_SDO()		GPIO_ResetBits(GPIOA, GPIO_Pin_7)


static void DelayCLK()
{
	__IO uint32_t nCount = 100;
	for(; nCount != 0; nCount--);
}
static void DelayBurst()
{
	__IO uint32_t nCount = 20;
	for(; nCount != 0; nCount--);
}
static void DelayMCU()
{
	__IO uint32_t nCount = 250;
	for(; nCount != 0; nCount--);
}

static void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

static uint16_t spi_tx_rx16_burst(uint16_t tx)
{
	uint16_t rx = 0;
	int i;

	for(i=0; i<16; i++)
	{
		CLR_SCL();

		if (tx & 0x8000)
			SET_SDO();
		else
			CLR_SDO();
		tx <<= 1;

		DelayBurst();
		SET_SCL();
	
		rx <<= 1;
		rx |= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
		DelayBurst();
	}

	return rx;
}

static uint16_t spi_tx_rx16(uint16_t tx)
{
	uint16_t rx = 0;
	int i;

	for(i=0; i<16; i++)
	{
		CLR_SCL();

		if (tx & 0x8000)
			SET_SDO();
		else
			CLR_SDO();
		tx <<= 1;

		DelayCLK();
		SET_SCL();

		rx <<= 1;
		rx |= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
		DelayCLK();
	}

	return rx;
}


static void CSLow()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);
}

static void CSHigh()
{
	GPIO_SetBits(GPIOA,GPIO_Pin_15);
}

int adis16405_init(void)
{
	SPI_InitTypeDef  SPI_InitStructure = {0};
	GPIO_InitTypeDef GPIO_InitStructure;

	// configure SCK,MISO,MOSI,CS (GPIOA^5,GPIOA^6,GPIOA^7,GPIOA^15)
#ifdef STM32F1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

#ifdef STM32F4
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

	GPIO_SetBits(GPIOA, GPIO_Pin_15);

	return 0;
}

#ifdef __GNUC__
#define NOP __ASM volatile ("nop")
#else
#define NOP __nop()
#endif

int16_t ReadFromADIS16405ViaSpi(unsigned char RegisterAddress)
{
	unsigned	char	ControlValue = 0;
	int16_t 	ReceiveData = 0;
	char		iTemp = 0;			//8位
	int	i = 0, j = 0;
	//先读低8位地址，再读高8位地址

	///Create the 8-bit header
		ControlValue = RegisterAddress ;//every register of ADIS16405 takes up two bytes ，先读低8位地址
		SET_SCL();
		NOP;
		NOP;
		NOP;
		NOP;
		NOP;
		NOP;
		SET_CS();
		NOP;
		NOP;
 		NOP;
		NOP;
		NOP;
		NOP;
		NOP;
		NOP;
		NOP;
 		NOP;
		NOP;
		NOP;
		NOP;
		NOP;
		CLR_CS();	 //bring CS low
		DelayCLK();

		//Write out the control word   和地址
		for(i=0; i<16; i++)
		{
			CLR_SCL();						//SCL由高变低时写入MISO
			//DelayCLK();
			if(0x80 == (ControlValue & 0x80))
			{
				SET_SDO();	  //Send one to DIN pin	of ADIS16405
			}
			else
			{
				CLR_SDO();	  //Send zero to DIN pin of ADIS16405
			}
			DelayCLK();
			SET_SCL();
			DelayCLK();
			ControlValue <<= 1;	//Rotate data
		}
		DelayCLK();

		//Read data in
		for(i=0; i<16; i++)
		{
			CLR_SCL();					
			DelayCLK();
	
			SET_SCL();			 //由低变高时 采样	  MOSI
			NOP;
			NOP;
			NOP;
			NOP;
			NOP;
			NOP;
			NOP;
			NOP;
			NOP;
			NOP;
		
			ReceiveData <<= 1;		//Rotate data
			ReceiveData |= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
			DelayCLK();

		}
		DelayCLK();
		SET_CS();	//bring CS high again
		translate_14bit(&ReceiveData);
		return ReceiveData;
}

int adis16405_burst_read(adis16405_burst_packet *out)
{
	unsigned short *p = (unsigned short*)out;
	int i;
	if (!out)
		return -1;

	CSLow();
	
	DelayCLK();

	spi_tx_rx16_burst(0x3E00);
	
	for(i=0; i<sizeof(adis16405_burst_packet)/2; i++)
	{
		DelayCLK();
		p[i] = spi_tx_rx16_burst(0);
	}
	DelayCLK();
	CSHigh();

	out->supply_measurement &= 0x1fff;
	translate_14bit(&out->accel_z);
	translate_14bit(&out->gyro_x);
	translate_14bit(&out->gyro_y);
	translate_14bit(&out->gyro_z);
	translate_14bit(&out->accel_x);
	translate_14bit(&out->accel_y);
	translate_14bit(&out->accel_z);
	translate_14bit(&out->mag_x);
	translate_14bit(&out->mag_y);
	translate_14bit(&out->mag_z);

	return 0;
}

int adis16405_read_register(unsigned char registerAddress, unsigned short *out)
{	
	CSLow();
	DelayMCU();
    spi_tx_rx16((registerAddress & 0x7f) << 8);
	DelayMCU();
	*out = spi_tx_rx16(0);
	DelayMCU();
	CSHigh();

	return 0;
}


int adis16405_write_register(unsigned char registerAddress, unsigned short new_value)
{
	if (registerAddress > aux_dac || (registerAddress & 0x1))
		return -1;

	CSLow();
	DelayMCU();
    spi_tx_rx16((((registerAddress+1) & 0x7f) << 8) | (new_value >> 8) | 0x8000 ) ;
	DelayMCU();
    spi_tx_rx16((((registerAddress) & 0x7f) << 8) | (new_value & 0xff) | 0x8000) ;
	DelayMCU();
	CSHigh();

	return 0;
}
