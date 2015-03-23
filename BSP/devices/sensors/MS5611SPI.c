#include "MS5611SPI.h"
#include "../common/common.h"

// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E
// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define MS561101BA_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.
#define EXTRA_PRECISION 5 // trick to add more precision to the pressure and temp readings
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)
#define ADDR_INCREMENT		(1<<6)

#define SAMPLEING_TIME 10000 // 10000us, 10ms

static uint8_t OSR = MS561101BA_OSR_4096;
static int temperature = 0;
static int pressure = 0;
static int new_temperature = 0;
static int64_t last_temperature_time = 0;
static int64_t last_pressure_time = 0;
static int64_t rawTemperature = 0;
static int64_t rawPressure = 0;
static int64_t DeltaTemp = 0;
static int64_t off;//  = (((int64_t)_C[1]) << 16) + ((_C[3] * dT) >> 7);
static int64_t sens;// = (((int64_t)_C[0]) << 15) + ((_C[2] * dT) >> 8);
static uint16_t refdata[6];
static uint16_t crc;

static void CSLow()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);
}

static void CSHigh()
{
	GPIO_SetBits(GPIOA,GPIO_Pin_15);
}

static uint8_t txrx(uint8_t tx)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, tx);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	return SPI_I2S_ReceiveData(SPI1);
}

static int read_reg(uint8_t reg, void *out, int count)
{
	int i;
	uint8_t *p = (uint8_t*)out;
	CSLow();

	txrx(reg);
	for(i=0; i<count; i++)
		p[i] = txrx(0);

	CSHigh();

	return 0;
}

static int write_reg(uint8_t data)
{
	CSLow();

	txrx(data);

	CSHigh();

	return 0;
}


// call initI2C before this
int init_MS5611spi(void)
{
	uint8_t tmp[3];
	int i;

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
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);

	// Reset MS5611 and read prog ram
	write_reg(MS561101BA_RESET);	
	delayms(10);
	for(i=0; i<6; i++)
	{
		read_reg(MS561101BA_PROM_BASE_ADDR+i*2, tmp, 2);
		refdata[i] = (tmp[0] << 8) + tmp[1];
	}
	
	if (read_reg(MS561101BA_PROM_BASE_ADDR+12, tmp, 2) < 0)
		return -1;
	crc = (tmp[0] << 8) + tmp[1];

	/*	
	// Temperature
	write_reg(MS5611Address, MS561101BA_D2 + OSR, 0x00);
	delayms(10);
	read_reg(MS5611Address, 0x00, tmp, 3);
	
	rawTemperature = ((int)tmp[0] << 16) + ((int)tmp[1] << 8) + (int)tmp[2];
	DeltaTemp = rawTemperature - (((int32_t)refdata[4]) << 8);
	temperature = ((1<<EXTRA_PRECISION)*2000l + ((DeltaTemp * refdata[5]) >> (23-EXTRA_PRECISION))) / ((1<<EXTRA_PRECISION));
		
	// Pressure
	write_reg(MS5611Address, MS561101BA_D1 + OSR, 0x00);
	delayms(10);
	read_reg(MS5611Address, 0x00, tmp, 3);
	
	rawPressure = ((int)tmp[0] << 16) + ((int)tmp[1] << 8) + (int)tmp[2];
	off  = (((int64_t)refdata[1]) << 16) + ((refdata[3] * DeltaTemp) >> 7);
	sens = (((int64_t)refdata[0]) << 15) + ((refdata[2] * DeltaTemp) >> 8);
	pressure = ((((rawPressure * sens) >> 21) - off) >> (15-EXTRA_PRECISION)) / ((1<<EXTRA_PRECISION));

	*/
	
	return 0;
}

int check_MS5611spi(void)
{
	uint8_t tmp[2];
	uint16_t crc2;
	read_reg(MS561101BA_PROM_BASE_ADDR+12, tmp, 2);
	crc2 = (tmp[0] << 8) + tmp[1];

	return crc == crc2;
}


int read_MS5611spi(int *data)
{
	int rtn = 1;
	uint8_t tmp[3];

	do
	{
		if (new_temperature == 0 && last_temperature_time == 0 && last_pressure_time == 0)
		{
			if (write_reg(MS561101BA_D2 + OSR) < 0)
			{
				rtn = -1;
				break;
			}
			last_temperature_time = getus();
		}
		
		if (getus() - last_temperature_time >  SAMPLEING_TIME && new_temperature == 0)
		{
			if (read_reg(0x00, tmp, 3) < 0)
			{
				rtn = -1;
				break;
			}
		
			rawTemperature = ((int)tmp[0] << 16) + ((int)tmp[1] << 8) + (int)tmp[2];
			DeltaTemp = rawTemperature - (((int32_t)refdata[4]) << 8);
			new_temperature = ((1<<EXTRA_PRECISION)*2000l + ((DeltaTemp * refdata[5]) >> (23-EXTRA_PRECISION))) / ((1<<EXTRA_PRECISION));
			
			if (write_reg(MS561101BA_D1 + OSR) < 0)
			{
				rtn = -1;
				break;
			}

			last_pressure_time = getus();
		}

		if (getus() - last_pressure_time >  SAMPLEING_TIME && last_pressure_time > 0)
		{
			if (read_reg(0x00, tmp, 3) <0)
			{
				rtn = -1;
				break;
			}
			
			rawPressure = ((int)tmp[0] << 16) + ((int)tmp[1] << 8) + (int)tmp[2];
			off  = (((int64_t)refdata[1]) << 16) + ((refdata[3] * DeltaTemp) >> 7);
			sens = (((int64_t)refdata[0]) << 15) + ((refdata[2] * DeltaTemp) >> 8);
			pressure = ((((rawPressure * sens) >> 21) - off) >> (15-EXTRA_PRECISION)) / ((1<<EXTRA_PRECISION));
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
