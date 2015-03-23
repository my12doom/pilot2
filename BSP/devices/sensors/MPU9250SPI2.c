#include "MPU9250.h"
#include <stdio.h>
#include "../common/common.h"
#include "../common/printf.h"
#include "../common/timer.h"

// Gyro and accelerator registers
#define	SMPLRT_DIV		0x19
#define	MPU9250_CONFIG	0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define EXT_SENS_DATA	0x49
#define	PWR_MGMT_1		0x6B
#define	PWR_MGMT_2		0x6C
#define	WHO_AM_I		0x75

static int res;

static void CSLow()
{
	GPIO_ResetBits(GPIOE,GPIO_Pin_8);
}

static void CSHigh()
{
	GPIO_SetBits(GPIOE,GPIO_Pin_8);
}

static uint8_t txrx(uint8_t tx)
{
	uint8_t rx;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2, tx);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

	rx = SPI_I2S_ReceiveData(SPI2);

// 	LOGE("%02x - %02x\n", tx, rx);
	return rx;
}

static int read_reg(uint8_t reg, void *out, int count)
{
	int i;
	uint8_t *p = (uint8_t*)out;
	CSLow();
	delayus(1);

	txrx((reg&0x7f) | 0x80);
	for(i=0; i<count; i++)
		p[i] = txrx(0);

	delayus(1);
	CSHigh();

	return 0;
}

static int write_reg_core(uint8_t reg, uint8_t data)
{
	CSLow();
	delayus(1);

	txrx(reg);
	txrx(data);

	delayus(1);
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
			LOGE("reg(%02x) error %02x/%02x\n", reg, read, data);
	}
}

int init_MPU9250spi(void)
{
	uint8_t who_am_i = 0;
	int i;

	// SPI & GPIO Init
	SPI_InitTypeDef  SPI_InitStructure = {0};
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	// configure SCK,MISO,MOSI (GPIOA^5,GPIOA^6,GPIOA^7)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// configure CS (GPIOA15)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOE, GPIO_Pin_8);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);
	
	// MPU9250 register initialization
	TRACE("start MPU9250\r\n");
	delayms(10);
	write_reg_core(PWR_MGMT_1, 0x80);
	delayms(10);
	write_reg(PWR_MGMT_1, 0x00);
	write_reg(PWR_MGMT_2, 0x00);
	write_reg(SMPLRT_DIV, 0x01);
	write_reg(MPU9250_CONFIG, 0x2);
	write_reg(GYRO_CONFIG, 1 << 3);			// full scale : 500 degree/s, ~65.5 LSB/degree/s
	write_reg(ACCEL_CONFIG, 0x18);			// full scale : 16g, 2048 = 1g
	
	res = read_reg(WHO_AM_I, &who_am_i, 1);
	LOGE("MPU9250 initialized, WHO_AM_I=%x\n", who_am_i);
	res = read_reg(WHO_AM_I, &who_am_i, 1);
	LOGE("MPU9250 initialized, WHO_AM_I=%x\n", who_am_i);
	res = read_reg(WHO_AM_I, &who_am_i, 1);
	LOGE("MPU9250 initialized, WHO_AM_I=%x\n", who_am_i);

	for(i=0; i<128; i++)
	{
		uint8_t data;
		read_reg(i, &data, 1);
		LOGE("reg %02x = %02x\n", i, data);
	}
	
	delayms(10);

	if (who_am_i != 0x71)
		return -1;

	// enter SPI high speed mode for data only access
 	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
 	SPI_Init(SPI2, &SPI_InitStructure);

	//
	
	return 0;
}

// data[0 ~ 7] :
// accel_x, accel_y, accel_z, raw_temperature, gyro_x, gyro_y, gyro_z
static short gyro_o[3];
static short gyro_raw[3];
static int64_t lastus = -1;
int read_MPU9250spi(short*data)
{
	int i;
	int result;
	SPI_InitTypeDef  SPI_InitStructure = {0};
	
	//int64_t us;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
 	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	
	
	result = read_reg(ACCEL_XOUT_H, (uint8_t*)data, 14);
	for(i=0; i<7; i++)
		swap((uint8_t*)&data[i], 2);

	return result;
}
