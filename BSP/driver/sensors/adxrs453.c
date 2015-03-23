#include "adxrs453.h"

#include "../common/mcu.h"

uint8_t spi_tx_rx(uint8_t tx)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, tx);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	return SPI_I2S_ReceiveData(SPI1);
}

uint32_t spi_tx_rx32(uint32_t tx)
{
	int i;
	uint32_t o = 0;
	for(i=0; i<4; i++)
	{
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPI1, tx >> 24);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

		o |= SPI_I2S_ReceiveData(SPI1);
		o <<= 8;
		tx >>= 8;
	}
	return o;
}

void CSLow()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);
}

void CSHigh()
{
	GPIO_SetBits(GPIOA,GPIO_Pin_15);
}

int adxrs453_init(void)
{
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

	// Enable SPI1
	SPI_Cmd(SPI1, ENABLE);

	return 0;
}

int adxrs453_read_register(unsigned char registerAddress, unsigned short *out)
{
	int i;
	unsigned char  dataBuffer[4] = {0, 0, 0, 0};
	unsigned char  outBuffer[4] = {0, 0, 0, 0};
    unsigned long  command       = 0;
    unsigned char  bitNo         = 0;
    unsigned char  sum           = 0;

	if (!out)
		return -1;
	
	adxrs453_init();

    dataBuffer[0] = ADXRS453_READ | (registerAddress >> 7);
    dataBuffer[1] = (registerAddress << 1);
    command = ((unsigned long)dataBuffer[0] << 24) |
              ((unsigned long)dataBuffer[1] << 16) |
              ((unsigned short)dataBuffer[2] << 8) |
              dataBuffer[3];
    for(bitNo = 31; bitNo > 0; bitNo--)
        sum += ((command >> bitNo) & 0x1);
    if(!(sum % 2))
        dataBuffer[3] |= 1;

	CSLow();
    for(i=0; i<4; i++)
    	outBuffer[i] = spi_tx_rx(dataBuffer[i]);
	CSHigh();
	
    *out = ((unsigned long)outBuffer[1] << 11) |
           ((unsigned long)outBuffer[2] << 3) |
           ((unsigned long)outBuffer[3] >> 5);

    // TODO: check parity bits

	return 0;
}

short adxrs453_get_data(void)
{
	short out;
	if (adxrs453_read_register(ADXRS453_REG_RATE, (unsigned short*)&out) < 0)
		return 0;	// maybe some error code?

	return out;
}
