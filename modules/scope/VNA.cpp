#include <string.h>
#include <stdio.h>
#include <math.h>

#include <stdint.h>
#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4SysTimer.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4VCP.h>
#include <HAL/STM32F4/F4UART.h>

#include <HAL/Interface/II2C.h>
#include <HAL/aux_devices/si5351.h>
#include <HAL/aux_devices/LMX2572.h>

using namespace STM32F4;
using namespace HAL;

extern "C" void SetSysClock(int hse_mhz = 0);
int default_download_IC_1(bool _24bit);
void i2s_setup();
int set_volume(uint8_t gain_code, bool muted/* = false */);	// 0.75db/LSB
int hmc960_read();

static F4GPIO scl(GPIOC, GPIO_Pin_13);
static F4GPIO sda(GPIOC, GPIO_Pin_14);
static I2C_SW i2c(&scl, &sda);
static F4GPIO cs_LO(GPIOB, GPIO_Pin_1);
LMX2572 LO;

//#define TRACE printf
#define TRACE(...)

bool detect_adc(II2C &i2c)
{
    if (i2c.start()<0) {
        return false;
    }

    i2c.tx(0x70&0xFE);

    if (i2c.wait_ack()<0) {
		i2c.stop();
		return false;
    }

	i2c.stop();
	return true;
}

static int imin(int a, int b)
{
	return a>b?b:a;
}

uint8_t regs[256];
uint16_t &sweep_points = *(uint16_t*)&regs[20];

typedef struct
{
	int32_t fwd0r;
	int32_t fwd0i;
	int32_t rev0r;
	int32_t rev0i;
	int32_t rev1r;
	int32_t rev1i;
	uint16_t freq_index;
	uint8_t reserved[6];
} fifo_value;

fifo_value mock = 
{
	0x100000,
	0,
	0x10000,
	0,
	0,
	0x600000,
	0,
};

int freq_index = 0;


int handle_vcp(IUART &vcp)
{
	uint8_t op;
	int arg_len;
	uint8_t arg_buf[10];

	if (vcp.peak(&op, 1) != 1)
		return 0;

	switch (op)
	{
		case 0x00:
		case 0x0d:
			arg_len = 0;
			break;
		case 0x10:
		case 0x11:
		case 0x12:
			arg_len = 1;
			break;
		case 0x18:
		case 0x20:
			arg_len = 2;
			break;
		case 0x21:
			arg_len = 3;
			break;
		case 0x22:
			arg_len = 5;
			break;
		case 0x23:
			arg_len = 9;
			break;
		default:
			vcp.read(&op, 1);
			return 0;
	}
	
	if (vcp.available() < 1 + arg_len)
		return 0;

	vcp.read(&op, 1);
	vcp.read(arg_buf, arg_len);
	TRACE("op %02x, arg=", op);
	for(int i=0; i<arg_len; i++)
		TRACE("%02x,", arg_buf[i]);
	TRACE("\n");

	switch (op)
	{
		case 0x00:					// NOP
			break;
		case 0x0d:					// INDICATE
			vcp.write("2", 1);
			TRACE("INDICATE\n");
			break;

		case 0x10:					// READ1
			vcp.write(&regs[arg_buf[0]], 1);
			TRACE("READ8 @ %02x\n", arg_buf[0]);
			break;
		case 0x11:					// READ2
			vcp.write(&regs[arg_buf[0]], 2);
			TRACE("READ16 @ %02x\n", arg_buf[0]);
			break;
		case 0x12:					// READ4
			vcp.write(&regs[arg_buf[0]], 4);
			TRACE("READ32 @ %02x\n", arg_buf[0]);
			break;

		case 0x18:					// READ FIFO
			if (arg_buf[0] == 0x30)	// value FIFO
			{
				int sweep_points = *(uint16_t*)&regs[0x20];
				int valuesPerFrequency = *(uint16_t*)&regs[0x22];
				mock.rev0r = (systimer->gettime() % 500000 < 250000) ? 0x60000 : 0x600;
				
				for(int i=0; i<arg_buf[1]; i++)
				{
					if (++mock.reserved[0] == valuesPerFrequency)
					{
						mock.freq_index = (mock.freq_index+1) % sweep_points;
						mock.reserved[0] = 0;
					}
					vcp.write(&mock, sizeof(mock));
				}

			}
			break;

		case 0x20:
			TRACE("WRITE8 @ %02x\n", arg_buf[0]);
			memcpy(&regs[arg_buf[0]], arg_buf+1, 1);
			break;
		case 0x21:
			TRACE("WRITE16 @ %02x\n", arg_buf[0]);
			memcpy(&regs[arg_buf[0]], arg_buf+1, 2);
			break;
		case 0x22:
			TRACE("WRITE32 @ %02x\n", arg_buf[0]);
			memcpy(&regs[arg_buf[0]], arg_buf+1, 4);
			break;
		case 0x23:
			TRACE("WRITE64 @ %02x\n", arg_buf[0]);
			memcpy(&regs[arg_buf[0]], arg_buf+1, 8);
			break;
		default:
			break;
	}

	return 0;
}


int SIGMA_WRITE_REGISTER_BLOCK( uint8_t devAddress, uint16_t address, int length, uint8_t* pData )
{
    if (i2c.start()<0) {
        return -1;
    }

    i2c.tx(devAddress&0xFE);

    if (i2c.wait_ack()<0) {
		i2c.stop();
		return -1;
    }

    i2c.tx(address>>8);
    if (i2c.wait_ack()<0)
	{
		i2c.stop();
		return -1;
	}

	i2c.tx(address);
    if (i2c.wait_ack()<0)
	{
		i2c.stop();
		return -1;
	}

    for(int i=0; i<length; i++)
    {
        i2c.tx(pData[i]);
        
        if (i2c.wait_ack()<0)
        {
            i2c.stop();
            return -1;
        }
    }

    i2c.stop();

    return 0;
}

int SIGMA_WRITE_DELAY( uint8_t devAddress, int length, uint8_t *pData )
{
	systimer->delayms(100);
	return 0;
}

	F4SPI spi(SPI1);
	static F4GPIO cs(GPIOB, GPIO_Pin_2);

int hmc960_write(uint8_t reg, uint32_t value)
{
	reg = ((reg & 0x1f) << 3) | 0x06;	// 0x06: chip id
	cs.write(0);
	spi.txrx(value >> 16);
	spi.txrx(value >> 8);
	spi.txrx(value);
	spi.txrx(reg);
	cs.write(1);
	return 0;
}

uint32_t read960[4] = {0};
int hmc960_read()
{
	hmc960_write(1,0x0e);
	
	for(int i=0; i<3; i++)
	{


		spi.set_mode(0, 0);
		hmc960_write(0, i);
		spi.set_mode(0, 1);

		uint32_t read;
		cs.write(0);
		read = spi.txrx(0)<<16;
		read |= spi.txrx(0)<<8;
		read |= spi.txrx(0);
		spi.txrx(0);
		cs.write(1);

		read960[i] = read;
	}
}

uint16_t regs2572[126];
int test2572()
{
	LO.init(&spi, &cs_LO);
	for(int i=0; i<126; i++)
		regs2572[i] = LO.read_reg(i);
	
	LO.set_ref(100000000, false, 1, 1, 1);
	LO.set_freq(3200000000+1);
	LO.set_output(true, 63);
	//LO.set_freq(25e6);

	//while (!LO.is_locked())
	{
		//
		printf("..");
	}


	return 0;
}


int main()
{
	// css
	cs.set_mode(MODE_OUT_PushPull);
	cs.write(1);
	cs_LO.set_mode(MODE_OUT_PushPull);
	cs_LO.write(1);


	spi.set_speed(1000000);	

	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));		// switch to HSI
	RCC->CR &= ~((uint32_t)RCC_CR_HSEON | RCC_CR_PLLON);	// disable HSE & PLL

	F4GPIO led(GPIOB, GPIO_Pin_9);
	led.set_mode(HAL::MODE_OUT_PushPull);
	led.write(0);

	si5351 si;
	if (si.set_i2c(&i2c) < 0)
	{
		while(1)
		{
			led.write(1);
			systimer->delayms(500);
			led.write(0);
			systimer->delayms(500);
		}
	}
	

	si.set_ref_freq(26.0);
	si.set_pll(0, 900);
	si.set_pll(1, 12.288*60);
	si.set_output_freq(0, 0, 9);			// 100M ref clock
	si.set_output_freq(1, 1, 60);			// 24.576M adc clock
	si.set_output_freq(2, 2, 900/25.0);		// 24M mcu clock
	si.set_output(0, false, false);
	si.set_output(1, false, false);
	si.set_output(2, false, false);


	// use clock from si5351
	SetSysClock(25);
	SystemCoreClockUpdate();
	test2572();

	led.write(SystemCoreClock == 84000000);

	default_download_IC_1(true);
	set_volume(63, false);

	// disable adau1761 clock for now
	si.set_output(1, true, false);
	systimer->delayms(1);
	i2s_setup();
	// enable it back after i2s init
	si.set_output(1, false, false);

	// we should get no i2s frame error here
	systimer->delayms(2);
	if (SPI3->SR & 0x100)
	while(1)
	{
		NVIC_SystemReset();

		led.write(1);
		systimer->delayms(100);
		led.write(0);
		systimer->delayms(100);
	}
	
	// MCO test
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_4);

	
	// default registers	
	*(uint16_t*)&regs[20] = 150;		// sweep_points

	// USB ON
	F4VCP vcp;

	while(1)
	{
		hmc960_read();
		handle_vcp(vcp);
	}
}


int16_t codec_data[96*4];		// 1ms buffer

void i2s_setup()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	// GPIO & AF configure
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);			// CK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_I2S3ext);		// SD_ext
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);			// SD
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SPI3);		// WS
	
	/*
	i2s_configure_tx();
	//I2S_Cmd(SPI3, ENABLE);
	I2S_Cmd(I2S3ext, ENABLE);
	return;
	*/

	// DMA configure
	NVIC_InitTypeDef   NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(SPI3->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&codec_data[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = sizeof(codec_data)/2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Cmd(DMA1_Stream0, DISABLE);

	DMA_DeInit(DMA1_Stream0);
	DMA_Init(DMA1_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA1_Stream0, ENABLE);
	DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_HTIF0);
	DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
	DMA_ITConfig(DMA1_Stream0, DMA_IT_HT, ENABLE);
	DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
	
	// I2S configure
	I2S_InitTypeDef i2s_init_str;
	i2s_init_str.I2S_Mode = I2S_Mode_SlaveRx;
	i2s_init_str.I2S_Standard = I2S_Standard_LSB;
	i2s_init_str.I2S_DataFormat = I2S_DataFormat_24b;
	i2s_init_str.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
	i2s_init_str.I2S_AudioFreq = 96000;
	i2s_init_str.I2S_CPOL = I2S_CPOL_High;
	I2S_Cmd(SPI3, DISABLE);
	SPI_I2S_DeInit(SPI3);
	I2S_Init(SPI3, &i2s_init_str);	
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);
	
	// go!
	I2S_Cmd(SPI3, ENABLE);
}

int32_t i2s_fmt(int32_t in)
{
	int32_t out;
	uint8_t *pin = (uint8_t*)&in;
	uint8_t *pout = (uint8_t*)&out;

	// input
	// bus: XXB MSB MID LSB
	// dma buffer: [MSB XXB] [LSB MID]
	// out LSB MID MSB MSB&0x80
	pout[0] = pin[2];
	pout[1] = pin[3];
	pout[2] = pin[0];
	pout[3] = pin[0] & 0x80 ? 0xff : 0;

	return out;
}

uint64_t integrate[4];
uint64_t integrate_out[4];
int integrate_counter = 0;
int integrate_out_counter = 0;
const int integrate_count = 2;

int32_t int_debug[96];

extern "C" void DMA1_Stream0_IRQHandler(void)
{
	int32_t *data;
	int flag = DMA1->LIFCR & 0x30;

	// first half buffer
    if (DMA1->LISR & DMA_LISR_HTIF0) {
        DMA1->LIFCR = DMA_LIFCR_CHTIF0;
		data = (int32_t*)codec_data;
    }

	// second half buffer
    if (DMA1->LISR & DMA_LISR_TCIF0) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF0;
		data = (int32_t*)(codec_data + 96);
    }

	// strange i2s byte mapping
	int sample_count = sizeof(codec_data)/2 / sizeof(uint32_t) / 2;
	for(int i=0; i<sample_count; i++)
	{
		int32_t *p = data + i*2;
		int32_t c1 = i2s_fmt(p[0]);
		int32_t c2 = i2s_fmt(p[1]);

		integrate[0] += c1;
		integrate[1] += c2;

		int_debug[i] = c1;
		int_debug[i+48] = c2;
	}

	if (++integrate_counter == integrate_count)
	{
		integrate_counter = 0;
		integrate_out_counter ++;
		memcpy(integrate_out, integrate, sizeof(integrate));
		memset(integrate, 0 , sizeof(integrate));
	}
}


int set_volume(uint8_t gain_code, bool muted/* = false */)	// 0.75db/LSB
{
	gain_code &= 0x3f;
	gain_code <<= 2;
	gain_code |= 1;
	if (!muted)
		gain_code |= 0x2;
	uint8_t v[2] = {gain_code, gain_code};

	return SIGMA_WRITE_REGISTER_BLOCK(0x70, 0x400E, 2, v);
}