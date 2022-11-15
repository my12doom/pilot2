#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stm32f4xx_dma.h>

#include <stdint.h>
#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4SysTimer.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4VCP.h>
#include <HAL/STM32F4/F4UART.h>

#include <HAL/Interface/II2C.h>
#include <HAL/aux_devices/LMX2572.h>
#include <modules/Protocol/common.h>
#include <utils/fifo.h>

using namespace STM32F4;
using namespace HAL;

// board

// io
static F4GPIO scl(GPIOC, GPIO_Pin_13);
static F4GPIO sda(GPIOC, GPIO_Pin_14);

static F4GPIO cs_LO2(GPIOB, GPIO_Pin_6);
static F4GPIO cs_source2(GPIOB, GPIO_Pin_9);
static F4GPIO cs_IF2(GPIOE, GPIO_Pin_0);
F4GPIO att_le2(GPIOB, GPIO_Pin_7);

static F4GPIO cs_LO(GPIOA, GPIO_Pin_1);
static F4GPIO cs_source(GPIOA, GPIO_Pin_4);
static F4GPIO cs_IF(GPIOA, GPIO_Pin_0);
F4GPIO att_le(GPIOA, GPIO_Pin_2);

F4GPIO swd1[3] = {F4GPIO(GPIOD, GPIO_Pin_11), F4GPIO(GPIOD, GPIO_Pin_12), F4GPIO(GPIOD, GPIO_Pin_13)};
static F4GPIO sync(GPIOB, GPIO_Pin_13);

F4SPI spi(SPI1);
static I2C_SW i2c(&scl, &sda);
LMX2572 LO;
LMX2572 LO2;
LMX2572 source;

// funcs
void i2s_setup();
int hmc960_init();
int init_regs();

// gloabal variables
bool sweeping = false;

typedef enum
{
	SAMPLE_DISCARD1,
	/*
	SAMPLE_DISCARD2,
	SAMPLE_DISCARD3,
	SAMPLE_DISCARD4,
	SAMPLE_DISCARD5,
	*/
	SAMPLE_COMMIT,
	SAMPLE_NEXT_FREQ,
} sample_state;

sample_state state = SAMPLE_NEXT_FREQ;

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

CircularQueue<fifo_value, 128> fifo;

int64_t integrate[8];
int64_t integrate_out[8];
float phase[4];
double amp[4];
double amp_diff;
double amp_diff_db;
double amp_db[4];
float phase_diff;
int freq_index = 0;

int fifo_output_request = 0;

uint8_t regs[256];
uint8_t regs2[256];
uint16_t &sweep_points = *(uint16_t*)&regs[0x20];
uint64_t &sweep_start = *(uint64_t*)&regs[0x00];
uint64_t &sweep_step = *(uint64_t*)&regs[0x10];
uint16_t &valuesPerFrequency = *(uint16_t*)&regs[0x22];

int fifo_out(IUART &vcp)
{
	if (fifo_output_request == 0)
		return 0;

	while (fifo_output_request >0 && fifo.count())
	{
		fifo_value v;
		if (fifo.pop(&v) == 0)
		{
			float A = log10((double)v.fwd0r * v.fwd0r + (double)v.fwd0i * v.fwd0i) * 10;
			float A2 = log10((double)v.rev0r * v.rev0r + (double)v.rev0i * v.rev0i) * 10;
			float diff = A-A2;
			//if (fabs(diff) > 1)
			//printf("o:%d, amp=%.2f/%.2f, diff=%.2f\n", v.freq_index, A, A2, diff);
			vcp.write(&v, sizeof(v));
			fifo_output_request--;

			if (fifo_output_request == 0)
			{
				TRACE("\n\n\n---ROUND---\n\n\n");
			}

		}
		else
		{
			break;
		}
	}

	return 0;
}

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
				fifo_output_request = arg_buf[1];
				//fifo_fill(fifo_output_request);
				TRACE("\n\n\n---%d---\n\n\n", fifo_output_request);

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

int hmc960_write(uint8_t reg, uint32_t value)
{
	spi.set_mode(0, 0);
	reg = ((reg & 0x1f) << 3) | 0x06;	// 0x06: chip id
	cs_IF.write(0);
	cs_IF2.write(0);
	spi.txrx(value >> 16);
	spi.txrx(value >> 8);
	spi.txrx(value);
	spi.txrx(reg);
	cs_IF.write(1);
	cs_IF2.write(1);
	return 0;
}

uint32_t read960[4] = {0};
int hmc960_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	F4GPIO css[4] = {F4GPIO(GPIOA, GPIO_Pin_0), F4GPIO(GPIOA, GPIO_Pin_1),
		F4GPIO(GPIOA, GPIO_Pin_2), F4GPIO(GPIOA, GPIO_Pin_4)};
	for(int i=0; i<4; i++)
	{
		css[i].set_mode(MODE_OUT_PushPull);
		css[i].write(1);
	}

	hmc960_write(1,0x0f);
	hmc960_write(2,0x37);	// spi
	hmc960_write(3,40);		// 0.5db / LSB
	
	for(int i=0; i<3; i++)
	{
		hmc960_write(0, i);
		
		spi.set_mode(0, 1);

		uint32_t read;
		cs_IF.write(0);
		read = spi.txrx(0)<<16;
		read |= spi.txrx(0)<<8;
		read |= spi.txrx(0);
		spi.txrx(0);
		cs_IF.write(1);

		read960[i] = read;
	}
	spi.set_mode(0, 0);
	
	return 0;
}


void set_att(int att, int id = 1)
{
	IGPIO *le = id == 1 ? &att_le : &att_le2;
	att &= 0x7f;
	uint8_t tx = 0;
	for(int i=0; i<8; i++)
		tx |= ((att>>i)&1) << (7-i);

	att = limit(att, 0, 127);
	spi.txrx(tx);
	le->write(1);
	systimer->delayus(1);
	le->write(0);
}

// path: 1 - 6
int select_path(int path)
{
	if (path < 1 || path > 6)
		return -1;

	int path_tbl[7][3] = {{0,0,0}, {0,0,0}, {1,0,0}, {0,1,0}, {1,1,0}, {0,0,1}, {1,0,1}};

	for(int i=0; i<3; i++)
	{
		//swd1[2-i].write(path_tbl[path][i]);
		swd1[2-i].write(path_tbl[7-path][i]);
	}

	return 0;
}

int init_path()
{
	for(int i=0; i<3; i++)
	{
		swd1[i].set_mode(MODE_OUT_PushPull);
		//swd2[i].set_mode(MODE_OUT_PushPull);
	}

	select_path(1);

	return 0;
}

int get_path_by_freq(int64_t freq)
{
	if (freq > 3400000000)
		return 4;
	else if (freq > 1750000000)
		return 5;
	else if (freq > 900000000)
		return 6;
	else if (freq > 500000000)
		return 3;
	else if (freq > 250000000)
		return 2;
	else
		return 1;
}

int select_path_by_freq(int64_t freq)
{
	return select_path(get_path_by_freq(freq));
}

bool new_freq = true;
int64_t fc = 3640e6;
//int f_af = 12000;
//int f_if = 120e6;
int f_af = 12e3;
int f_if = 120e6;

int att = 120;
uint64_t last_fc = 0;

uint16_t LOregs[126];
uint16_t LO2regs[126];
uint16_t sourceregs[126];
int test_sync = 0;
uint16_t r0;
int update_freq()
{
	if (last_fc != fc)
	{
		sweeping = true;

		/*
		uint16_t LO_cap = 0x2742;
		uint16_t LO_DAC = 0xB0;
		uint16_t LO2_cap = 0x2748;
		uint16_t LO2_DAC = 0xB0;

		LO.write_reg(19, LO_cap);
		LO2.write_reg(19, LO2_cap);
		LO.write_reg(16, LO_DAC);
		LO2.write_reg(16, LO2_DAC);

		uint16_t r20 = (4<<11) | 0x48 | 0x4400;
		uint16_t r8 = 0x6000;
		LO.write_reg(8, r8);
		LO2.write_reg(8, r8);
		LO.write_reg(20, r20);
		LO2.write_reg(20, r20);
		*/
		
		//printf("\rfreq:%lld  ", fc);


		source.set_freq(fc, true);
		LO.set_freq((fc<6000e6) ? (fc - f_af + f_if) : (fc+f_af-f_if), true);
		LO2.set_freq((fc<6000e6) ? (fc - f_af + f_if) : (fc+f_af-f_if), true);

		/*
		LO.sync_start(NULL);
		LO2.sync_start(NULL);
		source.sync_start(NULL);

		systimer->delayus(5000);
		*/

		sync.write(1);
		systimer->delayus(1);
		sync.write(0);
		systimer->delayus(1);
		sync.write(1);
		
		select_path_by_freq(fc);
		systimer->delayus(1500);

		for(int i=0; i<126; i++)
		{
			//LOregs[i] = LO.read_reg(i);
			//LO2regs[i] = LO2.read_reg(i);
			//sourceregs[i] = source.read_reg(i);
		}
	
		last_fc = fc;
		sweeping = false;
	}
	
	if (test_sync)
	{
		test_sync = 0;
				
		sync.write(0);
		systimer->delayus(1);
		sync.write(1);
		systimer->delayus(1);
		sync.write(0);
		
		/*
		r0 = source.read_reg(0);
		r0 &= ~0x4000;
		source.write_reg(0, r0);
		r0 |= 0x4008;
		source.write_reg(0, r0);
		*/
		
		for(int i=0; i<126; i++)
		{
			LOregs[i] = LO.read_reg(i);
			LO2regs[i] = LO2.read_reg(i);
			sourceregs[i] = source.read_reg(i);
		}
	}

	return 0;
}

int init_LO_source()
{
	int fref = 40000000;

	LO.init(&spi, &cs_LO);
	LO.set_ref(fref, false, 1, 1, 1, true);
	LO.set_freq(fc-f_af-f_if, true);
	LO.set_output(true, 63, true, 63);

	LO2.init(&spi, &cs_LO2);
	LO2.set_ref(fref, false, 1, 1, 1, true);
	LO2.set_freq(fc-f_af-f_if, true);
	LO2.set_output(true, 63, true, 63);

	source.init(&spi, &cs_source);	
	source.set_ref(fref, false, 1, 1, 1, true);
	source.set_freq(fc, true);
	source.set_output(false, 0, true, 25);

	LO.sync_start(NULL);
	LO2.sync_start(NULL);
	source.sync_start(NULL);

	return 0;
}

static int n = 0;
int update_sweep()
{
	if (state == SAMPLE_NEXT_FREQ)
	{
		if (++n >= valuesPerFrequency)
		{
			freq_index = (freq_index+1) % sweep_points;
			n = 0;
		}

		fc = sweep_start + sweep_step * (uint64_t)freq_index;
		att = 60 - fc * 120 / 6400000000;
		att = limit(att, 0, 120);
		set_att(att, 1);
		set_att(att, 2);
		new_freq = true;
		state = SAMPLE_DISCARD1;
		update_freq();
		
	}

	return 0;
}


uint8_t reg3101_0[16];
uint8_t reg3101_1[16];

int configure_af_adc()
{
	F4GPIO led(GPIOD, GPIO_Pin_2);
	led.set_mode(HAL::MODE_OUT_PushPull);
	led.write(1);

	uint8_t add0 = 0x30;
	uint8_t add1 = 0x32;

	// soft reset & page 0
	i2c.write_reg(add0, 0, 0);
	i2c.write_reg(add1, 0, 0);
	if (i2c.write_reg(add0, 1, 1) < 0 || 
		i2c.write_reg(add1, 1, 1) < 0)
			return -1;

	// clock setting;
	// BCLK = 48*32*2 = 3072khz, WCLK = 48khz

	// ADC_MODCLK = ADCCLK_IN / 1 / 2 = 6144khz (128x oversample)
	// BCLK = ADC_CLK / 8
	// WCLK == ADC_FS = ADC_MODCLK / OSR	(not listed in datasheet)

	// master clock config
	int NADC = 1;
	if (40)
	{
		// for 40Mhz MCLK input
		// ADCCLK_IN = PLL_CLK, ADCCLK = PLL_CLK / 8
		// PLLCLK = MCLK * K * R / P, (K = J + D / 10000, R = vco divider, P = reference divider)
		// PLL VCO range: 80M - 110M
		// J range: 4 - 55 integer, 4 - 11 frac.
		// PLL pfd range: 0.5M - 20M integer, 10M - 20M frac
		// VCO frequency = 12.288 * 8 = 98.304 = 4.9152 * 20M
		// R = 1, J = 4, D = 9152, P = 2

		i2c.write_reg(add0, 0x04, 0x03);		// PLL_CLKIN = MCLK, ADC_CLKIN = PLL_CLK
		i2c.write_reg(add0, 0x05, 0x80 | 1 | (2<<4));		// P=2, R=1, PLL power up
		i2c.write_reg(add0, 0x06, 0x04);		// J=4
		i2c.write_reg(add0, 0x07, 9152 >> 8);	// D = 9152
		i2c.write_reg(add0, 0x08, 9152 & 0xff);

		NADC = 8;
	}
	else
	{
		// for 12.288Mhz MCLK input
		// ADCCLK = ADCCLK_IN = MCLK
		i2c.write_reg(add0, 0x04, 0x00);		// ADC_CLKIN = MCLK
		i2c.write_reg(add0, 0x05, 0x11);		// P=1, R=1, PLL power down
		i2c.write_reg(add0, 0x06, 0x00);		// J=0,
		i2c.write_reg(add0, 0x07, 0x00);		// D = 0
		i2c.write_reg(add0, 0x08, 0x00);
	}

	// clkout for slave
	i2c.write_reg(add0, 0x34, 0x10);		// GPIO1 = CLKOUT
	i2c.write_reg(add0, 0x19, 0x06);		// CLKOUT_SRC = ADC_CLK
	i2c.write_reg(add0, 0x1A, 0x81);		// CLKOUT enabled, divider = 1

	// slave clock config
	i2c.write_reg(add1, 0x04, 0x00);		// ADC_CLKIN = MCLK
	i2c.write_reg(add1, 0x05, 0x11);		// P=1, R=1, PLL power down
	i2c.write_reg(add1, 0x06, 0x00);		// J=0,
	i2c.write_reg(add1, 0x07, 0x00);		// D = 0
	i2c.write_reg(add1, 0x08, 0x00);



	// adc clock setting, ADC_CLK = 12.288Mhz, ADC_MODCLK = 6.144Mhz
	// master
	i2c.write_reg(add0, 0x12, NADC);		// NADC = 1, power down for now
	i2c.write_reg(add0, 0x13, 0x82);		// MADC = 2, power up
	i2c.write_reg(add0, 0x14, 0x80);		// AOSR = 128
	// slave
	i2c.write_reg(add1, 0x12, 0x81);		// NADC = 1, power up
	i2c.write_reg(add1, 0x13, 0x82);		// MADC = 2, power up
	i2c.write_reg(add1, 0x14, 0x80);		// AOSR = 128

	// analog(page 1)
	int8_t gain = 40;
	i2c.write_reg(add0, 0, 1);
	i2c.write_reg(add0, 0x33, 0x00);		// mic bias not used
	i2c.write_reg(add0, 0x3b, gain);		// left gain
	i2c.write_reg(add0, 0x3c, gain);		// right gain
	i2c.write_reg(add0, 0x34, 0x3f);		// left = differential
	i2c.write_reg(add0, 0x37, 0x3f);		// right = differential
	i2c.write_reg(add1, 0, 1);
	i2c.write_reg(add1, 0x33, 0x00);		// mic bias not used
	i2c.write_reg(add1, 0x3b, gain);		// left gain
	i2c.write_reg(add1, 0x3c, gain);		// right gain
	i2c.write_reg(add1, 0x34, 0x3f);		// left = differential
	i2c.write_reg(add1, 0x37, 0x3f);		// right = differential

	// power up adc & unmute (page 0)
	i2c.write_reg(add0, 0, 0);
	i2c.write_reg(add0, 0x51, 0xc2);
	i2c.write_reg(add0, 0x52, 0x00);
	i2c.write_reg(add1, 0, 0);
	i2c.write_reg(add1, 0x51, 0xc2);
	i2c.write_reg(add1, 0x52, 0x00);

	// power up main clock path
	i2c.write_reg(add0, 0, 0);
	i2c.write_reg(add1, 0, 0);	
	i2c.write_reg(add0, 0x12, 0x80 | NADC);		// power up ADC N divider
	systimer->delayms(100);

	// interface setting: i2s, 16bit word
	// master
	i2c.write_reg(add0, 0x1b, 0x0D);		// i2s master mode, 16bit word length, data 3-state enabled
	i2c.write_reg(add0, 0x1c, 0x00);		// data offset = 0 bclk, both channel
	i2c.write_reg(add0, 0x1d, 0x02);		// BCLK_IN = ADC_CLK
	i2c.write_reg(add0, 0x1e, 0x84);		// BCLK = BCLK_IN / 4, enable bclk divider
	i2c.write_reg(add0, 0x3d, 0x01);		// PRB_P1
	// slave
	i2c.write_reg(add1, 0x1b, 0x01);		// i2s slave mode, 16bit word length, data 3-state enabled
	i2c.write_reg(add1, 0x1c, 0x10);		// data offset = 16 bclk, both channel
	i2c.write_reg(add1, 0x1d, 0x02);		// BCLK_IN = ADC_CLK
	i2c.write_reg(add1, 0x1e, 0x84);		// BCLK = BCLK_IN / 4, enable bclk divider
	i2c.write_reg(add1, 0x3d, 0x01);		// PRB_P1


	// back to page 0 and read back
	i2c.write_reg(add0, 0, 0);
	i2c.write_reg(add1, 0, 0);
	//add1 = 0x66;
	i2c.read_regs(add0, 0, reg3101_0, 16);
	i2c.read_regs(add1, 0, reg3101_1, 16);



	// we should get no i2s frame error here
	i2s_setup();
	systimer->delayms(2);
	if (SPI3->SR & 0x100)
	while(1)
	{
		//NVIC_SystemReset();

		led.write(1);
		systimer->delayms(100);
		led.write(0);
		systimer->delayms(100);
	}
	
	return 0;
}

void calibrate_pll(IUART *vcp)
{
	while (!vcp->available())
		;
	
	int total = 512;
	for(int i = 0; i< total; i++)
	{
		uint64_t freq = i * 100e6 / total + 6300e6;
		fc = freq;
		update_freq();
		
		uint16_t regs[3][3];
		for(int i=0; i<3; i++)
		{
			regs[0][i] = source.read_reg(110+i);
			regs[1][i] = LO.read_reg(110+i);
			regs[2][i] = LO2.read_reg(110+i);
		}
		
		//for(int i=0; i<3; i++)
		{
			int vco = (regs[1][0] >> 5) &0x7;
			int cap = (regs[1][1]) &0xff;
			int dac = (regs[1][2]) &0x1ff;

			int vco2 = (regs[2][0] >> 5) &0x7;
			int cap2 = (regs[2][1]) &0xff;
			int dac2 = (regs[2][2]) &0x1ff;

			int cap_assist = LO.regs[19] & 0xff;
			int cap_assist2 = LO2.regs[19] & 0xff;

			char tmp[200];
			sprintf(tmp, "%lld\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", fc, vco, cap, dac, vco2, cap2, dac2, cap_assist, cap_assist2);
			vcp->write(tmp, strlen(tmp));
		}
		
	}
}

int main()
{	
	// USB ON
	F4VCP vcp;

	// default registers	
	*(uint16_t*)&regs[0x20] = 5;		// sweep_points
	sweep_points = 1;
	sweep_start = 40e6;
	sweep_step = 1e5;

	// cs_IF pins
	cs_IF.set_mode(MODE_OUT_PushPull);
	cs_IF.write(1);
	cs_LO.set_mode(MODE_OUT_PushPull);
	cs_LO.write(1);
	cs_source.set_mode(MODE_OUT_PushPull);
	cs_source.write(1);
	att_le.set_mode(MODE_OUT_PushPull);
	att_le.write(0);
	
	cs_IF2.set_mode(MODE_OUT_PushPull);
	cs_IF2.write(1);
	cs_LO2.set_mode(MODE_OUT_PushPull);
	cs_LO2.write(1);
	cs_source2.set_mode(MODE_OUT_PushPull);
	cs_source2.write(1);
	att_le2.set_mode(MODE_OUT_PushPull);
	att_le2.write(0);
	sync.set_mode(MODE_OUT_PushPull);
	sync.write(1);
	
	init_path();

	spi.set_speed(1000000);	
	hmc960_init();
	init_LO_source();

	F4GPIO led(GPIOD, GPIO_Pin_2);
	led.set_mode(HAL::MODE_OUT_PushPull);
	led.write(0);

	led.write(SystemCoreClock == 84000000);
	
	configure_af_adc();
	
	calibrate_pll(&vcp);

	while(1)
	{
		fifo_out(vcp);
		handle_vcp(vcp);
		update_sweep();
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
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
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
	i2s_init_str.I2S_Standard = I2S_Standard_Phillips;
	i2s_init_str.I2S_DataFormat = I2S_DataFormat_32b;
	i2s_init_str.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
	i2s_init_str.I2S_AudioFreq = 48000;
	i2s_init_str.I2S_CPOL = I2S_CPOL_High;
	I2S_Cmd(SPI3, DISABLE);
	SPI_I2S_DeInit(SPI3);
	I2S_Init(SPI3, &i2s_init_str);	
	SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);
	
	// wait WS go high
	while(!(GPIOA->IDR & GPIO_Pin_15))
		;
	
	// go!
	I2S_Cmd(SPI3, ENABLE);
}

extern "C" void DMA1_Stream0_IRQHandler(void)
{
	int32_t *data;
	int flag = DMA1->LISR & 0x30;

	// first half buffer
    if (DMA1->LISR & DMA_LISR_HTIF0) {
        DMA1->LIFCR = DMA_LIFCR_CHTIF0;
		data = (int32_t*)codec_data;
    }

	// second half buffer
    if (DMA1->LISR & DMA_LISR_TCIF0) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF0;
		data = (int32_t*)(codec_data + sizeof(codec_data)/4);
    }

	if (sweeping)
		return;

	int sample_count = sizeof(codec_data)/2 / sizeof(uint32_t) / 2;
	int mix_tbl[4] = {1,1,-1,-1};
	memset(integrate, 0, sizeof(integrate));
	for(int i=0; i<sample_count; i++)
	{
		int32_t *p = data + i*2;
		int32_t c3 = p[1] >> 16;
		int32_t c4 = p[0] >> 16;
		int32_t c2 = int16_t(p[0] & 0xffff);
		int32_t c1 = int16_t(p[1] & 0xffff);

		integrate[0] += c1 * mix_tbl[i%4];
		integrate[1] += c1 * mix_tbl[(i+1)%4];
		integrate[2] += c2 * mix_tbl[i%4];
		integrate[3] += c2 * mix_tbl[(i+1)%4];

		integrate[4] += c3 * mix_tbl[i%4];
		integrate[5] += c3 * mix_tbl[(i+1)%4];
		integrate[6] += c4 * mix_tbl[i%4];
		integrate[7] += c4 * mix_tbl[(i+1)%4];
	}

	memcpy(integrate_out, integrate, sizeof(integrate));
	memset(integrate, 0 , sizeof(integrate));

	phase[0] = atan2((double)integrate_out[0], (double)integrate_out[1]);
	phase[1] = atan2((double)integrate_out[6], (double)integrate_out[7]);
	phase_diff = radian_sub(phase[0], phase[1]);

	amp[0] = sqrt(double(integrate_out[0] * integrate_out[0] + integrate_out[1] * integrate_out[1]));
	amp[1] = sqrt(double(integrate_out[2] * integrate_out[2] + integrate_out[3] * integrate_out[3]));
	amp[2] = sqrt(double(integrate_out[4] * integrate_out[4] + integrate_out[5] * integrate_out[5]));
	amp[3] = sqrt(double(integrate_out[6] * integrate_out[6] + integrate_out[7] * integrate_out[7]));
	
	amp_diff = amp[0] / amp[1];
	amp_diff_db = log10(amp_diff) * 20;

	amp_db[0] = log10(amp[0]) * 20;
	amp_db[1] = log10(amp[1]) * 20;
	amp_db[2] = log10(amp[2]) * 20;
	amp_db[3] = log10(amp[3]) * 20;

	if (state < SAMPLE_NEXT_FREQ)
	{
		if (state == SAMPLE_COMMIT)
		{
			fifo_value v = {0};
			v.freq_index = freq_index;

			if (fc<6000e6)
			{			
				v.fwd0r = integrate_out[1];
				v.fwd0i = integrate_out[0];
				v.rev0r = integrate_out[3];
				v.rev0i = integrate_out[2];
				v.rev1r = integrate_out[7];
				v.rev1i = integrate_out[6];
			}
			else
			{
				v.fwd0r = integrate_out[0];
				v.fwd0i = integrate_out[1];
				v.rev0r = integrate_out[2];
				v.rev0i = integrate_out[3];	
				v.rev1r = integrate_out[6];
				v.rev1i = integrate_out[7];
			}

			fifo.push(v);
			TRACE("s:%d\n", freq_index);
		}

		state = sample_state (state + 1);
	}
}
