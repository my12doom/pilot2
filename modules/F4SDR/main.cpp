#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_dma.h"
#include <stm32f4xx_flash.h>
#include <stm32f4xx_tim.h>

#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4SysTimer.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4HSBulk.h>

#include <HAL/aux_devices/si5351.h>
#include <HAL/aux_devices/ADF4351.h>
#include <HAL/aux_devices/MAX2871.h>
#include <HAL/aux_devices/ADF4001.h>
#include "fpga_config.h"
#include "F4SDR.h"

using namespace STM32F4;
using namespace HAL;
using namespace F4SDR;

// constants
int64_t IF_freq = 902.5e6;
int64_t LF_freq = 100e6;

// board resource
F4GPIO led0(GPIOC, GPIO_Pin_6);
F4GPIO led1(GPIOC, GPIO_Pin_7);
ADF4351 LO2;
MAX2871 LO1;
F4SPI spi(SPI3);
F4GPIO cs_adc(GPIOD, GPIO_Pin_0);		// ADC driver & ADC
F4GPIO cs4351(GPIOA, GPIO_Pin_9);		// adf4351
F4GPIO cs2871(GPIOC, GPIO_Pin_9);		// max2871

// signal path
bool stream_enabled = true;
int64_t last_center_freq = 0;
reg_rf_path rfpath = 
{
	0, IF_900, 0, 0,
	IF_AMP_BYPASS, FE_BYPASS,
};

// bulk buffer
#define BUFFER_COUNT 32768
static int16_t adc_buffer[BUFFER_COUNT];
static int16_t *adc_buffer0 = adc_buffer;
static int16_t *adc_buffer1 = adc_buffer+BUFFER_COUNT/2;
int data_valid = 0;
int data_flying = 0;
int64_t parrallel_reset_request = 1;
bool double_buffer = false;

// sweeping
int64_t sweep_start = 1e9;
int64_t sweep_step = 1e6;
int sweep_points = 0;
int sweep_blocks = 1;

int sweep_current_point = 0;
int sweep_current_block = 0;
int block_to_skip = 1;



// functions
void tune(uint64_t center_freq);
void set_adc_gain(bool LNA, uint8_t gain);
int parallel_port_config(bool double_buffer = false);
int usb_event_cb(F4HSBulk *h, int event, void *_p, int size);
void fpga_reg_write(uint8_t add, uint8_t value);
uint8_t fpga_reg_read(uint8_t add);
void sweep_next();

void request_port_reset()
{
	DMA_Cmd(DMA2_Stream6, DISABLE);
	
	parrallel_reset_request = systimer->gettime();	
}

void set_adc_gain(bool LNA, uint8_t gain)
{
	request_port_reset();
	spi.set_mode(0,0);
	spi.set_speed(4000000);
	cs_adc.set_mode(MODE_OUT_PushPull);
	cs_adc.write(1);
	systimer->delayus(1);
	
	cs_adc.write(0);
	spi.txrx(0x32);
	spi.txrx((LNA ? 0x80 : 0) | gain);
	cs_adc.write(1);
	
	request_port_reset();
}

void set_rf_path(uint8_t path)
{
	request_port_reset();
	reg_rf_path *p = (reg_rf_path*)&path;
	if (!p->IF_sel_override)
		p->IF_select = rfpath.IF_select;
	
	rfpath = *p;
	fpga_reg_write(0, path);
	request_port_reset();
}

extern "C" void DMA2_Stream6_IRQHandler(void)
{
	bool overflow = false;
	if(DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6))
	{
		int piece = !double_buffer ? 2 : (2-DMA_GetCurrentMemoryTarget(DMA2_Stream6));
		if (data_valid & piece)
			overflow = true;
		data_valid |= piece;
	}
	
	if (!double_buffer)
	{
		if(DMA_GetITStatus(DMA2_Stream6, DMA_IT_HTIF6))
		{
			if (data_valid & 1)
				overflow = true;
			data_valid |= 1;
		}
	}
	
	DMA_ClearITPendingBit(DMA2_Stream6, DMA_FLAG_HTIF6 | DMA_FLAG_TCIF6);
	
	led1.write(!overflow);
}

/*
extern "C" void DMA2_Stream2_IRQHandler(void)
{
	bool overflow = false;
	if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))
	{
		if (data_valid & 2)
			overflow = true;
		data_valid |= 2;
	}
	if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_HTIF2))
	{
		if (data_valid & 1)
			overflow = true;
		data_valid |= 1;
	}
	DMA_ClearITPendingBit(DMA2_Stream2, DMA_FLAG_HTIF2 | DMA_FLAG_TCIF2);
	
	led1.write(!overflow);
		
}
*/



int parallel_port_config(bool use_double_buffer /*= false*/)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	double_buffer = use_double_buffer = true;
	data_valid = 0;
	data_flying = 0;
	parrallel_reset_request = 0;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);	
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_DeInit(TIM1);
	TIM_InternalClockConfig(TIM1);
	TIM_TimeBaseStructure.TIM_Prescaler = 0xff;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=0xffff;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter    = 0x0;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
	TIM_Cmd(TIM1,ENABLE);
	
	TIM_DMACmd(TIM1, TIM_DMA_CC1, ENABLE ); // Enable TIM1_CC1 DMA Requests
	
	DMA_InitTypeDef DMA_InitStructure = {0};
	DMA_InitStructure.DMA_Channel = DMA_Channel_0; 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(GPIOE->IDR));
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adc_buffer[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = sizeof(adc_buffer)/sizeof(adc_buffer[0]);
	if (double_buffer)
		DMA_InitStructure.DMA_BufferSize /= 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
	DMA_DeInit(DMA2_Stream6);
	DMA_Init(DMA2_Stream6, &DMA_InitStructure);
	
	if (use_double_buffer)
	{
		DMA_DoubleBufferModeConfig(DMA2_Stream6, (uint32_t)(adc_buffer + sizeof(adc_buffer)/sizeof(adc_buffer[0]/2)),
			DMA_GetCurrentMemoryTarget(DMA2_Stream6) ? DMA_Memory_1 : DMA_Memory_0);
		DMA_DoubleBufferModeCmd(DMA2_Stream6, ENABLE);
	}
	
	DMA_Cmd(DMA2_Stream6, ENABLE);
	
	DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Stream6, DMA_IT_HT, use_double_buffer ? DISABLE : ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	return 0;
}

int handle_IN_event(F4HSBulk *h, control_verndor_transfer *t)
{
	char outbuf[64];
	int outsize = 0;

	switch (t->request)
	{
	case ping:
		for(int i=0; i<t->length; i++)
			outbuf[i] = i;
		outsize = t->length;
		break;
	}

	h->ioctl(tx_ctrl_block, outbuf, outsize);

	return 0;
}

int erase_fpga_rom_sectors()
{
	uint32_t sectors_to_erase[] = {FLASH_Sector_4, FLASH_Sector_5, FLASH_Sector_6, FLASH_Sector_7, FLASH_Sector_8, FLASH_Sector_9, FLASH_Sector_10, };
	int i,j;
	FLASH_Unlock();
	for(i=0; i<sizeof(sectors_to_erase)/sizeof(sectors_to_erase[0]); i++)
	{
		// empty check
		uint32_t *p = (uint32_t*)(0x08010000 + i * (i==1?0x10000:0x20000));
		int word_count = i==0?0x4000:0x8000;
		for(j=0; j<word_count; j++)
		{
			if (p[j] != 0xffffffff)
			{
				FLASH_EraseSector(sectors_to_erase[i], VoltageRange_3);
				FLASH_WaitForLastOperation();
				break;
			}
		}
	}
	
	return 0;
}

int handle_OUT_event(F4HSBulk *h, control_verndor_transfer *t)
{
	int64_t freq = t->length == 8 ? *(uint64_t*)t->data : 0;
	switch (t->request)
	{
	case reset_mcu:
		NVIC_SystemReset();
		break;

	case stream_enable:
		stream_enabled = t->value;
		break;

	case erase_fpga_rom:		
		if (t->length == 4 && *(uint32_t*)t->data == 0xDEADBEAF)
		{
			erase_fpga_rom_sectors();
		}
		break;
	
	case write_fpga_rom:
		{
			uint32_t address = (t->value << 16) | t->index;
			uint32_t *data = (uint32_t*)t->data;
			for(int i=0; i<t->length; i+=4)
				FLASH_ProgramWord(address+i, data[i/4]);
		}
		break;

	case F4SDR::tune:
		if (t->length == 8)
			::tune (*(uint64_t*)t->data);
		break;

	case set_gain:
		if (t->length == 1)
			set_adc_gain(t->data[0]&0x80, t->data[0]&0x7f);
		break;
	case F4SDR::set_path:
		if (t->length == 1)
			set_rf_path(t->data[0]);
		break;

	case set_sweep_start:
		sweep_start = freq;
		break;

	case set_sweep_step:
		sweep_step = freq;
		break;

	case set_sweep_points:
		sweep_points = freq;	// little endian only
		break;

	case set_sweep_blocks:
		sweep_blocks = t->value;
		break;

	default:
		break;
	}
	return 0;
}

int64_t last_tx_time[3] = {0};
int tx_dt[2];
int dt;

int usb_event_cb(F4HSBulk *h, int event, void *_p, int size)
{
	switch (event)
	{
	case tx_done:
		data_valid &= data_flying ? 0xfe : 0xfd;
		

		
		// fall through
	case tx_ready:
		if (data_valid)
		{

			int next_part = (data_valid == 3) ? (1-data_flying) : (data_valid & 1);
			
			if (next_part == data_flying)
				led0.write(0);
			else
				led0.write(1);
			
			data_flying = next_part;
			int16_t *buffer_to_send = data_flying ? adc_buffer : (adc_buffer + BUFFER_COUNT/2);
			
			// sweeping handling
			if (sweep_points > 0)
			{
				// skip blocks for LO lock/transients
				if (block_to_skip > 0)
				{
					block_to_skip --;
					return -1;
				}

				// fill header
				sweep_header *header = (sweep_header*)buffer_to_send;
				header->center_freq = last_center_freq;
				header->block_index = sweep_current_block;
			}
			
			if (!stream_enabled)
				return -1;
			
			/*
			last_tx_time[0] = last_tx_time[1];
			last_tx_time[1] = last_tx_time[2];
			last_tx_time[2] = systimer->gettime();
			
			tx_dt[0] = last_tx_time[1] - last_tx_time[0];
			tx_dt[1] = last_tx_time[2] - last_tx_time[1];
			*/
			

			h->ioctl(tx_block, buffer_to_send, sizeof(adc_buffer)/2);			
			
			// sweeping handling
			if (sweep_points > 0)
			{
				// how many blocks sent for this point?
				sweep_current_block ++;
				if (sweep_current_block >= sweep_blocks)
				{
					int l = systimer->gettime();
					sweep_next();
					dt = systimer->gettime() - l;
				}
			}			
		}
		else
		{
			return -1;	// no more data for now
		}
		break;

	case control_IN:
		handle_IN_event(h, (control_verndor_transfer*)_p);
		break;

	case control_OUT:
		handle_OUT_event(h, (control_verndor_transfer*)_p);
		break;

	case rx_ready:
		static uint8_t rx_buffer[512];
		h->ioctl(rx_block, rx_buffer, sizeof(rx_buffer));
		break;
	
	case rx_done:
		{
			// handle incoming data
			uint8_t *p = (uint8_t *)_p;
			if (p[0] == 0x81)
			{
				erase_fpga_rom_sectors();
			}
			
			else if (p[0] == 0x82)
			{
				uint32_t *data = (uint32_t*)(p+1);
				uint32_t address = data[0];
				int count = data[1];
				data += 2;
				
				for(int i=0; i<count/4; i++)
					FLASH_ProgramWord(address+i*4, data[i]);
			}
			else if (p[0] == 0x83)
			{
				NVIC_SystemReset();
			}
			else if (p[0] == 0x84)
			{
				// do nothing
			}
		}

		break;
	}

	return 0;
}

int write_ad9251(int16_t add, uint8_t v)
{
	cs_adc.write(0);
	spi.txrx(0x31);
	spi.txrx((add>>8)&0xf);
	spi.txrx(add&0xff);
	spi.txrx(v);
	cs_adc.write(1);

	return 0;
}

uint8_t read_ad9251(int16_t add)
{
	uint8_t v;
	cs_adc.write(0);
	spi.txrx(0x31);
	spi.txrx(0x80 | ((add>>8)&0xf));
	spi.txrx(add&0xff);
	v = spi.txrx(0);
	cs_adc.write(1);

	return v;
}

uint8_t readback[20];
int configure_ad9251()
{
	spi.set_mode(0,0);
	spi.set_speed(4000000);

		
	//write_ad9251(0x00, 0x3C);	// software reset
	//write_ad9251(0x05, 0x03);	// select both channel
	//write_ad9251(0xff, 0x01);	// execute
	
	//systimer->delayms(1);
	

	write_ad9251(0x05, 0x03);	// select both channel
	write_ad9251(0x14, 0x21);	// use twos complement output, interleaved
	write_ad9251(0x17, 0x20);	// delay clock 4.5ns
	write_ad9251(0x15, 0x22);	// weak cmos drive
	write_ad9251(0x16, 0x80);	// DCO invert

	//write_ad9251(0x14, 0x01);	// use twos complement output
	//write_ad9251(0x17, 0x00);	// no delay

	//write_ad9251(0x0b, 0x00);	// clock divider : /1
	write_ad9251(0x0b, 0x01);	// clock divider : /2
	write_ad9251(0x09, 0x01);	// clock duty cycle stabalize
	write_ad9251(0xff, 0x01);	// execute
	
	// disable channel B
	/*
	write_ad9251(0x05, 0x02);	// select channel B
	write_ad9251(0x14, 0x21);	// disable
	write_ad9251(0xff, 0x01);	// execute
	*/
	
	for(int i=0; i<20; i++)
		readback[i] = read_ad9251(i);
	
	return 0;
}

void tune(uint64_t center_freq)
{
	if (center_freq == last_center_freq)
		return;

	last_center_freq = center_freq;

	request_port_reset();
	// 1st IF filter:
	// 900M option(NDF9299) center:902.5 passband: 890 - 915 Mhz, stop band: <880Mhz or > 925Mhz
	// 800M option(NDF8010) center:836.5 passband: 824 - 849 Mhz, stop band:  <800Mhz or > 869Mhz
	uint8_t if_select_prev = rfpath.IF_select;
	rfpath.IF_select = center_freq > ((902.5e6f+837.0e6f)/2) ? IF_800 : IF_900;
	rfpath.IF_select = IF_900;
	rfpath.front_end = FE_BYPASS;
	//rfpath.front_end = FE_LNA;
	rfpath.IF_amp = IF_AMP;
	if (if_select_prev != rfpath.IF_select)
	{
		IF_freq = (rfpath.IF_select == IF_800) ? 836.5e6f : 902.5e6f;
		set_rf_path(*(uint8_t*)&rfpath);
	}
	
	if (center_freq > IF_freq*2 + LF_freq)
	{
		// low side LO1, low side LO2
		LO1.set_freq(center_freq-IF_freq);
		LO2.set_freq(IF_freq - LF_freq);
	}
	else
	{
		// high side LO1, high side LO2
		LO1.set_freq(center_freq + IF_freq);
		LO2.set_freq(IF_freq + LF_freq); 
	}
	
	//LO1.set_output(false, false, false);
	//LO2.set_freq(100e6);
	//LO2.set_output(false, false, false);
	
	request_port_reset();
}

uint8_t rb;

void fpga_reg_write(uint8_t add, uint8_t value)
{
	request_port_reset();
	spi.set_mode(0,0);
	spi.set_speed(4000000);
	
	cs_adc.write(0);
	spi.txrx(0x33);
	spi.txrx(add);
	spi.txrx(value);
	cs_adc.write(1);

	request_port_reset();
}

uint8_t fpga_reg_read(uint8_t add)
{
	//request_port_reset();
	spi.set_mode(0,0);
	spi.set_speed(4000000);
	
	cs_adc.write(0);
	spi.txrx(0x33);
	spi.txrx(add | 0x80);
	uint8_t rb = spi.txrx(0xAA);
	cs_adc.write(1);
	
	//request_port_reset();

	return rb;
}

uint8_t tx[32];
uint8_t rx[32];
void fpga_reg_read_multi(uint8_t start, uint8_t *out, int count)
{
	spi.set_mode(0,0);
	spi.set_speed(4000000);
	tx[0] = 0x33;
	tx[1] = start | 0x80;
	
	cs_adc.write(0);
	spi.txrx2(tx, rx, count+2);
	/*
	spi.txrx(0x33);
	spi.txrx(start | 0x80);
	for(int i=0; i<count; i++)
		out[i] = spi.txrx(0xAA);
	*/
	cs_adc.write(1);
	memcpy(out, rx+2, count);
}

void write_4001_reg(uint32_t v)
{
	cs_adc.write(0);
	spi.txrx(0x34);
	spi.txrx((v>>16)&0xff);
	spi.txrx((v>>8)&0xff);
	spi.txrx((v>>0)&0xff);

	cs_adc.write(1);
}

void configure_clock_pll()
{
	ADF4001_REG0 reg0 = {0};
	ADF4001_REG1 reg1 = {0};
	ADF4001_REG2 reg2 = {0};
	spi.set_mode(0, 0);
	spi.set_speed(20000000);

	do
	{
		reg0.ADDR = 0;
		reg0.anti_backlash = 0;
		reg0.lock_detect_precision = 1;
		reg0.test_mode = 0;
		reg0.R = 3072/4;							// "ref" divider: 61.44Mhz / 3072 = 20khz

		reg1.ADDR = 1;
		reg1.reserved = 0;
		reg1.N = 2000/4;							// "RF" divider: 40Mhz / 2000 = 20khz
		reg1.cp_gain = 3;

		reg2.ADDR = 2;
		reg2.counter_reset = 0;
		reg2.power_down1 = reg2.power_down2 = 0;
		reg2.mux_out = 1;	// digital lock detect
		reg2.polarity = 0;
		reg2.cp_3state = 0;
		reg2.fast_lock_en = 0;
		reg2.fast_lock_mode = 0;
		reg2.timer_control = 0;
		reg2.cp_current1 = reg2.cp_current2 = 7;		// 5mA

		write_4001_reg(*(uint32_t*)&reg0);
		write_4001_reg(*(uint32_t*)&reg1);
		write_4001_reg(*(uint32_t*)&reg2);
	} while(0);
	
}

uint8_t fpga_read[9] = {0};
int fpga_regs_test()
{
	fpga_reg_write(1, 0x3f);
	fpga_reg_write(2, 0xff);
	
	fpga_read[0] = fpga_reg_read(0);
	fpga_read[1] = fpga_reg_read(1);
	fpga_read[2] = fpga_reg_read(2);
	return 0;
}

uint32_t int_times = 0;
uint32_t next_int = 61440;
uint8_t reg_shift = 8;

int64_t timeout = 0;
int32_t got[2] = {0};
double gotlpf = 0;
float gotdb = 0;
bool int_disable = true;
void read_integrate()
{
	if (int_disable)
		return;
	
	if (systimer->gettime() < timeout)
		return;
	
	if (next_int != int_times)
	{
		int_times = next_int;
		uint8_t tmp[4];
		memcpy(tmp, &int_times, 4);
		for(int i=0; i<4; i++)
		{
			fpga_reg_write(i+4, tmp[i]);
			fpga_read[i] = fpga_reg_read(i+4);
		}
		fpga_reg_write(8, reg_shift);
	}
	
	//request_port_reset();
	/*
	fpga_read[0] = fpga_reg_read(0);
	fpga_read[1] = fpga_reg_read(1);
	fpga_read[2] = fpga_reg_read(2);

	for(int i=0; i<8; i++)
		fpga_read[i] = fpga_reg_read(4+i);
	*/
	fpga_reg_read_multi(64, fpga_read, 9);
	
	memcpy(&got, fpga_read+1, 8);
	
	//got[0] += 30720;
	//got[1] += 30720;
	
	int64_t v = (int64_t)got[0]*(int64_t)got[0]+(int64_t)got[1]*(int64_t)got[1];
	
		
	gotlpf = gotlpf *0.95 + 0.05 * v;
	
	gotdb = log10(gotlpf)*10;
	//request_port_reset();
	
	timeout = systimer->gettime() + 50000;
}

void sweep_next()
{
	sweep_current_point = (sweep_current_point + 1) % sweep_points;
	int64_t center_freq = sweep_start + sweep_step * sweep_current_point;
	block_to_skip = 0;
	sweep_current_block = 0;

	::tune(center_freq);

	// do parallel port reset
	cs_adc.write(0);
	parallel_port_config();
	systimer->delayus(50);
	parrallel_reset_request = 0;
	cs_adc.write(1);
}

extern "C" void SetSysClock();
int main()
{
	cs_adc.set_mode(MODE_OUT_PushPull);
	cs_adc.write(1);	

	// use clock from USB3320
	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));		// switch to HSI
	
	F4GPIO usb_reset(GPIOC, GPIO_Pin_14);
	usb_reset.set_mode(MODE_OUT_PushPull);
	usb_reset.write(0);
	systimer->delayms(10);
	usb_reset.write(1);
	systimer->delayms(5);
	SetSysClock();
	SystemCoreClockUpdate();
	led0.write(1);
	led1.write(1);
	led0.set_mode(MODE_OUT_OpenDrain);
	led1.set_mode(MODE_OUT_OpenDrain);
	if (SystemCoreClock != 168000000)
		led0.write(0);
	else
		led1.write(0);
	PWR->CR &= ~PWR_CR_VOS;
	
	//while(0)
	{
		systimer->delayms(50);
	}
	
	// erase fpga
	F4GPIO fpga_erase(GPIOA, GPIO_Pin_0);
	fpga_erase.set_mode(MODE_OUT_PushPull);
	fpga_erase.write(0);
	systimer->delayms(5);
	fpga_erase.write(1);
	
	// fpga configureation
	F4GPIO init_b(GPIOA, GPIO_Pin_6);
	fpga_config(&init_b);
	
	configure_clock_pll();
	fpga_regs_test();


	// LO2 config
	LO2.init(&spi, &cs4351);
	LO2.set_ref(26000000, false, false, 1);
	LO2.set_freq(IF_freq - LF_freq);		// 1st IF filter passband: 890 - 915Mhz, stop band: <880Mhz or > 925Mhz
	LO2.set_output(true, false, true);		// baseband mapped to 888 of IF2.
	
	// LO1 config
	LO1.init(&spi, &cs2871);
	LO1.set_ref(26000000, false, false, 1);
	LO1.set_freq(1507000000);
	LO1.set_output(true, true, false);

	// ADC config
	configure_ad9251();

	// IF config	
	set_adc_gain(true, 1);
	
	::tune(2400e6);
	
	// do a soft reset if first power on
	// adc clock divider not configured properly may corrupt fpga filter states
	if (!(RCC->CSR & 0x10000000))
		NVIC_SystemReset();


	// USB
	F4HSBulk bulk;
	bulk.ioctl(set_callback, (void*)usb_event_cb, 0);
	
	while(1)
	{
		//read_integrate();
		if (parrallel_reset_request > 0 && systimer->gettime() > parrallel_reset_request)
		{
			parrallel_reset_request = 0;

			// ADC config
			cs_adc.write(0);
			parallel_port_config();
			systimer->delayms(1);
			cs_adc.write(1);
		}
	}
}
