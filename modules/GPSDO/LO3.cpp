#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4SysTimer.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4Timer.h>
#include <HAL/STM32F4/F4ADC.h>
#include <HAL/Interface/II2C.h>
#include <HAL/aux_devices/OLED_I2C.h>

#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_dma.h>

#include <HAL/aux_devices/ADF4001.h>
#include "../scope/adc.h"

using namespace STM32F4;
using namespace HAL;

F4GPIO cs_div(GPIOA, GPIO_Pin_4);

F4Interrupt trig;
F4SPI spi(SPI1);
dma_adc adc;

int counter = 0;
float avgf;
float lpf_p = NAN;
float lpf_alpha = 0.7;
int pulse_length = 0;
int64_t pulse_start = 0;
int64_t capture_start_time = 0;
int capture_length = 0;
ADF4001_REG0 reg0 = {0};
ADF4001_REG1 reg1 = {0};
ADF4001_REG2 reg2 = {0};
int count;

float input_freq;
float cc1freq;
int cc1_count = 0;

int counter_read();
int counter_stop();
int counter_start();

void capture_start()
{
	capture_start_time = systimer->gettime();
	counter_start();
	adc.init(GPIOA, 2);
	adc.begin();
	counter ++;
}

void capture_end(bool discard_data = false)
{
	capture_length = (systimer->gettime() - capture_start_time);
	int fcount = counter_read();
	cc1_count = fcount;
	adc.stop();
	counter_stop();
		
	int P = powf(2, reg2.prescaler) * 8;
	int N = reg1.N * P + reg1.reserved;

	if (!discard_data)
	{
		int avg = 0;
		count = adc.full() ? max_adc_data_count : adc.pos();
		if ( count < 10)
			return;
		for(int i=0; i<count; i++)
			avg += adc.adc_data[i];

		avgf = (float)avg/count;

		lpf_p = isnan(lpf_p) ? avgf : (lpf_p * lpf_alpha + avgf*(1-lpf_alpha));
		
		
		float period = (float)capture_length/fcount;
		cc1freq = (float)fcount/capture_length;
		input_freq = cc1freq * N*4;
	}
}


int pwm_init()
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	// open B1(TIM4_CH3) as output
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	
	// Time base configuration
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = 4094;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_Cmd(TIM3, ENABLE);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	
	TIM_SetCompare1(TIM3, 2048);
	
	return 0;
}

int pwm_set(int16_t value)
{
	TIM_SetCompare1(TIM3, value);

	return 0;
}

int write_4001_reg(F4GPIO &cs, uint32_t v)
{
	cs.write(0);

	spi.txrx((v>>16)&0xff);
	spi.txrx((v>>8)&0xff);
	spi.txrx((v>>0)&0xff);

	cs.write(1);

	return 0;
}

int overflow = 0;
extern "C" void TIM1_UP_TIM10_IRQHandler()
{
	if (TIM1->SR & TIM_IT_Update)
	{
		overflow ++;
		TIM_ClearFlag(TIM1, TIM_IT_Update);	
	}
}


int freq_counter_init()
{
	spi.set_mode(0, 0);
	spi.set_speed(20000000);
	cs_div.set_mode(HAL::MODE_OUT_PushPull);
	cs_div.write(1);



	do
	{
		reg0.ADDR = 0;
		reg0.anti_backlash = 0;
		reg0.lock_detect_precision = 1;
		reg0.test_mode = 0;
		reg0.R = 5;

		reg1.ADDR = 1;
		reg1.reserved = 0;
		reg1.N = 4;
		reg1.cp_gain = 3;

		reg2.ADDR = 2;
		reg2.counter_reset = 0;
		reg2.power_down1 = reg2.power_down2 = 0;
		reg2.mux_out = 2;	// N divided out
		reg2.polarity = 0;
		reg2.cp_3state = 1;
		reg2.fast_lock_en = 0;
		reg2.fast_lock_mode = 0;
		reg2.timer_control = 0;
		reg2.cp_current1 = reg2.cp_current2 = 0;
		reg2.prescaler = 3;		// P = 32, N=8, N = BP+ A = 256

		write_4001_reg(cs_div, *(uint32_t*)&reg0);
		write_4001_reg(cs_div, *(uint32_t*)&reg1);		
		write_4001_reg(cs_div, *(uint32_t*)&reg2);
	} while(0);
	
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);	


	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_DeInit(TIM1);
	TIM_InternalClockConfig(TIM1);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=0xffff;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter    = 0x0;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
	
	
	TIM_ITRxExternalClockConfig(TIM1, TIM_TIxExternalCLK1Source_TI1);	
	
	// over flow interrupt
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// enable
	TIM_Cmd(TIM1,ENABLE);
	
	return 0;
}


int counter_read()
{
	return TIM1->CNT + overflow * (TIM1->ARR+1);	
}

int counter_stop()
{	
	// power down
	TIM_Cmd(TIM1, DISABLE);
	//reg2.power_down1 = reg2.power_down2 = 1;
	//write_4001_reg(cs_div, *(uint32_t*)&reg2);
	
	return 0;
}

int counter_start()
{
	// reset & start timer
	TIM_Cmd(TIM1, DISABLE);
	TIM1->CNT = 0;
	overflow = 0;
	TIM_Cmd(TIM1,ENABLE);
	//reg2.power_down1 = reg2.power_down2 = 0;
	//write_4001_reg(cs_div, *(uint32_t*)&reg2);
	
	return 0;
}

F4GPIO key(GPIOB, GPIO_Pin_8);

#if 1
// LO2
F4GPIO cs_lo2(GPIOA, GPIO_Pin_4);
F4GPIO ce(GPIOA, GPIO_Pin_3);
F4GPIO rf_en(GPIOA, GPIO_Pin_2);
F4GPIO led(GPIOB, GPIO_Pin_15);
F4GPIO sel1(GPIOC, GPIO_Pin_13);
F4GPIO sel2(GPIOC, GPIO_Pin_14);
void hw_init(){}
#else
// LO3
F4GPIO cs_lo2(GPIOB, GPIO_Pin_1);
F4GPIO ce(GPIOB, GPIO_Pin_2);
F4GPIO rf_en(GPIOB, GPIO_Pin_0);

F4GPIO cs_att(GPIOA, GPIO_Pin_1);
F4GPIO dbl(GPIOA, GPIO_Pin_0);
F4GPIO dbl_n(GPIOC, GPIO_Pin_15);
F4GPIO sel1(GPIOC, GPIO_Pin_13);
F4GPIO sel2(GPIOC, GPIO_Pin_14);
F4GPIO sw1(GPIOA, GPIO_Pin_2);
F4GPIO sw2(GPIOA, GPIO_Pin_3);
F4GPIO sw3(GPIOA, GPIO_Pin_4);
F4GPIO led(GPIOB, GPIO_Pin_15);

void set_att(int att)
{
	att &= 0x7f;
	uint8_t tx = 0;
	for(int i=0; i<8; i++)
		tx |= ((att>>i)&1) << (7-i);

	spi.txrx(tx);
	cs_att.write(1);
	systimer->delayus(1);
	cs_att.write(0);
}

void hw_init()
{
	cs_att.set_mode(HAL::MODE_OUT_PushPull);
	dbl.set_mode(HAL::MODE_OUT_PushPull);
	dbl_n.set_mode(HAL::MODE_OUT_PushPull);
	sel1.set_mode(HAL::MODE_OUT_PushPull);
	sel2.set_mode(HAL::MODE_OUT_PushPull);
	sw1.set_mode(HAL::MODE_OUT_PushPull);
	sw2.set_mode(HAL::MODE_OUT_PushPull);
	sw3.set_mode(HAL::MODE_OUT_PushPull);
	
	cs_att.write(1);
	dbl.write(1);
	dbl_n.write(0);
	
	sel1.write(0);
	sel2.write(1);
	
	sw1.write(0);
	sw2.write(0);
	sw3.write(0);
	
	spi.set_mode(0, 0);
	spi.set_speed(5000000);
	set_att(0*4);
}

#endif


void trig_cb(void *p, int flags)
{
	if (flags == interrupt_falling)
	{
		pulse_length = capture_length;
		capture_end();
		led.write(0);
	}
	else if (flags == interrupt_rising)
	{
		pulse_start = capture_start_time;
		capture_start();
		led.write(1);
	}
	else
	{
	}
}


uint32_t v;
uint8_t v4[4];
void _write_lo2(uint32_t R)
{
	v = R;
	cs_lo2.write(0);
	for(int i=0; i<4; i++)
	{
		v4[i] = v>>24;
		spi.txrx(v>>24);
		v<<=8;
	}
	cs_lo2.write(1);
}

void write_lo2(uint32_t R)
{
	uint8_t tx[4] = {R>>24, R>>16, R>>8, R};
	uint8_t rx[4];
	
	cs_lo2.write(0);
	spi.txrx2(tx, rx, 4);
	cs_lo2.write(1);
}



#include "HAL/aux_devices/ADF4355.h"

uint32_t endian_swap32(uint32_t v)
{
	uint8_t *p = (uint8_t *)&v;
	uint8_t t = p[0];
	p[0] = p[3];
	p[3] = t;
	t = p[1];
	p[1] = p[2];
	p[2] = t;

	return v;
}

uint32_t frac1;
uint8_t vv;

const int freq_points = 25;
uint8_t readback1[4];
uint8_t readback2[4];
uint8_t vco_core[freq_points];
uint8_t vco_band[freq_points];
uint8_t vco_bias[freq_points];
int64_t freq = 4000e6;
int dt;
int LO2()
{
	hw_init();
	spi.set_mode(0, 0);
	spi.set_speed(52000000);
	cs_lo2.set_mode(HAL::MODE_OUT_PushPull);
	cs_lo2.write(1);
	ce.set_mode(HAL::MODE_OUT_PushPull);
	ce.write(1);
	rf_en.set_mode(HAL::MODE_OUT_PushPull);
	rf_en.write(1);
	led.set_mode(HAL::MODE_OUT_PushPull);
	led.write(0);
	F4GPIO mux_out(GPIOA, GPIO_Pin_6);
	
	int64_t fref = 40e6;
	
	int N = freq/fref;
	frac1 = (freq - N*fref)*16777216/fref;
		
	adf4355_reg0 r0 = {0, N, 0, 1, };	// N = 100, prescaler=4/5, autocal enable
	adf4355_reg1 r1 = {1, frac1};		// FRAC = 0,
	adf4355_reg2 r2 = {2, 1, 0};		// mod2=1, frac2 = 0
	adf4355_reg3 r3 = {3, 0, 0, 0, 0};	// all phase ops disabled
	adf4355_reg4 r4 = {4, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 7};	// PD/COUNT/CP enabled, 3.3V MUXOUT, normal polarity, single ended ref
																// ICP = 0.31mA, double buffer on, R = 1, divider disabled, doubler enabled, digital LD
	adf4355_reg5 r5 = {5, 0x0080002};
	adf4355_reg6 r6 = {6, 3, 1, 3, 1,   0, 0, 0, 0,   0, 1, 0xA, };	// DIV/1, fundamental FB, both output 5dbm enabled
	adf4355_reg7 r7 = {7, 0, 3, 0, 0,   0, 6, 0, 1,   0x4};			// frac LD, 12ns LD, LOL disable, LD cycle = 1024, 
																	// vco readback = calibration complete,  LE sync
	adf4355_reg8 r8 = {8, 0x102D042};
	adf4355_reg9 r9 = {9, 31, 10, 40, 20};				// vco cal div = 20, fpfd = 40M, fbandsel = 125k, 8us/cycle, 
	adf4355_reg10 r10 = {10, 1, 1, 100, 0x300, 1};		// ADC clk = 40M/100 = 400khz, vco read = core and band
	adf4355_reg11 r11 = {11, 0, 3, 1, 0x6};
	adf4355_reg12 r12 = {12, 0x41, 1024};				// resync clock = 1024


	r4.MUXOUT = 6;
	r4.CURRENT = 12;
	r0.AUTOCAL = 1;
	r6.dbl_output_n = 0;
	r6.neg_bleed = 1;
	r6.CP_bleed_current = 130;
	r6.rf_divider = 1;
	
	write_lo2(r12);
	write_lo2(r11);
	write_lo2(r10);
	write_lo2(r9);
	write_lo2(r8);
	write_lo2(r7);
	write_lo2(r6);
	write_lo2(r5);
	write_lo2(r4);
	write_lo2(r3);
	write_lo2(r2);

	
	write_lo2(r1);
	systimer->delayus(200);
	write_lo2(r0);
	
	key.set_mode(HAL::MODE_IN);
	
	bool last_key = false;
	bool sel = true;
	while(1)
	{
		bool k = key.read();
		if (k && !last_key)
			sel = !sel;
		last_key = k;
		
		led.write(sel);
		sel1.write(sel);
		sel2.write(!sel);
//		led.write(mux_out.read());
	}
	
	r4.MUXOUT = 7;
	write_lo2(r4);	


	int64_t freq_start = 3800e6;
	int64_t freq_step = 6e3;
	
	// build vco table
	for(int i=0; i<freq_points; i++)
	{
		// set R7 to vco autocal completion readback
		r7.vco_readback = 6;
		write_lo2(r7);

		// calculate and program N
		freq = freq_start + freq_step * i;
		N = freq/fref;
		frac1 = (freq - N*fref)*16777216/fref;
		r0.N_INT = N;
		r1.N_FRAC = frac1;
		write_lo2(r1);
		systimer->delayus(200);
		write_lo2(r0);
		
		// wait for lock
		while(!mux_out.read())
			;
		
		// set R7 to vco readback
		r7.vco_readback = 7;
		write_lo2(r7);

		// set R10 to vco core and band readback
		r10.VCO_READ = 1;
		write_lo2(r10);
		
		// readback
		// spi set to sample 2nd edge
		spi.set_mode(0, 1);
		for(int j=0; j<2; j++)
			readback1[j] = spi.txrx(0);
		
		// revert spi
		spi.set_mode(0, 0);
		
		// set R10 to vco bias code readback
		r10.VCO_READ = 3;
		write_lo2(r10);

		// readback
		// spi set to sample 2nd edge
		spi.set_mode(0, 1);
		for(int j=0; j<2; j++)
			readback2[j] = spi.txrx(0);
		
		// revert spi
		spi.set_mode(0, 0);
		
		vco_core[i] = (readback1[0]>>4) & 0x7;
		vco_band[i] = ((readback1[0] & 0xf) << 4) | (readback1[1]>>4);
		vco_bias[i] = readback2[1] >> 4;
	}
	
	
	r0.AUTOCAL = 0;
	r4.MUXOUT = 6;
	r4.CURRENT = 15;
	write_lo2(r4);
	r10.VCO_WRITE = 1;
	write_lo2(r10);
	
	// gen freq table
	int n_int[freq_points];
	int n_frac[freq_points];
	for(int i=0; i<freq_points; i++)
	{
		freq = freq_start + freq_step * i;
		n_int[i] = N = freq/fref;
		n_frac[i] = (freq - N*fref)*16777216/fref;
	}
	
	// sweep!
	while(1)
	{
		bool first = false;
		int vco_core_tbl[5] = {1, 1, 2, 4, 8};
		for(int j=0; j<freq_points*2; j++)
		{
			led.write(0);
			int i = (j>=freq_points) ? (freq_points*2-1-j) : j;
			int64_t t = systimer->gettime();
			
			// calculate N
			r0.N_INT = n_int[i];
			r1.N_FRAC = n_frac[i];			
			
			// program vco
			//if (i==0)
			{
				r11.vco_band = vco_band[i];
				r11.vco_bias = vco_bias[i];
				r11.vco_core = vco_core_tbl[vco_core[i]];
				write_lo2(r11);
				r10.VCO_WRITE = 4 - r10.VCO_WRITE;
				write_lo2(r10);
				write_lo2(r11);
				first = true;
			}

			// program N
			write_lo2(r1);
			write_lo2(r0);
			
			dt = systimer->gettime() - t;
			
			int64_t timeout = systimer->gettime() + 650;
			while (systimer->gettime() < timeout)
				led.write(mux_out.read());
		}
	}
	
	while(1)
	{
		vv = spi.txrx(0);
		led.write(vv);
		continue;
		for(int i=0; i<40; i++)
		{
			freq = 6960e6 + (i>20?(40-i):i)*1e5;
			N = freq/fref;
			frac1 = (freq - N*fref)*16777216/fref;
			adf4355_reg0 r0 = {0, N, 0, 1, };	// N = 100, prescaler=4/5, autocal enable
			adf4355_reg1 r1 = {1, frac1};		// FRAC = 0,
			write_lo2(r1);
			systimer->delayus(200);
			write_lo2(r0);
			systimer->delayms(50);
		}
		
	}
}

int clk_count = 0;
int dtus;
int main()
{
	LO2();
	led.set_mode(HAL::MODE_OUT_PushPull);
	trig.init(GPIOA, GPIO_Pin_11, interrupt_rising_or_falling);
	trig.set_callback(trig_cb, NULL);
	
	pwm_init();
	pwm_set(1.5*4095/3.3);
	
	pwm_init();
	
	
	int64_t sys = systimer->gettime();
	dtus = systimer->gettime() - sys;

	devices::OLED96 oled;
	F4GPIO scl(GPIOA, GPIO_Pin_10);
	F4GPIO sda(GPIOA, GPIO_Pin_9);
	I2C_SW i2c(&scl, &sda);
	i2c.set_speed(1);
	oled.init(&i2c, 0x78);
	oled.show_str(0, 0, "HelloWorld");

	freq_counter_init();
	
	while(1)
	{
		// free run entry (no trig for 500ms)
		if (systimer->gettime() - pulse_start > 500000)
		{
			capture_start();
			systimer->delayus(5000);
			capture_end();
			
			pulse_length = 0;

			led.write(0);
		}
		
		float v = lpf_p * 3.3f / 4095;
		if (!isnan(lpf_p))
		{
			char tmp[50] = "                                   ";
			//oled.show_str(0,0,tmp);
			//sprintf(tmp, "%dus, %.1fdbm(5G)  ", pulse_length, dbm5g);
			//oled.show_str(0,0,tmp);
			sprintf(tmp, "%.3fV               ", v);
			//oled.clear();
			oled.show_str(0,0,tmp);
			sprintf(tmp, "%.3fGhz    ", input_freq*1e-3f);
			//if (input_freq > 2499)
			oled.show_str(0, 1, tmp);
			
			sprintf(tmp, "%d us     ", capture_length);
			oled.show_str(0, 2, tmp);
			
		}
	}
	
	return 0;
}
