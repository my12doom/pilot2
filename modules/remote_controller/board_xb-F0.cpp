#include "board.h"

#include <HAL/STM32F0/F0SPI.h>
#include <HAL/STM32F0/F0GPIO.h>
#include <HAL/STM32F0/F0Interrupt.h>
#include <HAL/STM32F0/F0Timer.h>
#include <HAL/Interface/II2C.h>
#include <HAL/Interface/ISysTimer.h>
#include <string.h>
#include "DAC_F0.h"

using namespace STM32F0;
using namespace HAL;

F0GPIO qon(GPIOB, GPIO_Pin_0);
F0Timer button_timer(TIM16);
int16_t adc_data[7] = {0};

F0GPIO vib(GPIOB, GPIO_Pin_8);

F0GPIO _cs(GPIOB, GPIO_Pin_12);
F0GPIO _ce(GPIOB, GPIO_Pin_11);
F0GPIO _irq(GPIOC, GPIO_Pin_6);
F0GPIO _dbg(GPIOC, GPIO_Pin_0);
F0GPIO _dbg2(GPIOC, GPIO_Pin_1);
F0GPIO _SCL(GPIOC, GPIO_Pin_13);
F0GPIO _SDA(GPIOC, GPIO_Pin_14);

F0SPI _spi;
F0Interrupt _interrupt;
F0Timer _timer(TIM14);

F0GPIO pa6(GPIOA, GPIO_Pin_6);

static void adc_config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	// Configure GPIO0~2 as analog input
	for(int ADC_Channel=0; ADC_Channel<7; ADC_Channel++)
	{
		GPIO_InitStructure.GPIO_Pin = 1 << ADC_Channel;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}
	
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adc_data;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 7;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);		
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	// ADC1 configuration
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE ;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4); 


	ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_1, ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_2, ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_3, ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_4, ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_5, ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_6, ADC_SampleTime_239_5Cycles);
	
	ADC_OverrunModeCmd(ADC1, ENABLE);
	ADC_GetCalibrationFactor(ADC1);
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN) == RESET);
	ADC_StartOfConversion(ADC1);
}

I2C_SW i2c;

int64_t last_click = -2000000;
static int64_t last_key_down = -2000000;
void on_click(int64_t click_start, int click_long)
{
	if (click_long < 400000)
		last_click = click_start + click_long;
}

void on_key_down()
{
	vib.write(true);
}

void on_key_up()
{
	vib.write(false);
}

void shutdown()
{
	uint8_t v;
	i2c.read_reg(0x6b<<1, 5, &v);
	v &= 0xcf;
	i2c.write_reg(0x6b<<1, 5, v);
	i2c.read_reg(0x6b<<1, 7, &v);
	v |= 0x20;			
	i2c.write_reg(0x6b<<1, 7, v);	
	i2c.write_reg(0x6b<<1, 7, v);
	i2c.write_reg(0x6b<<1, 7, v);
	
	i2c.read_reg(0x6b<<1, 7, &v);

	systimer->delayms(10);
	NVIC_SystemReset();
}

bool powerup = false;

void on_click_and_press()
{	
	if (!powerup)
	{
		powerup = true;
	}
	else
	{
		shutdown();
	}
}

void on_long_press()
{
	if (systimer->gettime() - last_click < 1200000)
		on_click_and_press();
}

void button_entry(void *parameter, int flags)
{
	if (qon.read())
	{
		last_key_down = systimer->gettime();
		on_key_down();
	}
	else
	{
		if (last_key_down > 0)
		{
			on_key_up();
			on_click(last_key_down, systimer->gettime() - last_key_down);
		}
	}
}
void button_timer_entry(void *p)
{
	static int64_t up = systimer->gettime();
	static bool calling = false;
	if (!qon.read())
	{
		up = systimer->gettime();
		calling = false;
	}
	if (systimer->gettime() > up + 750000 && !calling)
	{
		calling = true;
		on_long_press();
	}
	
	if (systimer->gettime() > last_key_down + 150000 && qon.read())
	{
		vib.write(false);
	}
}

F0Interrupt button_int;
uint8_t reg08;
uint8_t reg[10];
int cat();
int board_init()
{
	// NRF GPIOs and interrupt 
	::cs = &_cs;
	::ce = &_ce;
	::irq = &_irq;
	::dbg = &_dbg;
	::dbg2 = &_dbg2;
	::SCL = &_SCL;
	::SDA = &_SDA;
	::spi = &_spi;
	::interrupt = &_interrupt;
	::timer = &_timer;
	::bind_button = &qon;
	qon.set_mode(MODE_IN);
	
	_spi.init(SPI2);
	_interrupt.init(GPIOC, GPIO_Pin_6, interrupt_falling);
	
	pa6.set_mode(MODE_IN);
			
	
	vib.write(false);
	vib.set_mode(MODE_OUT_PushPull);
	button_int.init(GPIOB, GPIO_Pin_0, interrupt_rising_or_falling);
	button_entry(NULL, 0);
	button_int.set_callback(button_entry, NULL);
	button_timer.set_period(10000);
	button_timer.set_callback(button_timer_entry, NULL);
	adc_config();
	int64_t up = systimer->gettime();
	qon.set_mode(MODE_IN);
	::dbg->write(false);
	::dbg2->write(false);
	::dbg->set_mode(MODE_OUT_OpenDrain);
	::dbg2->set_mode(MODE_OUT_OpenDrain);
	::vibrator = &vib;
	
	//systimer->delayms(600);
	i2c.init(::SCL, ::SDA);
	i2c.set_speed(25);
	/*
	i2c.read_reg(0x6b<<1, 8, &reg08);
	*/
	//i2c.write_reg(0x6b<<1, 1, 0x80);		// reset registers
	//systimer->delayms(10);
	i2c.write_reg(0x6b<<1, 5, 0x8C);		// disable i2c watchdog
	i2c.write_reg(0x6b<<1, 3, 0x10);		// 256mA pre-charge, 128mA termination
	i2c.write_reg(0x6b<<1, 2, 0xA0);		// 2.5A charge
	i2c.write_reg(0x6b<<1, 0, 0x07);		// allow charger to pull vbus down to 3.88V, 3A max input
	i2c.write_reg(0x6b<<1, 1, 0x3B);		// default value, 3.5V minimum system voltage, charge enable, boost enable.
	i2c.write_reg(0x6b<<1, 7, 0x4B);		// default value
	
	for(int i=0; i<10; )
	{
		if (0 == i2c.read_reg(0x6b<<1, i, reg+i))
			i++;
		else
		{
			::SCL->write(false);
			::SCL->set_mode(MODE_OUT_PushPull);
			systimer->delayus(10);
			::SDA->write(false);
			::SDA->set_mode(MODE_OUT_PushPull);
			systimer->delayus(10);
			::SCL->write(true);
			systimer->delayus(10);
			::SDA->write(true);
			::SCL->set_mode(MODE_OUT_OpenDrain);
			::SDA->set_mode(MODE_OUT_OpenDrain);

		}
		
		::dbg->toggle();
	}
	
	int64_t last_charging_or_click = last_click;
	while(!powerup)
	{
		uint8_t reg8;
		i2c.read_reg(0x6b<<1, 8, &reg8);
		bool power_good = reg8 & (0x4);
		bool charging = ((reg8>>4)&0x03) == 0x01 || ((reg8>>4)&0x03) == 0x02;
		if (last_click > last_charging_or_click)
			last_charging_or_click = last_click;
		
		if (charging)
			last_charging_or_click = systimer->gettime();
		if (charging || systimer->gettime() < last_charging_or_click + 500000)
		{
			// power good and charging, show battery state, no shutting down.
			::dbg->write(true);
			::dbg2->write(systimer->gettime() % 200000 < 100000);		
		}
		else 
		{
			::dbg2->write(true);
			::dbg->write(systimer->gettime() % 200000 < 100000);
			if (systimer->gettime() > last_charging_or_click + 5000000)
				shutdown();
		}		
	}
	::dbg->write(true);
	::dbg2->write(true);
	dac_config(NULL, false);
	
	return 0;	
}



void read_channels(int16_t *channel, int max_channel_count)
{
	// channel map:
	// PA0		roll
	// PA1		throttle
	// PA2		rudder
	// PA4		pitch
	// PB0		center button
	// PB1		left button
	// PB2		right button
	
	channel[0] = adc_data[0];
	channel[1] = adc_data[3];
	channel[2] = adc_data[1] - 300;
	channel[3] = adc_data[2];
	channel[4] = 0;
	channel[5] = 0;
}
