#include "board.h"

#include <HAL/STM32F1/F1SPI.h>
#include <HAL/STM32F1/F1GPIO.h>
#include <HAL/STM32F1/F1Interrupt.h>
#include <HAL/STM32F1/F1Timer.h>
#include <HAL/Interface/II2C.h>
#include <HAL/Interface/ISysTimer.h>
#include <string.h>
#include <utils/space.h>
#include <misc.h>

using namespace STM32F1;
using namespace HAL;

F1GPIO qon(GPIOB, GPIO_Pin_3);
F1Timer button_timer(TIM4);
int16_t adc_data[8] = {0};
bool show_charging = false;
bool flash_battery_led = true;
bool long_press_ing = false;
bool calling = false;
F1GPIO vib(GPIOB, GPIO_Pin_10);
F1GPIO beep_en(GPIOC, GPIO_Pin_15);

F1GPIO _cs(GPIOB, GPIO_Pin_12);
F1GPIO _ce(GPIOB, GPIO_Pin_11);
F1GPIO _irq(GPIOC, GPIO_Pin_6);
F1GPIO _dbg(GPIOB, GPIO_Pin_4);		// dummy debug output
F1GPIO _dbg2(GPIOB, GPIO_Pin_6);	// dummy debug output
F1GPIO _SCL(GPIOC, GPIO_Pin_13);
F1GPIO _SDA(GPIOC, GPIO_Pin_14);

F1SPI _spi;
F1Interrupt _interrupt;
F1Timer _timer(TIM2);

F1GPIO power_leds[4] =
{
	F1GPIO(GPIOC, GPIO_Pin_3),
	F1GPIO(GPIOC, GPIO_Pin_2),
	F1GPIO(GPIOC, GPIO_Pin_1),
	F1GPIO(GPIOC, GPIO_Pin_0),
};

F1GPIO state_led[3] =
{
	F1GPIO(GPIOB, GPIO_Pin_7),
	F1GPIO(GPIOB, GPIO_Pin_8),
	F1GPIO(GPIOB, GPIO_Pin_9),
};

F1GPIO keys[8] =
{
	F1GPIO(GPIOC, GPIO_Pin_7),
	F1GPIO(GPIOC, GPIO_Pin_8),
	F1GPIO(GPIOC, GPIO_Pin_9),
	F1GPIO(GPIOC, GPIO_Pin_10),
	F1GPIO(GPIOC, GPIO_Pin_11),
	F1GPIO(GPIOC, GPIO_Pin_9),
	F1GPIO(GPIOB, GPIO_Pin_0),
	F1GPIO(GPIOB, GPIO_Pin_2),
};

F1GPIO ants[3] =
{
	F1GPIO(GPIOC, GPIO_Pin_7),
	F1GPIO(GPIOC, GPIO_Pin_8),
	F1GPIO(GPIOC, GPIO_Pin_9),
};

int iabs(int a)
{
	return a>0 ? a : -a;
}

static void adc_config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	// Configure GPIO0~2 as analog input
	for(int ADC_Channel=0; ADC_Channel<7; ADC_Channel++)
	{
		GPIO_InitStructure.GPIO_Pin = (1 << (ADC_Channel%8));
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(ADC_Channel>8?GPIOB:GPIOA, &GPIO_InitStructure);
	}

	
	DMA_InitTypeDef DMA_InitStructure;
	
	/* DMA channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adc_data;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 8;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	/* ADC1 configuration */		
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 8;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_239Cycles5);
	
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
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
	long_press_ing = false;
	calling = false;
}

uint8_t v;
void shutdown()
{
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

void on_long_press(int elapsed)
{
	if (systimer->gettime() - last_click < 1200000)
	{
		if (!calling)
			long_press_ing = true;
		if (elapsed > 750000 && !calling)
		{
			calling = true;
			long_press_ing = false;
			on_click_and_press();
		}
		
		if (!calling)
		{			
			int n = (elapsed-250000) * 5 / 500000;
			if (powerup)
				n = 4 - n;
			for(int i=0; i<4; i++)
				power_leds[i].write(i>n);
		}
	}
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

int16_t mode = 0;
void mode_button_entry(void *parameter, int flags)
{
	mode = 4095 - mode;
	if (powerup)
	{
		state_led[0].write(!mode);
		state_led[1].write(false);
	}
	else
	{
		state_led[0].write(true);
		state_led[1].write(true);
		state_led[2].write(true);
	}
}


int64_t last_charging = -9999999;
uint8_t reg8;
bool charge_done;
void read_charge_state()
{
	i2c.read_reg(0x6b<<1, 8, &reg8);
	bool power_good = reg8 & (0x4);
	bool charging = ((reg8>>4)&0x03) == 0x01 || ((reg8>>4)&0x03) == 0x02;
	charge_done = (reg8>>4)&0x03 == 0x03;
	if (charging)
		last_charging = systimer->gettime();
	
	show_charging = systimer->gettime() < last_charging + 500000;
}

void button_timer_entry(void *p)
{
	static int64_t up = systimer->gettime();
	if (!qon.read())
	{
		up = systimer->gettime();
	}
	if (systimer->gettime() > up + 250000)
	{
		on_long_press(systimer->gettime() - up);
	}
	
	if (systimer->gettime() > last_key_down + 150000 && qon.read())
	{
		vib.write(false);
	}	
	
	// power LED: 
	// note: no division used for M0's sake
	// <3.30V: 0 led, adc
	// 3.300V - 3.525V: 1 led, adc > 2048
	// 3.525V - 3.750V : 2 leds, adc > 2188
	// 3.750V - 3.965V : 3 leds, adc > 2327
	// 3.975V - 4.20V : 4 leds, adc > 2467
	int num_led = 0;
	int16_t adc = adc_data[6];
		
	static int16_t last_used_adc = 0;
	if (iabs(last_used_adc - adc) < 30)		// a little hysteresis
		adc = last_used_adc;
	last_used_adc = adc;
	if (adc > 2048)
		num_led = 1;
	if (adc > 2188)
		num_led = 2;
	if (adc > 2327)
		num_led = 3;
	if (adc > 2467)
		num_led = 4;
	
	flash_battery_led = systimer->gettime() < last_click + 1000000;
	
	if (long_press_ing)
	{
		// do nothing, let on_long_press() control leds
	}
	
	else if (show_charging)
	{
		int loop_time = (num_led+1)* 200000;
		int c = (systimer->gettime() % loop_time) * (num_led+1) / loop_time;
		if (num_led > c)
			num_led = c;
		for(int i=1; i<=4; i++)
		{
			power_leds[i-1].write(i>num_led);
		}
	}
	
	else
	{
		for(int i=1; i<=4; i++)
		{
			if (flash_battery_led && systimer->gettime() %500000 < 250000)
				power_leds[i-1].write(true);
			else
				power_leds[i-1].write(i>num_led);
		}
	}
	

}

void update_config()
{
	configure_entry config[6];


	if (space_read("conf", 4, &config, sizeof(config), NULL) < 0)
	{
		// default configuration
		for(int i=0; i<sizeof(config)/sizeof(config[0]); i++)
		{
			config[i]._min = 0;
			config[i]._max = 4095;
			config[i].middle = 2048;
			config[i].reverse = 0;
			config[i].dead_band = 0;
		}
				
		config[0].reverse = true;
		config[1].reverse = true;
		config[3].reverse = false;
		
		space_write("conf", 4, &config, sizeof(config), NULL);		
	}
	else if (!config[0].reverse)
	{
		config[0].reverse = true;
		config[1].reverse = true;
		config[3].reverse = false;
		
		space_write("conf", 4, &config, sizeof(config), NULL);		
	}
}

F1Interrupt button_int;
F1Interrupt mode_button_int;
uint8_t reg08;
uint8_t reg[10];


int board_init()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	update_config();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	watchdog_init();
	
	
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
	for(int i=0; i<4; i++)
	{
		power_leds[i].write(true);
		power_leds[i].set_mode(MODE_OUT_OpenDrain);
	}
	for(int i=0; i<sizeof(ants)/sizeof(ants[0]); i++)
	{
		ants[i].set_mode(MODE_OUT_PushPull);
		ants[i].write(false);
	}
	for(int i=0; i<3; i++)
	{
		state_led[i].write(true);
		state_led[i].set_mode(MODE_OUT_OpenDrain);
	}
	for(int i=0; i<sizeof(keys)/sizeof(keys[0]); i++)
		keys[i].set_mode(MODE_IN);
	
	
	vib.write(false);
	vib.set_mode(MODE_OUT_PushPull);
	adc_config();
	button_int.init(GPIOB, GPIO_Pin_3, interrupt_rising_or_falling);
	button_entry(NULL, 0);
	button_int.set_callback(button_entry, NULL);
	button_timer.set_period(10000);
	button_timer.set_callback(button_timer_entry, NULL);
	
	mode_button_int.init(GPIOB, GPIO_Pin_1, interrupt_rising);
	mode_button_int.set_callback(mode_button_entry, NULL);
	
	int64_t up = systimer->gettime();
	qon.set_mode(MODE_IN);
	::dbg->write(false);
	::dbg2->write(false);
	::dbg->set_mode(MODE_OUT_OpenDrain);
	::dbg2->set_mode(MODE_OUT_OpenDrain);
	::vibrator = &vib;
	
	i2c.init(::SCL, ::SDA);
	i2c.set_speed(25);
	int64_t bq_timeout = systimer->gettime() + 500000;
	while(systimer->gettime() < bq_timeout)
	{
		if (i2c.read_reg(0x6b<<1, 8, &reg08) == 0)
			break;
	}
	systimer->delayms(600);
	i2c.write_reg(0x6b<<1, 1, 0x80);		// reset registers
	systimer->delayms(100);
	i2c.write_reg(0x6b<<1, 5, 0x8C);		// disable i2c watchdog
	i2c.write_reg(0x6b<<1, 3, 0x10);		// 256mA pre-charge, 128mA termination
	i2c.write_reg(0x6b<<1, 2, 0x20);		// 1A charge
	i2c.write_reg(0x6b<<1, 0, 0x07);		// allow charger to pull vbus down to 3.88V, 3A max input
	i2c.write_reg(0x6b<<1, 1, 0x1F);		// 3.7V minimum system voltage, charge enable
	i2c.write_reg(0x6b<<1, 7, 0x4B);		// default value
	
	for(int i=0; i<10; i++)
	{
		if (0 == i2c.read_reg(0x6b<<1, i, reg+i))
			i++;
		else
		{
			i2c.reset_bus();
			
			if (i == 9)
				NVIC_SystemReset();
		}
		
		::dbg->toggle();
	}
	
	while(!powerup)
	{
		read_charge_state();

		if (show_charging || systimer->gettime() < last_click + 500000)
		{
			// charging or click not timed out, no shutting down.
			::dbg->write(true);
			::dbg2->write(systimer->gettime() % 200000 < 100000);
		}
		else
		{
			::dbg2->write(true);
			::dbg->write(systimer->gettime() % 200000 < 100000);
			if (!charge_done && systimer->gettime() > last_click + 5000000 && systimer->gettime() > last_charging + 5000000)
				shutdown();
		}
		
		watchdog_reset();
	}
	state_led[0].write(!mode);
	state_led[1].write(false);

	::dbg->write(true);
	::dbg2->write(true);

	return 0;
}

void read_channels(int16_t *channel, int max_channel_count)
{
	// channel map:
	// PA3		roll
	// PA2		pitch
	// PA1		throttle
	// PA0		rudder
	// PA5		gimbal
	
	channel[0] = adc_data[3];
	channel[1] = adc_data[2];
	channel[2] = adc_data[1];
	channel[3] = adc_data[0];
	channel[4] = adc_data[5];
	channel[5] = 2048;
}

extern int apply_channel_statics();
uint8_t gkey;
bool applied = false;
int64_t apply_timer = 0;
void read_keys(uint8_t* key, int max_keys)
{
	*key = 0;
	for(int i=0; i<8; i++)
		*key |= (keys[i].read() ? 1 : 0) << i;
	gkey = *key;

	if ((gkey & 0x1f) == 0)
		apply_timer = (apply_timer == 0) ? systimer->gettime() : apply_timer;
	else
		apply_timer = 0;
	
	if ( (apply_timer != 0) && (systimer->gettime() - apply_timer > 3000000) && !applied)
	{
		apply_channel_statics();
		applied = true;
		vib.write(1);
		systimer->delayms(500);
		vib.write(0);
	}
}


extern "C" void TIM2_IRQHandler(void)
{
	_timer.call_callback();
}
extern "C" void TIM4_IRQHandler(void)
{
	button_timer.call_callback();
}