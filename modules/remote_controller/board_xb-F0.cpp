#include "board.h"

#include <HAL/STM32F0/F0SPI.h>
#include <HAL/STM32F0/F0GPIO.h>
#include <HAL/STM32F0/F0Interrupt.h>
#include <HAL/STM32F0/F0Timer.h>
#include <HAL/Interface/II2C.h>
#include <HAL/Interface/ISysTimer.h>
#include <string.h>

using namespace STM32F0;
using namespace HAL;

HAL::ISPI *spi;
HAL::IGPIO *cs;
HAL::IGPIO *ce;
HAL::IGPIO *irq;
HAL::IGPIO *dbg;
HAL::IGPIO *dbg2;
HAL::IGPIO *SCL;
HAL::IGPIO *SDA;
HAL::IInterrupt *interrupt;
HAL::ITimer *timer;

F0GPIO qon(GPIOB, GPIO_Pin_0);
F0Timer button_timer(TIM16);
int16_t adc_data[6] = {0};

F0GPIO vib(GPIOB, GPIO_Pin_8);
int dac_config();

namespace sheet1
{
	F0GPIO cs(GPIOB, GPIO_Pin_12);
	F0GPIO ce(GPIOB, GPIO_Pin_11);
	F0GPIO irq(GPIOC, GPIO_Pin_6);
	F0GPIO dbg(GPIOC, GPIO_Pin_0);
	
	F0GPIO dbg2(GPIOC, GPIO_Pin_1);
	F0GPIO SCL(GPIOC, GPIO_Pin_13);
	F0GPIO SDA(GPIOC, GPIO_Pin_14);
	
	F0SPI spi;
	F0Interrupt interrupt;
	F0Timer timer(TIM14);
	
	F0GPIO pa6(GPIOA, GPIO_Pin_6);

	
	int sheet1_init()
	{
		::cs = &cs;
		::ce = &ce;
		::irq = &irq;
		::dbg = &dbg;
		::dbg2 = &dbg2;
		::SCL = &SCL;
		::SDA = &SDA;
		::spi = &spi;
		::interrupt = &interrupt;
		::timer = &timer;
		::bind_button = &qon;
		qon.set_mode(MODE_IN);
		
		spi.init(SPI2);
		interrupt.init(GPIOC, GPIO_Pin_6, interrupt_falling);
		
		pa6.set_mode(MODE_IN);
				
		return 0;
	}

}

using namespace sheet1;

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
	sheet1_init();
	vib.write(false);
	vib.set_mode(MODE_OUT_PushPull);
	button_int.init(GPIOB, GPIO_Pin_0, interrupt_rising_or_falling);
	button_entry(NULL, 0);
	button_int.set_callback(button_entry, NULL);
	button_timer.set_period(10000);
	button_timer.set_callback(button_timer_entry, NULL);
	
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
	i2c.write_reg(0x6b<<1, 1, 0x1B);		// default value, 3.5V minimum system voltage, charge enable.
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
	dac_config();
	
	return 0;	
}



void read_channels(int16_t *channel, int max_channel_count)
{
	if (max_channel_count > sizeof(adc_data)/2)
		max_channel_count = sizeof(adc_data)/2;
	
	memcpy(channel, adc_data, max_channel_count * 2);	
}
