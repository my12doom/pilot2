#include <string.h>
#include <stdio.h>
#include <math.h>

#include <stdint.h>
#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4SysTimer.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/aux_devices/LMX2572.h>
#include <HAL/Interface/II2C.h>
#include <HAL/aux_devices/OLED_I2C.h>
#include "adc.h"

#include <Protocol/common.h>

using namespace STM32F4;
using namespace HAL;

// configuration
int64_t MAX_FREQ = 6.8e9;
int64_t MIN_FREQ = 7.5e6;
float coupling = -19;


F4SPI spi(SPI1);
F4GPIO cs_2572(GPIOA, GPIO_Pin_4);
F4GPIO att_le(GPIOA, GPIO_Pin_3);

F4GPIO swd1[3] = {F4GPIO(GPIOB, GPIO_Pin_0), F4GPIO(GPIOB, GPIO_Pin_1), F4GPIO(GPIOB, GPIO_Pin_2)};
F4GPIO swd2[3] = {F4GPIO(GPIOB, GPIO_Pin_14), F4GPIO(GPIOB, GPIO_Pin_13), F4GPIO(GPIOB, GPIO_Pin_12)};
dma_adc adc;
int freq_digits[10] = {0, 1, 0, 0};
bool freq_changed = false;
int ptr_x = 1;
int ptr_y = 0;
int attenuator = 0;
int power_set = 0;
F4Interrupt up;
F4Interrupt down;
F4Interrupt left;
F4Interrupt right;
F4Interrupt center;

enum button_name
{
	button_up,
	button_down,
	button_left,
	button_right,
	button_center,
};

void change_freq(int64_t df);
void set_att(int att);

// path: 1 - 6
int select_path(int path)
{
	if (path < 1 || path > 6)
		return -1;

	int path_tbl[7][3] = {{0,0,0}, {0,0,0}, {1,0,0}, {0,1,0}, {1,1,0}, {0,0,1}, {1,0,1}};

	for(int i=0; i<3; i++)
	{
		swd1[2-i].write(path_tbl[path][i]);
		swd2[2-i].write(path_tbl[7-path][i]);
	}

	return 0;
}

int get_path_by_freq(int64_t freq)
{
	if (freq > 3400000000)
		return 3;
	else if (freq > 1750000000)
		return 2;
	else if (freq > 900000000)
		return 1;
	else if (freq > 500000000)
		return 4;
	else if (freq > 250000000)
		return 5;
	else
		return 6;
}
int select_path_by_freq(int64_t freq)
{
	return select_path(get_path_by_freq(freq));
}

int init_path()
{
	for(int i=0; i<3; i++)
	{
		swd1[i].set_mode(MODE_OUT_PushPull);
		swd2[i].set_mode(MODE_OUT_PushPull);
	}	

	select_path(6);

	return 0;
}

void button_cmd(int button)
{
	// X pos
	int x_limit_max = ptr_y == 0 ? 9 : 2;
	int x_delta = button == button_left ? -1 : (button == button_right ? 1 : 0);
	ptr_x = limit(ptr_x + x_delta, 0, x_limit_max);

	// Y pos
	int y_delta = button == button_down ? -1 : (button == button_up ? 1 : 0);
	if (ptr_y == 0 && y_delta != 0)
	{
		change_freq(y_delta * pow(10.0, 9-ptr_x));
		freq_changed = true;
	}

	if (ptr_y == 1 && y_delta != 0)
	{
		//attenuator = limit(attenuator + y_delta * pow(10.0, 2-ptr_x), 0, 127);
		//set_att(attenuator);

		power_set = limit(power_set + y_delta * pow(10.0, 2-ptr_x), -300, 300);
	}

	if (button == button_center)
	{
		ptr_y = 1 - ptr_y;
		if (ptr_y == 0)
			ptr_x = limit(ptr_x, 0, 9);
		else
			ptr_x = limit(ptr_x, 0, 2);

	}
	if (freq_digits[ptr_x] < 0)
		freq_digits[ptr_x] = 0;
}
	
void button_cb(void *p, int flags)
{
	if (flags == interrupt_falling)
	{
		int button = (int)p;
		button_cmd(button);
	}
}



int64_t read_freq()
{
	int64_t out = 0;
	for(int i=0; i<10; i++)
		out = out * 10 + freq_digits[i];

	return out;
}

void format_freq(int64_t in)
{
	if (in > MAX_FREQ)
		in = MAX_FREQ;
	if (in < MIN_FREQ)
		in = MIN_FREQ;
	for(int i=0; i<10; i++)
	{
		freq_digits[9-i] = in%10;
		in /= 10;
	}
}

void change_freq(int64_t df)
{
	format_freq(read_freq()+df);
}

void set_att(int att)
{
	att &= 0x7f;
	uint8_t tx = 0;
	for(int i=0; i<8; i++)
		tx |= ((att>>i)&1) << (7-i);

	att = limit(att, 0, 127);
	spi.txrx(tx);
	att_le.write(1);
	systimer->delayus(1);
	att_le.write(0);
}

int main()
{
	init_path();

	LMX2572 lmx2572;
	lmx2572.init(&spi, &cs_2572);
	lmx2572.set_ref(40000000, true, 1, 1, 1);
	lmx2572.set_freq(read_freq());
	select_path_by_freq(read_freq());
	

	F4GPIO lock_led(GPIOB, GPIO_Pin_8);
	lock_led.set_mode(HAL::MODE_OUT_PushPull);
	lock_led.write(0);

	att_le.write(0);
	att_le.set_mode(MODE_OUT_PushPull);
	set_att(attenuator);
		
	F4GPIO scl(GPIOA, GPIO_Pin_10);
	F4GPIO sda(GPIOA, GPIO_Pin_9);
	I2C_SW i2c(&scl, &sda);
	i2c.set_speed(1);
	devices::OLED96 oled;
	oled.init(&i2c, 0x78);
	oled.show_str(0, 0, "HelloWorld");	
	oled.clear();

	adc.init(GPIOA, 1);
	adc.begin();

	left.init(GPIOB, GPIO_Pin_6, interrupt_falling);
	right.init(GPIOB, GPIO_Pin_4, interrupt_falling);
	up.init(GPIOB, GPIO_Pin_7, interrupt_falling);
	down.init(GPIOB, GPIO_Pin_3, interrupt_falling);
	center.init(GPIOB, GPIO_Pin_5, interrupt_falling);

	left.set_callback(button_cb, (void*)button_left);
	right.set_callback(button_cb, (void*)button_right);
	up.set_callback(button_cb, (void*)button_up);
	down.set_callback(button_cb, (void*)button_down);
	center.set_callback(button_cb, (void*)button_center);

	// long press?	
	F4GPIO but_up(GPIOB, GPIO_Pin_7);
	F4GPIO but_down(GPIOB, GPIO_Pin_3);
	int64_t last_up = -1;
	int64_t last_down = -1;
	int last_up_count = 0;
	int last_down_count = 0;
	float vdet = 1.0f;
	float vset = 0.8f;
	float dbm_set = 0;
	float vset_per_lsb = 0.022f / 4;
	float vset_deadband = 0;//vset_per_lsb * 1 / 2;
	double ghz = read_freq() * 1e-9;
	float intercept_mv = 2.289 * ghz*ghz*ghz*ghz -22.99 * ghz*ghz*ghz +66.56 * ghz*ghz -89.60 * ghz + 693.7;
	float intercept = intercept_mv / 22.0f;
	char tmp[20];

	while(1)
	{
		// lock indicator
		lock_led.write(lmx2572.is_locked());

		// VDET & attenuator, unlevel warning
		if (adc.full())
		{
			adc.stop();
			int avg = 0;
			for(int i=0; i<max_adc_data_count; i++)
				avg += adc.adc_data[i];
			vdet = (float)avg/max_adc_data_count * 3.3f / 4096;

			char tmp[50];
			sprintf(tmp, "%.3fV, %.2fdb", vdet, attenuator * 0.25f);
			oled.show_str(0, 3, tmp, false);

			if (attenuator == 0 || attenuator == 127)
				oled.show_str(0, 4, "unlevel", false);
			else
				oled.show_str(0, 4, "        ", false);


			adc.begin();
		}

		if (1)
		{
			dbm_set = power_set / 10.0f;
			vset = (intercept - (dbm_set + coupling)) * 0.022f;

			if (fabs(vset - vdet) > vset_deadband)		// deadband : 2 * step
			{
				int delta_lsb = (vset - vdet) / vset_per_lsb  * 0.9f;
				attenuator = limit(attenuator + delta_lsb, 0, 127);

				set_att(attenuator);
			}
		}

		// freq UI
		for(int i=0,j=0; i<10; i++,j++)
		{
			if ((i-1)%3 == 0 && i != 0)
			{
				oled.show_str(j*6, 1, ",");
				j++;
			}

			char tmp[20];
			sprintf(tmp, "%d", freq_digits[i]);
			oled.show_str(j*6, 1, tmp, i == ptr_x && ptr_y == 0);
		}
		oled.show_str(13*6, 1, "hz");
		sprintf(tmp, "band %d", get_path_by_freq(read_freq()));
		oled.show_str(0, 2,  tmp);
		
		if (freq_changed)
		{
			ghz = read_freq() * 1e-9;
			float intercept_ref = -0.071 * ghz*ghz*ghz + 1.43 * ghz*ghz + -6.9 * ghz + 20.5;
			float intercept_mv = 2.289 * ghz*ghz*ghz*ghz -22.99 * ghz*ghz*ghz +66.56 * ghz*ghz -89.60 * ghz + 693.7;
			intercept = intercept_mv / 22.0f + coupling;
			
			freq_changed = false;
			int64_t t = systimer->gettime();
			lmx2572.set_freq(read_freq());
			select_path_by_freq(read_freq());
			
			int64_t timeout = t + 100000;
			while(!lmx2572.is_locked())
			{
				lock_led.write(0);
				if (systimer->gettime() > timeout)
					break;
			}
			printf("change freq took %lld us\n", systimer->gettime() - t);
		}


		// power UI
		int abs_power = abs((float)power_set);
		sprintf(tmp, "%d", abs_power/100);
		oled.show_str(0, 0, power_set >= 0 ? "+" : "-");
		oled.show_str(6, 0, tmp, ptr_y == 1 && ptr_x == 0);
		sprintf(tmp, "%d", (abs_power/10)%10);
		oled.show_str(12, 0, tmp, ptr_y == 1 && ptr_x == 1);
		oled.show_str(18, 0, ".");
		sprintf(tmp, "%ddbm", abs_power%10);
		oled.show_str(24, 0, tmp, ptr_y == 1 && ptr_x == 2);

		// long press buttons
		if (but_up.read())
		{
			last_up = last_up == -1 ? systimer->gettime() : last_up;
			int dt = systimer->gettime() - last_up;
			int count = dt > 400000 ? (dt-400000)/30000 : 0;

			if (count > last_up_count)
			{
				last_up_count = count;
				button_cmd(button_up);
			}
		}
		else
		{
			last_up = -1;
			last_up_count = 0;
		}

		if (but_down.read())
		{
			last_down = last_down == -1 ? systimer->gettime() : last_down;
			int dt = systimer->gettime() - last_down;
			int count = dt > 400000 ? (dt-400000)/30000 : 0;

			if (count > last_down_count)
			{
				last_down_count = count;
				button_cmd(button_down);
			}
		}
		else
		{
			last_down = -1;
			last_down_count = 0;
		}

	}
}
