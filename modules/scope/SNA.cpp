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
#include <HAL/STM32F4/F4UART2.h>
#include <HAL/aux_devices/LMX2572.h>
#include <HAL/Interface/II2C.h>
#include <HAL/aux_devices/OLED_I2C.h>
#include <HAL/aux_devices/MAX2871.h>
#include "adc.h"

using namespace STM32F4;
//F4VCP vcp;
F4UART2 vcp(UART4);

uint64_t freq_start = 60;
uint64_t freq_end = 6000;
uint64_t freq_step = 100e6;
uint64_t freq_last = freq_end;

dma_adc adc;
bool enable = false;

int parse_cmd()
{
	int c = 0;
	char tmp[200];
	do
	{
		c = vcp.readline(tmp, sizeof(tmp));

		if (c>0)
		{
			double start, end, step;
			if (sscanf(tmp, "freq,%lf,%lf,%lf", &start, &end, &step) == 3)
			{
				freq_start = start * 1e6;
				freq_end = end * 1e6;
				freq_step = step * 1e6;
			}
			
			enable = true;
		}
	}
	while (c>0);
	return 0;
}

int main()
{
	F4GPIO cs(GPIOB, GPIO_Pin_12);
	F4GPIO rf_en(GPIOC, GPIO_Pin_5);
	F4GPIO ld(GPIOC, GPIO_Pin_5);
	F4SPI spi(SPI2);

	MAX2871 max2871;
	max2871.init(&spi, &cs, &rf_en, &ld);
	max2871.set_ref(26000000, false, false, 1);
	max2871.set_freq(4900000000);
	max2871.set_output(true, true, false);


	while(1)
	{
		if (enable)
		{
		for(float f = freq_start; f <= freq_end; f+= freq_step)
		{
			if (f == freq_last)
				continue;

			freq_last = f;

			max2871.set_freq(f);

			while(!max2871.is_locked())
				;

			adc.init(GPIOB, 8);
			adc.begin();
			systimer->delayus(500);
			adc.stop();
			
			int avg = 0;
			int count = adc.full() ? max_adc_data_count : adc.pos();
			for(int i=0; i<count; i++)
				avg += adc.adc_data[i];
			float avgf = (float)avg/count * 3300 / 4095;
			
			
			char tmp[50];
			sprintf(tmp, "%.6lf\t%.2f\n", f*1e-6, avgf);
			vcp.write(tmp, strlen(tmp));
		}
		vcp.write("0\n", 2);
		}

		parse_cmd();
		
	}
}
