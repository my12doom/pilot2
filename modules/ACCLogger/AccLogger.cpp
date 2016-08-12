#include <HAL/STM32F4/F4UART.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/Interface/ISysTimer.h>
#include <HAL/Interface/II2C.h>
#include <utils/param.h>
#include <stdio.h>
#include <Protocol/crc32.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "ads1256.h"
#include <utils/log.h>

// BSP
using namespace STM32F4;
using namespace HAL;

static void swap(void *buf, int size)
{
	char *p = (char*)buf;
	int i;
	for(i=0; i<size/2; i++)
	{
		char t = p[i];
		p[i] = p[size-1-i];
		p[size-1-i] = t;
	}
}

void cb_entry(void *parameter, int flags)
{		
	ads1256_begin();
	ads1256_tx_rx(CMD_ReadData);
	systimer->delayus(5);
	
	uint8_t data[3];
	data[0] = ads1256_tx_rx(0xff);
	data[1] = ads1256_tx_rx(0xff);
	data[2] = ads1256_tx_rx(0xff);
	
	ads1256_end();
	
	int16_t v;
	((uint8_t*)&v)[0] = data[1];
	((uint8_t*)&v)[1] = data[0];
	
	//printf("%.5f,%d\n", systimer->gettime()/1000000.0f, v);
	//char tmp[200];
	//sprintf(tmp, "%.5f,%d\r\n", systimer->gettime()/1000000.0f, v);
	log_write(&v, 2);
}

int main()
{
	F4GPIO led1(GPIOA, GPIO_Pin_2);
	F4GPIO led2(GPIOA, GPIO_Pin_3);
	F4GPIO DRDY(GPIOA, GPIO_Pin_8);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	led1.set_mode(MODE_OUT_PushPull);
	led1.write(true);
	led2.set_mode(MODE_OUT_PushPull);
	led2.write(true);
	DRDY.set_mode(MODE_IN);
	
	F4GPIO scl(GPIOB, GPIO_Pin_6);
	F4GPIO sda(GPIOB, GPIO_Pin_7);
	I2C_SW i2c(&scl, &sda);
	i2c.set_speed(125);

	
	log_init();
	systimer->delayms(1000);
	
	for(int i=0; i<3; i++)
		ads1256_init();
		
	bool ok = true; 
	for(int i=0; i<3; i++)
	{		
		// configure buffer and calibration
		ads1256_status adc_status;
		ads1256_read_registers(REG_Status, 1, &adc_status);
		adc_status.AutoCalibration = 1;
		adc_status.BufferEnable = 0;
		ads1256_write_registers(REG_Status, 1, &adc_status);
		
		ok = ok && (adc_status.ID == 3);
		
		// input: single ended AIN3
		ads1256_mux adc_mux;
		ads1256_read_registers(REG_MUX, 1, &adc_mux);
		adc_mux.positive = ads1256_channnel_AIN2;
		adc_mux.negative = ads1256_channnel_AIN6;
		ads1256_write_registers(REG_MUX, 1, &adc_mux);

		// data rate
		ads1256_speed sps = ads1256_speed_15000sps;
		ads1256_write_registers(REG_DataRate, 1, &sps);

	}

	F4Interrupt interrupt;
	interrupt.init(GPIOA, GPIO_Pin_8, interrupt_falling);
	interrupt.set_callback(cb_entry, NULL);
	
	ads1256_go();
	while(1)
	{
		log_flush();
		
		int t = systimer->gettime() % 400000;
		
		IGPIO * p = (storage_ready && ok) ? &led1 : &led2;
		p->write(t<200000);
		
		if (!storage_ready)
			led1.write(true);
	}

	while(1)
	{
		IGPIO * p = (storage_ready && ok) ? &led1 : &led2;
		p->write(true);
		systimer->delayms(200);
		p->write(false);
		systimer->delayms(200);
	}
}


struct __FILE { int handle;  };
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

extern "C" int fputc(int ch, FILE *f)
{
	if (DEMCR & TRCENA) 
	{
		while (ITM_Port32(0) == 0);
		ITM_Port8(0) = ch;
	}
	return (ch);
}