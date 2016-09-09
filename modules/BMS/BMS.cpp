#include <HAL/STM32F4/F4UART.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/Interface/ISysTimer.h>
#include <HAL/Interface/II2C.h>
#include <utils/param.h>
#include <utils/log.h>
#include <stdio.h>
#include <Protocol/crc32.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <Algorithm/battery_estimator.h>

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

typedef struct cell_str
{
	float voltage;
	uint16_t id;
} _cell;

int compare_uint16 (const void * a, const void * b)
{
  if ( (*(_cell*)a).voltage <  (*(_cell*)b).voltage ) return -1;
  if ( (*(_cell*)a).voltage == (*(_cell*)b).voltage ) return 0;
  if ( (*(_cell*)a).voltage >  (*(_cell*)b).voltage ) return 1;
	
  return 0;
}

uint16_t gain = 0;			// uV / LSB
int8_t offset = 0;			// mV
uint8_t v = 0;
uint8_t uv = 0;
uint8_t ov = 0;
uint8_t state = 0;

enum batt_state
{
	initializing,
	charging,
	discharging,
	idle_balancing,
	idle,
};

const char* batt_state_str[] = 
{
	"initializing",
	"charging",
	"discharging",
	"idle_balancing",
	"idle",
};

int64_t idle_start_time = 0;

batt_state bstate = initializing;

float lsb2voltage(uint16_t lsb)
{
	float voltage = lsb * gain * 1e-6;
	voltage += offset * 1e-3;
	
	return voltage;
}

float lsb2packvoltage(uint16_t lsb, int pack_count)
{
	float voltage = 4 * lsb * gain * 1e-6;
	voltage += offset * 1e-3 * pack_count;
	
	return voltage;
}

int main()
{
	F4GPIO led1(GPIOA, GPIO_Pin_2);
	F4GPIO led2(GPIOA, GPIO_Pin_3);
	F4GPIO led3(GPIOA, GPIO_Pin_5);
	F4GPIO led4(GPIOA, GPIO_Pin_6);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	log_init();
	led1.set_mode(MODE_OUT_PushPull);
	led1.write(true);
	led2.set_mode(MODE_OUT_PushPull);
	led2.write(true);
	led3.set_mode(MODE_OUT_PushPull);
	led3.write(true);
	led4.set_mode(MODE_OUT_PushPull);
	led4.write(true);
	
	F4GPIO scl(GPIOB, GPIO_Pin_6);
	F4GPIO sda(GPIOB, GPIO_Pin_7);
	I2C_SW i2c(&scl, &sda);
	i2c.set_speed(125);


	// read gain and offset
	i2c.read_reg(0x30, 0x50, &v);
	gain = (v & 0x0C) << 1;
	i2c.read_reg(0x30, 0x59, &v);
	gain |= v >> 5;
	gain += 365;
	i2c.read_reg(0x30, 0x51, (uint8_t*)&offset);
	i2c.read_reg(0x30, 0x9, (uint8_t*)&ov);
	ov = 185;
	i2c.write_reg(0x30, 0x9, ov);
	i2c.read_reg(0x30, 0xA, (uint8_t*)&uv);
	uv = 255;
	i2c.write_reg(0x30, 0xA, uv);
	
	for(int i=0; i<0x69; i++)
	{
		v = 0;
		i2c.read_reg(0x30, i, &v);
		printf("reg(%02x) = %02x\n", i, v);
	}
		
	i2c.write_reg(0x30, 0, 0xf);				// clear OCD/SCD/UV/OV
	i2c.write_reg(0x30, 5, 0x43);				// enable DSG/CHG/CC
	i2c.write_reg(0x30, 6, 0 | (3 << 4));		// SCD : 70us, 56mV
	//i2c.write_reg(0x30, 7, 6 | (4 << 4));		// OCD : 160ms, 19mV
	i2c.write_reg(0x30, 7, 6 | (7 << 4));		// OCD : 160ms, 28mV
	i2c.write_reg(0x30, 0xB, 0x19);				// CC:0x19, wtf?!
		
	int64_t last_t = systimer->gettime();
	int64_t charging_time = 0;
	int balancing_cell = -1;
	
	log_printf("t,v1,v2,v3,e1,e2,e3,I\r\n");
	float cell_current[5] = {0};
	battery_estimator cell_estimator[5]; 
	
	while(1)
	{
		int64_t t = systimer->gettime();
		int64_t dt = t - last_t;
		float dts = dt/1000000.0f;
		last_t = t;
		
		i2c.read_reg(0x30, 5, &v);
		i2c.read_reg(0x30, 0, &state);
		
		led1.write(!(v&1));			// CHG
		led2.write(!(v&2));			// DSG
		

		i2c.read_reg(0x30, 0, &v);
		led3.write(!(v&1));			// OCD
		//led4.write(!(v&2));			// SCD
		led4.write(!(storage_ready));
		
		// bat voltage
		uint16_t vbat = 0;
		i2c.read_regs(0x30, 0x2A, (uint8_t*)&vbat, 2);
		swap(&vbat, 2);
		
		// cell voltage
		uint16_t cell[5] = {0};
		for(int i=0; i<5; i++)
		{
			i2c.read_regs(0x30, 0x0C+2*i, (uint8_t*)&cell[i], 2);
			swap(&cell[i], 2);
		}
		
		// current sensing
		int16_t CC = {0};
		i2c.read_regs(0x30, 0x32, (uint8_t*)&CC, 2);
		swap(&CC, 2);
		
		printf("\rvbat:%d(%.3f) (%d(%.3f)+%d(%.3f)+%d(%.3f), UV/OV:%.3f/%.3f, current:%d(%.6fV, %.3fA)), state:%2x, bstate:%s         ",
						vbat, lsb2packvoltage(vbat, 3), cell[0], lsb2voltage(cell[0]), cell[1], lsb2voltage(cell[1]), cell[4], lsb2voltage(cell[4]),
						lsb2voltage( (1<<12) |(uv<<4) ), lsb2voltage( (2<<12) |(ov<<4) ),
						CC, CC*8.44e-6, CC*8.44e-6/1e-3, state, batt_state_str[bstate] );
		
		for(int i=0; i<5; i++)
		{
			cell_current[i] = -CC*8.44e-6/1e-3;
			if (i == balancing_cell)
				cell_current[i] += lsb2voltage(cell[i]) / 20.0f;
			
			cell_estimator[i].update(lsb2voltage(cell[i]), cell_current[i], dts);
		}
		
		static float mah = 0;
		static int64_t last_t = 0;
		log_printf("%.3f, %.3f,%.3f,%.3f, %.3f,%.3f,%.3f, %.3f,%.3f,%d\r\n", t/1000000.0f,
			lsb2voltage(cell[0]), lsb2voltage(cell[1]), lsb2voltage(cell[4]),
			cell_estimator[0].get_internal_voltage(), cell_estimator[1].get_internal_voltage(), cell_estimator[4].get_internal_voltage(),
			CC*8.44e-6/1e-3, mah, balancing_cell);
		if (dts > 0.01f && dts < 1.0f)
		{
			mah += dts * CC*8.44e-6/1e-3 / 3.6f;
		}
		log_flush();
		systimer->delayms(500);
				
		// state handling
		if (CC > 10)
		{
			bstate = charging;
			
			charging_time += dt;
		}
		
		else if (CC < -10)
		{
			bstate = discharging;
			charging_time = 0;
		}
		else
		{			
			if (bstate != idle)
				idle_start_time = systimer->gettime();
			
			bstate = charging_time > 0 ? idle_balancing : idle;
			
			charging_time -= dt;
			
			
			if (bstate == idle && systimer->gettime() - idle_start_time > 10000000)
			{
				// shutdown
				printf("\nshutdown\n");
				systimer->delayms(20);
				i2c.write_reg(0x30, 4, 1);
				i2c.write_reg(0x30, 4, 2);
			}
		}
		
		bool balancing_required = false;
		balancing_required = (bstate == idle_balancing || bstate == charging);
		
		if (balancing_required)
		{
			// sort cells
			_cell cells[3] = 
			{
				{cell[0],0},
				{cell[1],1},
				{cell[4],4},
			};
			qsort(cells,  3, sizeof(_cell), compare_uint16);
			
			// should we balance? (imbalance not too much, not too small)
			float imbalance = cells[2].voltage - cells[0].voltage;			
			printf("imbalance=%.1f    ", imbalance);
			
			if (imbalance > 25 && imbalance < 1000)
			{
				static int64_t last = 0;
				if ( systimer->gettime() - last > 6000000)
				{
					last = systimer->gettime();
					i2c.write_reg(0x30, 1, 1<<cells[2].id);
					balancing_cell = cells[2].id;
				}
				else if (systimer->gettime() - last > 5000000)
				{
					i2c.write_reg(0x30, 1, 0);
					balancing_cell = -1;
				}
				else
				{
				}
			}
			else
			{
				if (bstate == idle_balancing)
					charging_time = 0;
				
				i2c.write_reg(0x30, 1, 0);
				balancing_cell = -1;
			}
		}
		else
		{
			i2c.write_reg(0x30, 1, 0);
			balancing_cell = -1;
		}
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