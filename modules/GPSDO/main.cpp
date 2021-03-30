#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4UART.h>
#include <HAL/STM32F4/F4SysTimer.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4HSBulk.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4Timer.h>
#include <HAL/STM32F4/F4ADC.h>
#include <HAL/Interface/II2C.h>
#include <HAL/aux_devices/OLED_I2C.h>
#include <Protocol/common.h>

#include <HAL/sensors/UartUbloxNMEAGPS.h>

#include <HAL/aux_devices/ADF4001.h>

#include <stm32f4xx_tim.h>
using namespace STM32F4;
using namespace HAL;

F4GPIO LD(GPIOB, GPIO_Pin_5);
F4GPIO led_main_out(GPIOB, GPIO_Pin_14);
F4GPIO led_div_out(GPIOB, GPIO_Pin_15);

F4GPIO cs_div(GPIOB, GPIO_Pin_12);
F4GPIO cs_main(GPIOB, GPIO_Pin_6);
F4GPIO cs_cpld(GPIOB, GPIO_Pin_0);


F4Interrupt trig;
F4Timer timer(TIM4);
F4UART uart1(USART1);
F4SPI spi(SPI1);
sensors::UartUbloxBinaryGPS gps;


extern "C" void TIM4_IRQHandler(void)
{
	timer.call_callback();
}

void timer_cb(void *p)
{
}

void trig_cb(void *p, int flags)
{
}

int pwm_init()
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	// open B1(TIM4_CH3) as output
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);
	
	// Time base configuration
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = 4094;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_Cmd(TIM4, ENABLE);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	
	TIM_SetCompare1(TIM4, 2048);
}

int pwm_set(int16_t value)
{
	TIM_SetCompare4(TIM4, value);

	return 0;
}


extern "C" void TIM5_IRQHandler(void)
{
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

float ppm;

int main()
{
	led_main_out.set_mode(HAL::MODE_OUT_PushPull);
	led_div_out.set_mode(HAL::MODE_OUT_PushPull);

	gps.init(&uart1, 115200);
	gps.ioctl(sensors::IOCTL_SAT_MINIMUM, NULL);
	pwm_init();
	F4ADC a(ADC1, ADC_Channel_3);
	a.read();

	spi.set_mode(0, 0);
	spi.set_speed(20000000);
	cs_div.set_mode(HAL::MODE_OUT_PushPull);
	cs_div.write(1);
	cs_main.set_mode(HAL::MODE_OUT_PushPull);
	cs_main.write(1);

	LD.set_mode(HAL::MODE_IN);

	ADF4001_REG0 reg0 = {0};
	ADF4001_REG1 reg1 = {0};
	ADF4001_REG2 reg2 = {0};

	do
	{
		reg0.ADDR = 0;
		reg0.anti_backlash = 0;
		reg0.lock_detect_precision = 1;
		reg0.test_mode = 0;
		reg0.R = 5;

		reg1.ADDR = 1;
		reg1.reserved = 0;
		reg1.N = 2;
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

		write_4001_reg(cs_div, *(uint32_t*)&reg0);
		write_4001_reg(cs_div, *(uint32_t*)&reg1);		
		write_4001_reg(cs_div, *(uint32_t*)&reg2);
	} while(0);

	do
	{
		reg0.ADDR = 0;
		reg0.anti_backlash = 0;
		reg0.lock_detect_precision = 1;
		reg0.test_mode = 0;
		reg0.R = 20;							// ref divider: 8Mhz / 20 = 400khz

		reg1.ADDR = 1;
		reg1.reserved = 0;
		reg1.N = 100;							// RF divider: 40Mhz / 100 = 400khz
		reg1.cp_gain = 3;

		reg2.ADDR = 2;
		reg2.counter_reset = 0;
		reg2.power_down1 = reg2.power_down2 = 0;
		reg2.mux_out = 1;	// digital lock detect
		reg2.polarity = 1;
		reg2.cp_3state = 0;
		reg2.fast_lock_en = 0;
		reg2.fast_lock_mode = 0;
		reg2.timer_control = 0;
		reg2.cp_current1 = reg2.cp_current2 = 7;		// 5mA

		write_4001_reg(cs_main, *(uint32_t*)&reg0);
		write_4001_reg(cs_main, *(uint32_t*)&reg1);		
		write_4001_reg(cs_main, *(uint32_t*)&reg2);
	} while(0);
	
	
	cs_cpld.set_mode(HAL::MODE_OUT_PushPull);
	cs_cpld.write(1);
	spi.set_mode(0,1);


	float adc_lpf = NAN;
	float vset = 2.05;
	const float RC2 = 1.0f/(2*3.1415926 * 3.5f);

	float I = 0;
	float KI = 1.0f;
	pwm_set(vset/3.3*4096);
	systimer->delayms(1);
	
	while(1)
	{
			cs_cpld.write(0);
			cs_cpld.write(1);
			cs_cpld.write(0);
			uint32_t data = 0;
			uint8_t *p = (uint8_t*)&data;
			p[3] = spi.txrx(0);
			p[2] = spi.txrx(0);
			p[1] = spi.txrx(0);
			p[0] = spi.txrx(0);
			cs_cpld.write(1);
			
			data++;

			printf("cpld=%d\n", data);
			ppm = (data-50e6)/50e6;
		
		systimer->delayms(100);
	}

	int64_t lastt = systimer->gettime();
	while(1)
	{
		int64_t t = systimer->gettime();
		float dt = (t-lastt)*1e-6;
		float alpha = dt / (dt + RC2);
		lastt = t;
		int16_t v = a.read();
		v = a.read();
		adc_lpf = isnan(adc_lpf)? v:(adc_lpf * (1-alpha) + alpha * v);

		float p = v - vset;
		I += p * dt;

		//pwm_set(vset-I*KI);

		printf("\rv=%.1f, I=%.1f  ", adc_lpf, I*KI);

		devices::gps_data data;
		if (gps.read(&data) == 0)
			led_main_out.toggle();
			
		led_div_out.write(data.fix == 3);
		
		
	}

}
