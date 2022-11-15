
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4UART.h>
#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4Timer.h>
#include <HAL/STM32F4/F4ADC.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/sensors/MPU6000.h>
#include <HAL/sensors/ICM40608.h>
#include <HAL/sensors/ADXL357.h>
#include <HAL/sensors/bmi160.h>
#include <modules/cal/app_cali_imu.hpp>

#include <stm32f4xx_tim.h>
#include <stm32f4xx_dac.h>
#include <stm32f4xx_exti.h>

#include "fifo3.h"

using namespace STM32F4;
using namespace sensors;
using namespace devices;

accelerometer_data ad;
accelerometer_data ad4;
accelerometer_data a357;
gyro_data gd;

int16_t volume = 0x3f;
int volume_changed = 0;
uint8_t muted = 0;

extern "C" {
#include <HAL/STM32F4/mic/usbd_audio_core.h>
#include <HAL/STM32F4/mic/usbd_usr.h>
#include <HAL/STM32F4/mic/usb_dcd_int.h>
}
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;


F4Timer t2(TIM2);
F4SPI spi(SPI1);
F4SPI spi2(SPI2);
F4GPIO nss_160(GPIOB, GPIO_Pin_10);
F4GPIO nss42688(GPIOC, GPIO_Pin_4);
STM32F4::F4UART dbg_uart(USART1);

F4Interrupt drdy;
ICM40608 icm4;
ICM40608 icm42688;
MPU6000 icm2;
ADXL357 adxl357;
BMI160 bmi;

FIFO<384*4> fifo;

float avgz_icm2 = 9.8;
float avgz_adxl = 9.8;

float lpf_adxl = 9.8;

float v_icm2 = 0;
float v_adxl = 0;

extern "C" void TIM2_IRQHandler(void)
{
	t2.call_callback();
}

void imu_cal_cb(void *p)
{
	icm2.read(&gd);
	icm2.read(&ad);
	//icm4.read(&gd4);
	//icm4.read(&ad4);
	bmi.read(&gd);
	accelerometer_data a160;
	bmi.read(&a160);
	adxl357.read(&a357);
	
	accelerometer_data a42688;
	gyro_data g42688;
	icm42688.read(&g42688);
	icm42688.read(&a42688);
	
	float acc[12] = {a160.x, a160.y, a160.z, a357.x, a357.y, a357.z, ad.x, ad.y, ad.z, a42688.x, a42688.y, a42688.z};
	float temperature[4] = {a160.temperature, a357.temperature, ad.temperature, a42688.temperature};
	const char* name[4] = {"BMI160", "ADXL357", "ICM20602", "ICM42688"};
	
	imu_cal_feed(acc+3*3, temperature+3, name+3);	
}

void icm_cb(void *p)
{
	icm2.read(&gd);
	icm2.read(&ad);
	adxl357.read(&a357);
		
	float alpha = 0.9999;
	float _1_alpha = 1- alpha;

	float alpha2 = 0.5;
	float _1_alpha2 = 1- alpha2;
	
	avgz_icm2 = avgz_icm2 * alpha + ad.z * _1_alpha;
	avgz_adxl = avgz_adxl * alpha + a357.z * _1_alpha;
	lpf_adxl = lpf_adxl * alpha2 + a357.z * _1_alpha2;

	// fusing
	float dt = 1/4000.0f;
	float alpha3 = 0.999;
	v_icm2 += dt * (ad.z - avgz_icm2);
	v_icm2 *= alpha3;

	v_adxl += dt * (lpf_adxl - avgz_adxl);
	v_adxl *= alpha3;
	
	
	// output
	static float scale = 1.0f / 0.00106530f;
//	int16_t data[2] = {ad.z * 1024 / 9.8065f, a357.z * 1024 / 9.8065f};
	int16_t data[2] = {gd.z *scale, gd.x * scale};
	//int16_t data[2] = {v_icm2 * 4096, v_adxl * 4096};
	for(int i=0; i<12; i++)
		fifo.put(data, 4);
}

void int_cb(void *p, int flag)
{
	icm_cb(p);
	//imu_cal_cb(p);
}

void pwm_init()
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	// open B4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3); // TIM3_CH1
	
	// Time base configuration
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = (2000)-1;
	TIM_TimeBaseStructure.TIM_Prescaler = 84/2-1;
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
}

void pwm_set(int value)
{
	TIM3->CCR1 = value;
}


int DAC_init()
{
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	// Configure DAC channe1 and DAC channel2 outputs pins //
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	DAC_InitTypeDef DAC_InitStruct;
	DAC_StructInit(&DAC_InitStruct);
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_Software;
	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits11_0;
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStruct);
	
	DAC_Cmd(DAC_Channel_1, ENABLE);	
	
	DAC->DHR12R1=2048;
	DAC_DualSoftwareTriggerCmd(ENABLE);
	
	return 0;
}

void DAC_set(int v)
{
	DAC->DHR12R1=v;
	DAC_DualSoftwareTriggerCmd(ENABLE);
}


float freq = 2;
float amplitude = 20;
int adc_low;
int adc_high;
F4ADC adc(ADC1, 8);



void pwm_cb(void *p)
{	
	static float PI = acos(-1.0f);
	float t = (systimer->gettime() % 1000000)*1e-6;
	pwm_set(1500 + amplitude * sin(t*2*PI*freq));
	
	int v = adc.read();
	v = (v-adc_low) * 4095 / (adc_high - adc_low);
	
	if (v < 0)
		v = 0;
	if (v > 4095)
		v = 4095;
	
	
	
	DAC_set(v);	
}

F4GPIO test(GPIOB, GPIO_Pin_5);

float accz = 0;
float dt = 1e-3;
const float RC = 1.0f/(2*3.1415926 * 5.5f);
float alpha = dt / (dt + RC);

void cb20602(void *p)
{
	test.write(1);
	icm2.read(&gd);
	icm2.read(&ad);
	
	// output
	accz = accz * (1-alpha) + abs(ad.z) * alpha;
	static float scale = 2048.0f / 9.8065f;
	int16_t data[2] = {accz *scale, ad.temperature+2000};
	for(int i=0; i<48; i++)
		fifo.put(data, 4);
	
	test.write(0);
}

int pwm2_init()
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
	
	TIM_SetCompare1(TIM3, 10);
	
	return 0;
}

int pwm2_set(int16_t value)
{
	TIM_SetCompare1(TIM3, value);

	return 0;
}



uint8_t reg5470[32];
uint8_t reg5470_4[32];
	F4GPIO scl(GPIOF, GPIO_Pin_7);
	F4GPIO sda(GPIOF, GPIO_Pin_8);
	HAL::I2C_SW i2c(&scl, &sda);

	uint8_t add = 0xC4;

uint8_t working_password = 0;
uint8_t password = 0;

int write5470(uint8_t password, int page)
{
	// check page
	reg5470[0x13] = 0xff;
	i2c.read_reg(add, 0x13, &reg5470[0x13]);
	if ((reg5470[0x13] & 0x7) != page)
		return -1;
	
	// copy registers
	// MTP code not modified
	/*
	for(int i=0; i<15; i++)
	{
		i2c.read_reg(add, i, reg5470+i);
		i2c.write_reg(add, i+0x40, reg5470[i]);
		
		i2c.read_reg(add, i+0x40, reg5470_4+i);
		
		if (reg5470_4[i] != reg5470[i])
			return -2;
	}
	*/
	
	// fill password and start programming
	//i2c.write_reg(add, 0x11, password);
	i2c.write_reg(add, 0x11, 0x95);
	i2c.write_reg(add, 0x11, 0x63);
	reg5470[0x0E] |= 0x20;
	i2c.write_reg(add, 0x0E, reg5470[0x0E]);
	
	// wait for MTP write done?
	systimer->delayms(150);
	while (reg5470[0x0E] & 0x20)
		i2c.read_reg(add, 0x0E, &reg5470[0x0E]);
	
	//
	reg5470[0x13] = 0xff;
	i2c.read_reg(add, 0x13, &reg5470[0x13]);
	
	if ((reg5470[0x13] & 0x7) != page)
		return 0;
	return -3; 
}

void test_mp5470()
{
	F4GPIO EN(GPIOF, GPIO_Pin_9);
	F4GPIO run(GPIOF, GPIO_Pin_10);
	
	EN.write(0);
	EN.set_mode(HAL::MODE_OUT_PushPull);
	run.write(0);
	run.set_mode(HAL::MODE_OUT_PushPull);

	systimer->delayms(20);
	
	
	i2c.set_speed(50);
	
	for(int i=0; i<256; i+=2)
	{
		if (i2c.read_reg(i, 0, reg5470) == 0)
		{
			add = i;
			break;
		}
	}
	for(int j=0; j<20; j++)
		i2c.read_reg(add, j, reg5470+j);
	
	
	// vref = 550 + 10 * regvref mV
	//
	int vset[4] = {3300, 1800, 1200, 3800};
	int force_pwm[4] = {0, 0, 1, 1};
	for(int i=0; i<4; i++)
	{
		int mv_set = vset[i];
		uint8_t reg = 0;
		if (mv_set >= 1650)
		{
			reg = 0x80;
			mv_set /= 3;
		}
		reg |= uint8_t((mv_set - 550) / 10);
		i2c.write_reg(add, 2+i*3, reg);
		reg5470[1+i*3] &= ~(1<<6);
		reg5470[1+i*3] |= force_pwm[i]<<6;
		i2c.write_reg(add, 1+i*3, reg5470[1+i*3]);
	}
	i2c.write_reg(add, 0x0D, 0xc2);
		
	systimer->delayms(20);
	EN.write(1);
	systimer->delayms(20);
	for(int j=0; j<20; j++)
	{
		i2c.read_reg(add, j, reg5470+j);
		i2c.read_reg(add, j+0x40, reg5470_4+j);
	}
	
	while(1)
	{
		if (password)
		{
			for(int i=0; i<256; i++)
			{
				int o = write5470(i, 1);
				printf("write5470(%02x)=%d\n", i, o);
			}
			password = 0;
		}
		
		/*
		run.write(1);
		systimer->delayus(150);
		run.write(0);
		systimer->delayms(20);
		*/
	}
}


int o = 0;
float max_power = 1;
float kp = 0.05f;
float kI = 0.01f;
float tset = 50;
float control = 0;
float I = 0;
float p = 0;

float override = 0;

int main()
{
	test_mp5470();
	F4GPIO nss_20602(GPIOC, GPIO_Pin_5);

	nss_20602.set_mode(HAL::MODE_OUT_PushPull);
	test.set_mode(HAL::MODE_OUT_PushPull);
	nss_20602.write(1);

	
	icm2.init(&spi,&nss_20602);
	
	t2.set_period(1000);	
	
	t2.set_callback(cb20602, NULL);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USBD_Init(&USB_OTG_dev,
		USB_OTG_HS_CORE_ID,
		&USR_desc, 
		&AUDIO_cb, 
		&USR_cb);
		
	
	pwm2_init();
	float dt = 1e-3;
	while(1)
	{

		p = tset - gd.temperature;
		if (control > 0 && control < 1 && override < 0)
			I += p* dt;
		
		float imax = max_power / kI;
		if (I<-imax)
			I = -imax;
		if (I>imax)
			I = imax;
		
		systimer->delayms(1);
		
		control = p * kp + I * kI;
		if (control > max_power)
			control = max_power;
		if (control < 0)
			control = 0;
		
		if (override >= 0)
			control = override;
		
		pwm2_set(control*4094);		
	}
}

int main_hz()
{
	dbg_uart.set_baudrate(115200);
	//while(1)
		dbg_uart.write("hello", 1);

	F4GPIO nss(GPIOB, GPIO_Pin_5);
	F4GPIO nss_357(GPIOE, GPIO_Pin_5);
	F4GPIO nss_20602(GPIOD, GPIO_Pin_3);
	F4SPI spi3(SPI3);

	nss_357.set_mode(HAL::MODE_OUT_PushPull);
	nss_20602.set_mode(HAL::MODE_OUT_PushPull);
	nss42688.set_mode(HAL::MODE_OUT_PushPull);
	nss_357.write(1);
	nss_20602.write(1);
	nss42688.write(1);

	//icm4.init(&spi, &nss);
	//while(1)
	bmi.init(&spi2, &nss_160);
	
	icm42688.init(&spi, &nss42688);
	
	icm2.init(&spi3,&nss_20602);

	adxl357.init(&spi3, &nss_357);
	
	imu_cal_init(1);
	
	t2.set_period(1000);
	pwm_init();
	pwm_set(1500 - amplitude);
	systimer->delayms(500);
	adc_low = adc.read();
	pwm_set(1500 + amplitude);
	systimer->delayms(500);
	adc_high = adc.read();
	
	DAC_init();
	
	int range = adc_high - adc_low;
	adc_low -= range;
	adc_high += range;
	
	
	//t3.set_callback(pwm_cb, NULL);
	drdy.init(GPIOC, GPIO_Pin_7, HAL::interrupt_rising);
	drdy.set_callback(int_cb, NULL);

	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USBD_Init(&USB_OTG_dev,
		USB_OTG_HS_CORE_ID,
		&USR_desc, 
		&AUDIO_cb, 
		&USR_cb);
		
	
	while(1)
	{
		systimer->delayms(950);
	}
}

extern "C" int codec_pop(void *p, int bytes)
{
	return fifo.pop(p, bytes);
}

extern "C"
{
#ifdef USE_USB_OTG_HS
void OTG_HS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ; 
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line20);
}

void OTG_HS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

void OTG_HS_EP1_IN_IRQHandler(void)
{
  USBD_OTG_EP1IN_ISR_Handler (&USB_OTG_dev);
}

void OTG_HS_EP1_OUT_IRQHandler(void)
{
  USBD_OTG_EP1OUT_ISR_Handler (&USB_OTG_dev);
}

#endif	
}