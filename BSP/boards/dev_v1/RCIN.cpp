#include "RCIN.h"

#include <string.h>
#include <stm32f4xx_exti.h>


static int64_t rc_update[6];
static int16_t rc_input[6];
static int16_t rc_static[8][2];

static int last_high_tim = -1;
static int ppm_channel_id = 0;
static int ppm_channel_count = 0;


static float f_min(float a, float b)
{
	return a > b ? b : a;
}
static float f_max(float a, float b)
{
	return a > b ? a : b;
}

int handle_ppm(int now)
{
	float delta = 0;
	if (last_high_tim < 0)
	{
		last_high_tim = now;
		return 0;
	}
	
	if (now > last_high_tim)
		delta = now - last_high_tim;
	else
		delta = now + 60000 - last_high_tim;

	last_high_tim = now;

	if (delta > 2100)
	{
		ppm_channel_count = ppm_channel_id;
		ppm_channel_id = 0;
		//TRACE("        %.0f\r", delta);
	}
	else if (ppm_channel_id < sizeof(rc_input)/sizeof(rc_input[0]))
	{
		rc_input[ppm_channel_id] = delta;
		rc_static[ppm_channel_id][0] = f_min(rc_static[ppm_channel_id][0], rc_input[ppm_channel_id]);
		rc_static[ppm_channel_id][1] = f_max(rc_static[ppm_channel_id][1], rc_input[ppm_channel_id]);
		//TRACE("%.0f,", g_pwm_input[ppm_channel_id-1]);

		rc_update[ppm_channel_id] = systimer->gettime();

		ppm_channel_id++;
	}
	return 0;
}

extern "C" void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		handle_ppm(systimer->gettime());
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
}

dev_v1::RCIN::RCIN()
{
	// open PC5 as input
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	EXTI_ClearITPendingBit(EXTI_Line5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5);
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

// total channel count
int dev_v1::RCIN::get_channel_count()
{
	return ppm_channel_count;
}

// return num channel written to out pointer
int dev_v1::RCIN::get_channel_data(int16_t *out, int start_channel, int max_count)
{
	int count = f_min(ppm_channel_count - start_channel, max_count);
	memcpy(out, rc_input + start_channel, count * sizeof(int16_t));
	
	return count;
}

// return num channel written to out pointer
int dev_v1::RCIN::get_channel_update_time(int64_t *out, int start_channel, int max_count)
{
	int count = f_min(ppm_channel_count - start_channel, max_count);
	memcpy(out, rc_static + start_channel, count * sizeof(int64_t));
	
	return count;
}
