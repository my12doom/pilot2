#include <stdio.h>
#include <string.h>
#include "i2s.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_dma.h"
#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4SysTimer.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4HSBulk.h>

#include <HAL/aux_devices/si5351.h>
#include <HAL/aux_devices/ADF4351.h>
#include <HAL/aux_devices/MAX2871.h>
#include "fpga_config.h"
#include "F4SDR.h"

using namespace STM32F4;
using namespace HAL;
using namespace F4SDR;

// board resource
F4GPIO led0(GPIOC, GPIO_Pin_6);
F4GPIO led1(GPIOC, GPIO_Pin_7);
ADF4351 LO2;
MAX2871 LO1;
F4SPI spi(SPI3);
F4GPIO cs_adc(GPIOD, GPIO_Pin_0);		// ADC driver & ADC
F4GPIO cs4351(GPIOA, GPIO_Pin_9);		// adf4351
F4GPIO cs2871(GPIOC, GPIO_Pin_9);		// max2871

// bulk buffer
#define BUFFER_COUNT 32768
static int16_t adc_buffer[BUFFER_COUNT];
int data_valid = 0;
int data_flying = 0;

// functions
void tune(uint64_t center_freq);
void set_adc_gain(bool LNA, uint8_t gain);
int parallel_port_config();
int usb_event_cb(F4HSBulk *h, int event, void *_p, int size);

void set_adc_gain(bool LNA, uint8_t gain)
{
	spi.set_mode(0,0);
	spi.set_speed(4000000);
	cs_adc.set_mode(MODE_OUT_PushPull);
	cs_adc.write(1);
	systimer->delayus(1);
	cs_adc.write(0);
	spi.txrx((LNA ? 0x80 : 0) | gain);
	cs_adc.write(1);	
}

extern "C" void DMA2_Stream6_IRQHandler(void)
{
	bool overflow = false;
	if(DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6))	
	{
		if (data_valid & 2)
			overflow = true;
		data_valid |= 2;
	}
	if(DMA_GetITStatus(DMA2_Stream6, DMA_IT_HTIF6))	
	{
		if (data_valid & 1)
			overflow = true;
		data_valid |= 1;
	}
	DMA_ClearITPendingBit(DMA2_Stream6, DMA_FLAG_HTIF6 | DMA_FLAG_TCIF6);
	
	led1.write(!overflow);

}

extern "C" void DMA2_Stream2_IRQHandler(void)
{
	bool overflow = false;
	if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))
	{
		if (data_valid & 2)
			overflow = true;
		data_valid |= 2;
	}
	if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_HTIF2))
	{
		if (data_valid & 1)
			overflow = true;
		data_valid |= 1;
	}
	DMA_ClearITPendingBit(DMA2_Stream2, DMA_FLAG_HTIF2 | DMA_FLAG_TCIF2);
	
	led1.write(!overflow);
		
}



int parallel_port_config()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);	
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_DeInit(TIM1);
	TIM_InternalClockConfig(TIM1);
	TIM_TimeBaseStructure.TIM_Prescaler = 0xff;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period=0xffff;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter    = 0x0;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
	TIM_Cmd(TIM1,ENABLE);
	
	TIM_DMACmd(TIM1, TIM_DMA_CC1, ENABLE ); /* Enable TIM1_CC1 DMA Requests     */
	
	DMA_InitTypeDef DMA_InitStructure = {0};
	DMA_InitStructure.DMA_Channel = DMA_Channel_0; 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(GPIOE->IDR));
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adc_buffer[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = sizeof(adc_buffer)/sizeof(adc_buffer[0]);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
	DMA_DeInit(DMA2_Stream6);
	DMA_Init(DMA2_Stream6, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream6, ENABLE);
	
	DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Stream6, DMA_IT_HT, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
	return 0;
}

int handle_IN_event(F4HSBulk *h, control_verndor_transfer *t)
{
	char outbuf[64];
	int outsize = 0;

	switch (t->request)
	{
	case ping:
		for(int i=0; i<t->length; i++)
			outbuf[i] = i;
		outsize = t->length;
		break;
	}

	h->ioctl(tx_ctrl_block, outbuf, outsize);

	return 0;
}

int handle_OUT_event(F4HSBulk *h, control_verndor_transfer *t)
{
	switch (t->request)
	{
	case reset_mcu:
		NVIC_SystemReset();
		break;

	case erase_fpga_rom:		
		if (t->length == 4 && *(uint32_t*)t->data == 0xDEADBEAF)
		{
			uint32_t sectors_to_erase[] = {FLASH_Sector_4, FLASH_Sector_5, FLASH_Sector_6, FLASH_Sector_7, FLASH_Sector_8, FLASH_Sector_9, FLASH_Sector_10, };
			int i,j;
			FLASH_Unlock();
			for(i=0; i<sizeof(sectors_to_erase)/sizeof(sectors_to_erase[0]); i++)
			{
				// empty check
				uint32_t *p = (uint32_t*)(0x08010000 + i * (i==1?0x10000:0x20000));
				int word_count = i==0?0x4000:0x8000;
				for(j=0; j<word_count; j++)
				{
					if (p[j] != 0xffffffff)
					{
						FLASH_EraseSector(sectors_to_erase[i], VoltageRange_3);
						FLASH_WaitForLastOperation();
						break;
					}
				}
			}
		}
		break;
	
	case write_fpga_rom:
		{
			uint32_t address = (t->value << 16) | t->index;
			uint32_t *data = (uint32_t*)t->data;
			for(int i=0; i<t->length; i+=4)
				FLASH_ProgramWord(address+i, data[i/4]);
		}
		break;

	case F4SDR::tune:
		// 1st IF filter passband: 890 - 915Mhz, stop band: <880Mhz or > 925Mhz
		// baseband mapped to 888 of IF2.
		if (t->length == 8)
		{
			if (*(uint64_t*)t->data > 888000000)				
				LO1.set_freq(*(uint64_t*)t->data-888000000);
			else
				LO1.set_freq(888000000 - *(uint64_t*)t->data);
		}
		break;

	case set_gain:
		if (t->length == 1)
			set_adc_gain(t->data[0]&0x80, t->data[0]&0x7f);
		break;
	
	default:
		break;
	}
	return 0;
}

int usb_event_cb(F4HSBulk *h, int event, void *_p, int size)
{
	switch (event)
	{
	case tx_done:
		data_valid &= data_flying ? 0xfe : 0xfd;
	
	case tx_ready:
		if (data_valid)
		{
			int next_part = (data_valid == 3) ? (1-data_flying) : (data_valid & 1);
			
			if (next_part == data_flying)
				led0.write(0);
			else
				led0.write(1);
			
			data_flying = next_part;
			
			
			h->ioctl(tx_block, data_flying ? adc_buffer : (adc_buffer + BUFFER_COUNT/2), sizeof(adc_buffer)/2);
		}
		else
		{
			return -1;	// no more data for now
		}
		break;

	case control_IN:
		handle_IN_event(h, (control_verndor_transfer*)_p);
		break;

	case control_OUT:
		handle_OUT_event(h, (control_verndor_transfer*)_p);
		break;

	/*
	case rx_ready:
		static uint8_t rx_buffer[512];
		h->ioctl(rx_block, rx_buffer, sizeof(rx_buffer));
		break;
	
	case rx_done:
		{
			// handle incoming data
			uint8_t *p = (uint8_t *)_p;
			if (p[0] == 0x81)
			{
				// erase fpga firmware flash
				uint32_t sectors_to_erase[] = {FLASH_Sector_4, FLASH_Sector_5, FLASH_Sector_6, FLASH_Sector_7, FLASH_Sector_8, FLASH_Sector_9, FLASH_Sector_10, };
				int i,j;
				FLASH_Unlock();
				for(i=0; i<sizeof(sectors_to_erase)/sizeof(sectors_to_erase[0]); i++)
				{
					// empty check
					uint32_t *p = (uint32_t*)(0x08010000 + i * (i==1?0x10000:0x20000));
					int word_count = i==0?0x4000:0x8000;
					for(j=0; j<word_count; j++)
					{
						if (p[j] != 0xffffffff)
						{
							FLASH_EraseSector(sectors_to_erase[i], VoltageRange_3);
							FLASH_WaitForLastOperation();
							break;
						}
					}
				}
			}
			
			else if (p[0] == 0x82)
			{
				uint32_t *data = (uint32_t*)(p+1);
				uint32_t address = data[0];
				int count = data[1];
				data += 2;
				
				for(int i=0; i<count/4; i++)
					FLASH_ProgramWord(address+i*4, data[i]);
			}
			else if (p[0] == 0x83)
			{
				NVIC_SystemReset();
			}
			else if (p[0] == 0x84)
			{
				// do nothing
			}
		}

		break;
	*/


	}

	return 0;
}

int write_ad9251(int16_t add, uint8_t v)
{
	cs_adc.write(0);
	spi.txrx((add>>8)&0xf);
	spi.txrx(add&0xff);
	spi.txrx(v);
	cs_adc.write(1);

	return 0;
}

uint8_t read_ad9251(int16_t add)
{
	uint8_t v;
	cs_adc.write(0);
	spi.txrx(0x80 | ((add>>8)&0xf));
	spi.txrx(add&0xff);
	v = spi.txrx(0);
	cs_adc.write(1);

	return v;
}

int configure_ad9251()
{
	cs_adc.set_mode(MODE_OUT_PushPull);
	cs_adc.write(1);
	spi.set_mode(0,0);
	spi.set_speed(4000000);

	write_ad9251(0x14, 0x01);	// use twos complement output	
	write_ad9251(0x0b, 0x00);	// clock divider : /1
	write_ad9251(0x09, 0x01);	// clock duty cycle stabalize
	write_ad9251(0xff, 0x01);	// execute
	
	return 0;
}


extern "C" void SetSysClock();
int main()
{
	// use clock from USB3320
	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));		// switch to HSI
	F4GPIO usb_reset(GPIOC, GPIO_Pin_14);
	usb_reset.set_mode(MODE_OUT_PushPull);
	usb_reset.write(0);
	systimer->delayms(10);
	usb_reset.write(1);
	systimer->delayms(5);	
	SetSysClock();
	SystemCoreClockUpdate();
	led0.write(1);
	led1.write(1);
	led0.set_mode(MODE_OUT_OpenDrain);
	led1.set_mode(MODE_OUT_OpenDrain);
	if (SystemCoreClock != 168000000)
		led0.write(0);
	else
		led1.write(0);
	PWR->CR &= ~PWR_CR_VOS;
	
	// erase fpga
	F4GPIO fpga_erase(GPIOA, GPIO_Pin_0);
	fpga_erase.set_mode(MODE_OUT_PushPull);
	fpga_erase.write(0);
	systimer->delayms(5);
	fpga_erase.write(1);
	
	// si5351 sample clock
	F4GPIO scl(GPIOA, GPIO_Pin_11);
	F4GPIO sda(GPIOA, GPIO_Pin_12);
	
	I2C_SW i2c(&scl, &sda);
	si5351 si;
	si.set_i2c(&i2c);	
	si.set_ref_freq(26.0);
	si.set_pll(0, 720);
	si.set_output_freq(0, 0, 720/80.0);
	si.set_output_freq(1, 0, 720/80.0);
	si.set_output_freq(2, 0, 720/80.0);
	si.set_output(0, true, false);
	si.set_output(1, false, false);
	si.set_output(2, true, true);
	
	// fpga configureation
	F4GPIO init_b(GPIOA, GPIO_Pin_6);
	fpga_config(&init_b);	

	// LO2 config
	LO2.init(&spi, &cs4351);
	LO2.set_ref(26000000, false, false, 1);
	LO2.set_freq(788000000);				// 1st IF filter passband: 890 - 915Mhz, stop band: <880Mhz or > 925Mhz
	LO2.set_output(true, false, true);		// baseband mapped to 888 of IF2.
	
	// LO1 config
	LO1.init(&spi, &cs2871);
	LO1.set_ref(26000000, false, false, 1);
	LO1.set_freq(1507000000);
	LO1.set_output(true, true, false);

	// ADC config
	cs2871.write(0);
	parallel_port_config();
	systimer->delayms(1);
	cs2871.write(1);
	configure_ad9251();

	// IF config	
	set_adc_gain(true, 10);

	// USB
	F4HSBulk bulk;
	bulk.ioctl(set_callback, (void*)usb_event_cb, 0);
	
	while(1)
	{
	}
}
