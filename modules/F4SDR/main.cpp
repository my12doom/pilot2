#include <stdio.h>
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


using namespace STM32F4;
using namespace HAL;

F4GPIO led0(GPIOC, GPIO_Pin_6);
F4GPIO led1(GPIOC, GPIO_Pin_7);

#define BUFFER_COUNT 32768
static int16_t buffer[BUFFER_COUNT];


int data_valid = 0;
int data_flying = 0;

extern "C" void DMA2_Stream6_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6))	
	{
		data_valid |= 2;
	}
	if(DMA_GetITStatus(DMA2_Stream6, DMA_IT_HTIF6))	
	{
		data_valid |= 1;
	}
	DMA_ClearITPendingBit(DMA2_Stream6, DMA_FLAG_HTIF6 | DMA_FLAG_TCIF6);
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
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&buffer[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = sizeof(buffer)/sizeof(buffer[0]);
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

uint8_t rx_buffer[512];

int usb_event_cb(F4HSBulk *h, int event, void *_p, int size)
{
	if (event == tx_done)
	{
		data_valid &= data_flying ? 0xfe : 0xfd;
	}
	
	if (event == tx_ready || event == tx_done)
	{
		if (data_valid)
		{
			data_flying = data_valid & 1;
			
			
			h->ioctl(tx_block, data_flying ? buffer : (buffer + BUFFER_COUNT/2), sizeof(buffer)/2);
		}
		else
		{
			return -1;	// no more data for now
		}
	}
	
	else if (event == rx_ready)
	{
		h->ioctl(rx_block, rx_buffer, 512);
	}
	
	else if (event == rx_done)
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
			int i;
			uint32_t *data = (uint32_t*)(p+1);
			uint32_t address = data[0];
			int count = data[1];
			data += 2;
			
			for(i=0; i<count/4; i++)
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
	return 0;
}

extern "C" void SetSysClock();
void switch_to_hsi()
{
    
}

F4SPI spi(SPI3);
F4GPIO cs(GPIOD, GPIO_Pin_0);			// ADC driver & ADC
F4GPIO cs4351(GPIOA, GPIO_Pin_9);		// adf4351
F4GPIO cs2871(GPIOC, GPIO_Pin_9);		// max2871

int write_ad9251(int16_t add, uint8_t v)
{
	cs.write(0);
	spi.txrx((add>>8)&0xf);
	spi.txrx(add&0xff);
	spi.txrx(v);
	cs.write(1);

	return 0;
}

uint8_t read_ad9251(int16_t add)
{
	uint8_t v;
	cs.write(0);
	spi.txrx(0x80 | ((add>>8)&0xf));
	spi.txrx(add&0xff);
	v = spi.txrx(0);
	cs.write(1);

	return v;
}

int configure_ad9251()
{
	cs.set_mode(MODE_OUT_PushPull);
	cs.write(1);
	spi.set_mode(0,0);
	spi.set_speed(4000000);


	// use twos complement output
	write_ad9251(0x14, 0x01);
	write_ad9251(0xff, 0x01);
	
	for(int i=0; i<25; i++)
		printf("%d=%02x\n", i, read_ad9251(i));
	
	return 0;
}

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
	
	
	// si5351 sample clock
	F4GPIO scl(GPIOA, GPIO_Pin_11);
	F4GPIO sda(GPIOA, GPIO_Pin_12);
	
	I2C_SW i2c(&scl, &sda);
	si5351 si;
	si.set_i2c(&i2c);
	si.set_ref_freq(26.0);
	si.set_pll(0, 720);
	si.set_output_freq(0, 0, 720/80.0);
	si.set_output(0, false, false);
	
	// fpga configureation
	F4GPIO fpga_erase(GPIOA, GPIO_Pin_0);
	F4GPIO init_b(GPIOA, GPIO_Pin_6);
	fpga_erase.set_mode(MODE_OUT_PushPull);
	fpga_erase.write(0);
	systimer->delayms(5);
	fpga_erase.write(1);
	fpga_config(&init_b);	

	// LO2 config
	ADF4351 LO2;
	LO2.init(&spi, &cs4351);
	LO2.set_ref(26000000, false, false, 1);
	LO2.set_freq(888000000);				// 1st IF filter passband: 890 - 915Mhz, stop band: <880Mhz or > 925Mhz
	LO2.set_output(true, false, true);		// baseband mapped to 888 of IF2.
	
	// LO1 config
	MAX2871 LO1;
	LO1.init(&spi, &cs2871);
	LO1.set_ref(26000000, false, false, 1);
	LO1.set_freq(1507000000);
	LO1.set_output(true, true, false);

	// ADC config
	configure_ad9251();
	parallel_port_config();

	// IF config	
	spi.set_mode(0,0);
	spi.set_speed(4000000);
	cs.set_mode(MODE_OUT_PushPull);
	cs.write(1);
	systimer->delayus(1);
	cs.write(0);
	spi.txrx(15 | 0x00);
	cs.write(1);

	// USB
	F4HSBulk bulk;
	bulk.ioctl(set_callback, (void*)usb_event_cb, 0);
	
	while(1)
	{
	}
}
