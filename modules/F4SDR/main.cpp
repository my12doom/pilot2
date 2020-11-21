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


using namespace STM32F4;
using namespace HAL;

F4GPIO led0(GPIOC, GPIO_Pin_6);
F4GPIO led1(GPIOC, GPIO_Pin_7);

#define BUFFER_COUNT 32768
static int16_t adc_buffer[BUFFER_COUNT];


int data_valid = 0;
int data_flying = 0;

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

	else if (event == control_IN)
	{
		control_verndor_transfer *t = (control_verndor_transfer*)_p;

		char word[] = "HelloWorld";

		int count = strlen(word) > t->length ? t->length : strlen(word);
		h->ioctl(tx_ctrl_block, word, count);
	}

	else if (event == control_OUT)
	{
		control_verndor_transfer *t = (control_verndor_transfer*)_p;

		printf("control_OUT : %d bytes\n", t->length);
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

static const unsigned char _acadc[0xC9] = {
  0x6C, 0x2F, 0x3B, 0x34, 0xFF, 0x30, 0xF7, 0x20, 0x73, 0x31, 0x25, 0x08, 0xD1, 0x0E, 0xF3, 0x39,
  0xA9, 0x18, 0x1B, 0x07, 0xC4, 0x15, 0x10, 0x31, 0x11, 0x02, 0x39, 0x21, 0x86, 0x15, 0x37, 0x3D,
  0xA1, 0x08, 0x9F, 0x16, 0x92, 0x2C, 0xE4, 0x37, 0x33, 0x35, 0x63, 0x00, 0x1F, 0x0F, 0x06, 0x1D,
  0xBB, 0x2D, 0x6B, 0x01, 0xA8, 0x3A, 0x2B, 0x05, 0xEE, 0x32, 0xCE, 0x01, 0xF5, 0x24, 0x12, 0x35,
  0x70, 0x06, 0x6C, 0x2F, 0x3B, 0x34, 0xFF, 0x30, 0xF7, 0x20, 0x73, 0x31, 0x25, 0x08, 0xD1, 0x0E,
  0xF3, 0x39, 0xA9, 0x18, 0x1B, 0x07, 0xC4, 0x15, 0x10, 0x31, 0x11, 0x02, 0x39, 0x21, 0x86, 0x15,
  0x37, 0x3D, 0xA1, 0x08, 0x9F, 0x16, 0x92, 0x2C, 0xE4, 0x37, 0x33, 0x35, 0x11, 0x02, 0x39, 0x21,
  0x86, 0x15, 0x37, 0x3D, 0xA1, 0x08, 0x9F, 0x16, 0x92, 0x2C, 0xE4, 0x37, 0x33, 0x35, 0x63, 0x00,
  0x34, 0x0A, 0xE8, 0x2F, 0x75, 0x2C, 0x9E, 0x25, 0xBA, 0x0F, 0x5B, 0x03, 0x82, 0x1D, 0xF5, 0x35,
  0x0A, 0x14, 0xE5, 0x15, 0x03, 0x37, 0x49, 0x27, 0xEA, 0x3A, 0x0C, 0x09, 0x5E, 0x38, 0x68, 0x36,
  0xE1, 0x1D, 0xC1, 0x3F, 0xE2, 0x3B, 0x90, 0x39, 0x9D, 0x12, 0xF3, 0x28, 0xBA, 0x0F, 0x5B, 0x03,
  0x82, 0x1D, 0xF5, 0x35, 0x0A, 0x14, 0xE5, 0x15, 0x03, 0x37, 0x49, 0x27, 0xEA, 0x3A, 0x0C, 0x09,
  0x5E, 0x38, 0x68, 0x36, 0xE1, 0x1D, 0xC1, 0x3F, 0xE2
};

int16_t *ref_pn9 = ((int16_t*)_acadc);

int configure_ad9251()
{
	cs.set_mode(MODE_OUT_PushPull);
	cs.write(1);
	spi.set_mode(0,0);
	spi.set_speed(4000000);


	write_ad9251(0x14, 0x01);	// use twos complement output
	
	write_ad9251(0x0b, 0x00);	// clock / 2
	
	//write_ad9251(0x14, 0x00);
	write_ad9251(0x09, 0x01);
	write_ad9251(0xff, 0x01);

	// test mode
	/*
	int error = 0;
	for(int i=0; i<16; i++)
	{
		int16_t v = i;
		write_ad9251(0x0d, 6);
		write_ad9251(0x19, (v<<2)&0xff);
		write_ad9251(0x1a, v>>6);
		write_ad9251(0xff, 0x01);
		
		systimer->delayms(2);
		
		//write_ad9251(0x0d, 0x00);
		volatile int16_t v2 = adc_buffer[0];
		
		if (v != v2)
			//error ++;
			printf("%04x->%04x, delta=%04x\n", v, v2, v ^ v2);

		//systimer->delayms(1);
	}
	

	//write_ad9251(0x0e, 0x05);
	//write_ad9251(0xff, 0x01);
	systimer->delayms(1);

	
	for(int i=0; i<38; i++)
		printf("%d=%02x\n", i, read_ad9251(i));
	
	bool found = false;
	for(int i=0; i<5120; i++)
	{
		if (adc_buffer[i] == ref_pn9[0])
		{
			for(int j=0; j<73; j++)
			{
				if (adc_buffer[i+j] != ref_pn9[j])
					printf("%04x - %04x @ %d/%d\n", adc_buffer[i+j],ref_pn9[j], i, j);
			}
			
			//break;
			found = true;
		}
	}
	
	if (!found)
	{
		printf("PN9 not found\n");
	}
	
	write_ad9251(0x0d, 0x00);
	write_ad9251(0xff, 0x01);
	
	*/

	
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
	PWR->CR &= ~PWR_CR_VOS;
	
	// MCO test
	/*
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_1);
	*/

	// SPI1 TI mode test
	//spi1_ti_test();
	
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
	si.set_output(1, true, false);
	
	si.set_ref_freq(26.0);
	si.set_pll(0, 720);
	si.set_output_freq(1, 0, 720/80.0);
	si.set_output(1, false, false);
	
	// fpga configureation
	F4GPIO init_b(GPIOA, GPIO_Pin_6);
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
	cs2871.write(0);
	parallel_port_config();
	systimer->delayms(1);
	cs2871.write(1);
	configure_ad9251();

	// IF config	
	spi.set_mode(0,0);
	spi.set_speed(4000000);
	cs.set_mode(MODE_OUT_PushPull);
	cs.write(1);
	systimer->delayus(1);
	cs.write(0);
	spi.txrx(0x00 | 10);
	cs.write(1);

	// USB
	F4HSBulk bulk;
	bulk.ioctl(set_callback, (void*)usb_event_cb, 0);
	
	while(1)
	{
	}
}


int16_t spi1_dummy = 0;
void spi1_ti_test()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	F4GPIO usb_reset(GPIOC, GPIO_Pin_13);
	usb_reset.set_mode(MODE_OUT_PushPull);
	usb_reset.write(0);
	systimer->delayms(5);

	GPIO_InitTypeDef GPIO_InitStructure = 
	{
		GPIO_Pin_3,
		GPIO_Mode_AF,
		GPIO_Speed_100MHz,
		GPIO_OType_PP,
		GPIO_PuPd_NOPULL,
	};

	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3,GPIO_AF_SPI1);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_SSOutputCmd(SPI1,ENABLE);
	SPI_TIModeCmd(SPI1, ENABLE);
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
	
	SPI_Cmd(SPI1, ENABLE);


	DMA_InitTypeDef DMA_InitStructure = {0};
	DMA_InitStructure.DMA_Channel = DMA_Channel_3; 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI1->DR));
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
	DMA_DeInit(DMA2_Stream2);
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream2, ENABLE);
	
	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Stream2, DMA_IT_HT, ENABLE);

	// dummy clock gen
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&spi1_dummy;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_DeInit(DMA2_Stream5);
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream5, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// USB
	F4HSBulk bulk;
	bulk.ioctl(set_callback, (void*)usb_event_cb, 0);
	
	while(1)
	{
		/*
		if (SPI1->SR & SPI_I2S_FLAG_TXE)
		{
			SPI1->DR = spi1_dummy;			
			//v = SPI1->DR;
		}
		*/
	}
}
