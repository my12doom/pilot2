#include "board.h"

#include <HAL/STM32F0/F0SPI.h>
#include <HAL/STM32F0/F0GPIO.h>
#include <HAL/STM32F0/F0Interrupt.h>
#include <HAL/STM32F0/F0SysTimer.h>
#include <HAL/STM32F0/F0Timer.h>
#include <string.h>
#include "PPMOUT_f0.h"
#include <HAL/STM32F0/F0UART.h>

using namespace STM32F0;
using namespace HAL;


HAL::ISPI *spi;
HAL::IGPIO *cs;
HAL::IGPIO *ce;
HAL::IGPIO *irq;
HAL::IGPIO *dbg;
HAL::IGPIO *dbg2;
HAL::IGPIO *SCL;
HAL::IGPIO *SDA;
HAL::IInterrupt *interrupt;
HAL::ITimer *timer;
int16_t adc_data[6] = {0};
extern HAL::IUART *uart;
		int dac_config();

namespace sheet1
{
	F0GPIO cs(GPIOB, GPIO_Pin_12);
	F0GPIO ce(GPIOB, GPIO_Pin_3);
	F0GPIO irq(GPIOA, GPIO_Pin_15);
	F0GPIO dbg(GPIOB, GPIO_Pin_11);
	
	F0GPIO dbg2(GPIOB, GPIO_Pin_10);
	F0GPIO SCL(GPIOC, GPIO_Pin_13);
	F0GPIO SDA(GPIOC, GPIO_Pin_14);
	
	F0SPI spi;
	F0Interrupt interrupt;
	F0Timer timer(TIM14);
	
	
	static void ADC1_Mode_Config(void)
	{
		ADC_InitTypeDef ADC_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
		// Configure GPIO0~2 as analog input
		for(int ADC_Channel=0; ADC_Channel<7; ADC_Channel++)
		{
			GPIO_InitStructure.GPIO_Pin = (1 << (ADC_Channel%8));
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(ADC_Channel>8?GPIOB:GPIOA, &GPIO_InitStructure);
		}
		
		DMA_InitTypeDef DMA_InitStructure;
		DMA_DeInit(DMA1_Channel1);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);	 //ADC地址
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adc_data;//内存地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_BufferSize = 6;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址固定
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址固定
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//半字
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//循环传输
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		DMA_Init(DMA1_Channel1, &DMA_InitStructure);		
		DMA_Cmd(DMA1_Channel1, ENABLE);
		
		/* ADC1 configuration */
		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE ;
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
		ADC_InitStructure.ADC_ExternalTrigConv = 0;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
		ADC_Init(ADC1, &ADC_InitStructure);
		
		RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4); 


		ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_239_5Cycles);
		ADC_ChannelConfig(ADC1, ADC_Channel_1, ADC_SampleTime_239_5Cycles);
		ADC_ChannelConfig(ADC1, ADC_Channel_2, ADC_SampleTime_239_5Cycles);
		ADC_ChannelConfig(ADC1, ADC_Channel_4, ADC_SampleTime_239_5Cycles);
		ADC_ChannelConfig(ADC1, ADC_Channel_5, ADC_SampleTime_239_5Cycles);
		ADC_ChannelConfig(ADC1, ADC_Channel_6, ADC_SampleTime_239_5Cycles);
		
		ADC_OverrunModeCmd(ADC1, ENABLE);
		ADC_GetCalibrationFactor(ADC1);
		ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
		ADC_DMACmd(ADC1, ENABLE);
		ADC_Cmd(ADC1, ENABLE);
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN) == RESET);
		ADC_StartOfConversion(ADC1);
	}
	
	int sheet1_init()
	{
		FLASH_SetLatency(FLASH_Latency_1);
		::cs = &cs;
		::ce = &ce;
		::irq = &irq;
		::dbg = &dbg;
		::dbg2 = &dbg2;
		::SCL = &SCL;
		::SDA = &SDA;
		::spi = &spi;
		::interrupt = &interrupt;
		::timer = &timer;
		
		spi.init(SPI2);
		interrupt.init(GPIOA, GPIO_Pin_15, interrupt_falling);
		
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		static PPMOUT ppmout;
		::ppm = &ppmout;
		
		static STM32F0::F0UART uart(USART1);
		::uart = &uart;
		
		dac_config();
		
		//ADC1_Mode_Config();
		
		return 0;
	}	
}

int board_init()
{
	return sheet1::sheet1_init();
}



void read_channels(int16_t *channel, int max_channel_count)
{
	// channel map:
	// PA0		throttle
	// PA1		rudder
	// PA2		roll
	// PA4		pitch
	// PA5		left switch
	// PA6		right switch
		
	channel[0] = adc_data[2];
	channel[0] = adc_data[3];
	channel[0] = adc_data[0];
	channel[0] = adc_data[1];
	channel[0] = adc_data[4];
	channel[0] = adc_data[5];
}
