#include "dcmi_ov2640.h"
#include <stm32f4xx_dcmi.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_i2c.h>

volatile uint8_t image_counter = 0;
volatile uint32_t frame_counter;
volatile uint32_t time_last_frame = 0;
volatile uint32_t cycle_time = 0;
volatile uint32_t time_between_next_images;
volatile uint8_t dcmi_calibration_counter = 0;

volatile uint8_t dcmi_image_buffer_unused = 3;

void dcmi_hw_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStruct;
	#ifdef USE_WAVESHARE
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOH | 
                         RCC_AHB1Periph_GPIOI, ENABLE);
		RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI);

		GPIO_PinAFConfig(GPIOH, GPIO_PinSource8, GPIO_AF_DCMI);
		GPIO_PinAFConfig(GPIOH, GPIO_PinSource9, GPIO_AF_DCMI);
		GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_DCMI);
		GPIO_PinAFConfig(GPIOH, GPIO_PinSource11, GPIO_AF_DCMI);
		GPIO_PinAFConfig(GPIOH, GPIO_PinSource12, GPIO_AF_DCMI);
		GPIO_PinAFConfig(GPIOH, GPIO_PinSource14, GPIO_AF_DCMI);

		GPIO_PinAFConfig(GPIOI, GPIO_PinSource5, GPIO_AF_DCMI);
		GPIO_PinAFConfig(GPIOI, GPIO_PinSource6, GPIO_AF_DCMI);
		GPIO_PinAFConfig(GPIOI, GPIO_PinSource7, GPIO_AF_DCMI);
		GPIO_PinAFConfig(GPIOI, GPIO_PinSource4, GPIO_AF_DCMI);
		/* DCMI GPIO configuration **************************************************/
		/* D0..D4(PH9/10/11/12/14), HSYNC(PH8) */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | 
									GPIO_Pin_12 | GPIO_Pin_14| GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;  
		GPIO_Init(GPIOH, &GPIO_InitStructure);

		/* D5..D7(PI4/6/7), VSYNC(PI5) */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_5;
		GPIO_Init(GPIOI, &GPIO_InitStructure);

		/* PCLK(PA6) */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	#endif
	#ifdef DK_HAL_STM32F407_V_1_0
		RCC_AHB1PeriphClockCmd(
				RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC
						| RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);
		/* Connect DCMI pins to AF13 */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI); //DCMI_HSYNC
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI_PIXCL
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI); //DCMI_VSYNC

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_DCMI); //DCMI_D0
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_DCMI);//DCMI_D1
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_DCMI); //DCMI_D2
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_DCMI); //DCMI_D3
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_DCMI); //DCMI_D4
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI_D5
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI); //DCMI_D6
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI); //DCMI_D7
		
		/* DCMI GPIO configuration */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_9 | GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;// | GPIO_Pin_10 | GPIO_Pin_12;
		//GPIO_Init(GPIOC, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4
				| GPIO_Pin_5 | GPIO_Pin_6;
		GPIO_Init(GPIOE, &GPIO_InitStructure);
	#endif
	
	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);

//	I2C_DeInit(I2C2);

//	I2C_Cmd(I2C2, ENABLE);
//	
//	/* Set the I2C structure parameters */
//	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
//	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
//	I2C_InitStruct.I2C_OwnAddress1 = 0xFE;
//	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
//	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//	I2C_InitStruct.I2C_ClockSpeed = 30000;

//	I2C_Init(I2C2, &I2C_InitStruct);

}
void dcmi_dma_init(uint32_t image_buffer_address,uint16_t buffer_size,camera_format ImageFormat)
{

	DCMI_InitTypeDef DCMI_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	
	/*** Configures the DCMI to interface with the mt9v034 camera module ***/
	/* Enable DCMI clock */
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);

	/* DCMI configuration */
	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_Continuous ;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_Low;//DCMI_VSPolarity_Low;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;

	/* Configures the DMA2 to transfer Data from DCMI */
	/* Enable DMA2 clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	/* DMA2 Stream1 Configuration */
	DMA_DeInit(DMA2_Stream1);
	
	DCMI_DeInit();
	while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE){}
		
	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&DCMI->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = image_buffer_address;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = buffer_size; // buffer size in date unit (word)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	//DMA_DoubleBufferModeConfig(DMA2_Stream1,(uint32_t) dcmi_image_buffer_8bit_2, DMA_Memory_0);
	//DMA_DoubleBufferModeCmd(DMA2_Stream1,ENABLE);
	switch(ImageFormat)
	{
		case BMP_QQVGA:
		{
			/* DCMI configuration */ 
			DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
			DCMI_Init(&DCMI_InitStructure);

			/* DMA2 IRQ channel Configuration */
			DMA_Init(DMA2_Stream1, &DMA_InitStructure);
			break;
		}
		case BMP_QVGA:
		{
			/* DCMI configuration */
			DCMI_Init(&DCMI_InitStructure);

			/* DMA2 IRQ channel Configuration */
			DMA_Init(DMA2_Stream1, &DMA_InitStructure);
			break;
		}
		 default:
		{
			/* DCMI configuration */ 
			//DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
			DCMI_Init(&DCMI_InitStructure);
			/* DMA2 IRQ channel Configuration */
			DMA_Init(DMA2_Stream1, &DMA_InitStructure);
			break;
		}
	}
	dcmi_it_init();
	dma_it_init();
	//enable JPEG mode
	//DCMI_JPEGCmd(ENABLE);
}
void dcmi_dma_init_7725(uint32_t image_buffer_address,uint16_t buffer_size)
{

	DCMI_InitTypeDef DCMI_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	
	/* Enable DCMI clock */
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);

	/* DCMI configuration */
	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_Continuous ;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;//DCMI_VSPolarity_Low;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;

	/* Configures the DMA2 to transfer Data from DCMI */
	/* Enable DMA2 clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	/* DMA2 Stream1 Configuration */
	DMA_DeInit(DMA2_Stream1);
	
	DCMI_DeInit();
	while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE){}
		
	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&DCMI->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = image_buffer_address;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = buffer_size; // buffer size in date unit (word)
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	//DMA_DoubleBufferModeConfig(DMA2_Stream1,(uint32_t) dcmi_image_buffer_8bit_2, DMA_Memory_0);
	//DMA_DoubleBufferModeCmd(DMA2_Stream1,ENABLE);
	
	/* DCMI configuration */ 
	DCMI_Init(&DCMI_InitStructure);

	/* DMA2 IRQ channel Configuration */
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);
		
	
	dcmi_it_init();
	dma_it_init();
	//enable JPEG mode
	//DCMI_JPEGCmd(ENABLE);
}

void dcmi_dma_enable()
{
	/* Enable DMA2 stream 1 and DCMI interface then start image capture */
	DMA_Cmd(DMA2_Stream1, ENABLE);
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);
}
void dcmi_start(void)
{  
    DMA_Cmd(DMA2_Stream1, ENABLE);   //开启DMA2,Stream1 
    DCMI_CaptureCmd(ENABLE);         //DCMI捕获使能  
}

/**
  * @brief  DCMI停止传输
  * @param  无
  * @retval 无
  */
void dcmi_stop(void)
{ 
    DCMI_CaptureCmd(DISABLE);       //DCMI捕获使关闭
    while(DCMI->CR&0X01);           //等待传输结束  	
    DMA_Cmd(DMA2_Stream1,DISABLE);  //关闭DMA2,Stream1
} 
void dcmi_dma_disable()
{
	while(DCMI->CR&0X01);  
	DMA_Cmd(DMA2_Stream1, DISABLE);
	DCMI_Cmd(DISABLE);
	DCMI_CaptureCmd(DISABLE);
}
/**
 * @brief Initialize/Enable DCMI Interrupt
 */
void dcmi_it_init()
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the DCMI global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DCMI_ITConfig(DCMI_IT_FRAME,ENABLE);
}
void dma_it_init()
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the DMA global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA2_Stream1, DMA_IT_HT, ENABLE); // half transfer interrupt
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE); // transfer complete interrupt
}
/**
 * @brief Interrupt handler of DCMI
 */

void dma_swap_buffers(void)
{
	return;
}
void dcmi_clock_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_ClockSecuritySystemCmd(ENABLE);

	/* Enable GPIOs clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

	/* Configure MCO (PA8) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	#if HSE_VALUE==24000000
	RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);
	#else
	RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_5);
	#endif
}
