#include "F0UART.h"
#include <stdint.h>
#include <Protocol/common.h>
#include <stm32F0xx_gpio.h>
#include <stm32F0xx_rcc.h>
#include <stm32F0xx_misc.h>

namespace STM32F0
{
	static F0UART * uart_table[1] = {0};

	// UART1
	extern "C" void USART1_IRQHandler(void)
	{
		if (uart_table[0])
			uart_table[0]->usart_irq();
	}
	extern "C" void DMA1_Channel2_3_IRQHandler()
	{
		if (uart_table[0])
			uart_table[0]->dma_irq();
	}
	
	F0UART::F0UART(USART_TypeDef * USARTx):start(0),end(0),tx_start(0),tx_end(0),ongoing_tx_size(0),tx_dma_running(false),rx_dma_running(false),ongoing_rx_start(0),ongoing_rx_size(0)
	{		
		this->USARTx=USARTx;		
		GPIO_InitTypeDef GPIO_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		
		//choose which uart to initialize:
		if(USART1 == USARTx)
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
			
			
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
			
			// NVIC config
			NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			
			
			USART_InitStructure.USART_BaudRate = 115200;
			USART_InitStructure.USART_WordLength = USART_WordLength_8b;
			USART_InitStructure.USART_StopBits = USART_StopBits_1;
			USART_InitStructure.USART_Parity = USART_Parity_No ;
			USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
			USART_Init(USART1, &USART_InitStructure); 
			USART_Cmd(USART1, ENABLE);
			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

			uart_table[0] = this;
		}
		
		dma_tx_init();
	}
	int F0UART::set_baudrate(int baudrate)
	{
		this->baudrate=baudrate;
		USART_InitTypeDef USART_InitStructure;
		//USART config
		USART_InitStructure.USART_BaudRate = baudrate;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USARTx, &USART_InitStructure); 
		USART_Cmd(USARTx, ENABLE);
		USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
		return 1;
	}
	void F0UART::dma_tx_init()
	{
		if(USART1 == USARTx)
		{
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
			tx_DMAy_Streamx=DMA1_Channel2;
			
			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_3_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		
		DMA_InitTypeDef DMA_InitStructure = {0};
		DMA_DeInit(DMA1_Channel2);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USARTx->TDR));
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&tx_buffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
		DMA_InitStructure.DMA_BufferSize = sizeof(tx_buffer);
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		
		DMA_Init(tx_DMAy_Streamx, &DMA_InitStructure);
		
		DMA_ITConfig(tx_DMAy_Streamx,DMA_IT_TC,ENABLE);
		USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
	}
	
	int F0UART::write(const void *buf, int count)
	{
		int i;
		const char *p = (const char*)buf;
		int new_pos = (tx_end +count) % sizeof(tx_buffer);
		int old_size = tx_end-tx_start < 0 ? tx_end-tx_start + sizeof(tx_buffer) : tx_end-tx_start;
		if (count + old_size > sizeof(tx_buffer))
		{
			dma_handle_tx_queue();
			return 0;		// reject all data if buffer overrun
		}

		for(i=0; i<count; i++)
			tx_buffer[(tx_end+i)%sizeof(tx_buffer)] = p[i];

		tx_end = (tx_end+count)%sizeof(tx_buffer);

		count = tx_end-tx_start;
		if (count<0)
		count += sizeof(tx_buffer);

		dma_handle_tx_queue();

		return count;
	}
	int F0UART::available()
	{
		int _end = end;
		int size = _end - start;
		if (size<0)
			size += sizeof(rx_buffer);
		return size;
	}
	int F0UART::read(void *data, int max_count)
	{
		char *p = (char*)data;
		int _end = end;
		int j=0;
		int i;
		int size;
		int lastR = 0;
		if (_end == start)
			return -1;
		size = _end - start;
		if (size<0)
			size += sizeof(rx_buffer);
		if (max_count > size)
			max_count = size;
		for(i=0; i<max_count; i++)
			p[i] = rx_buffer[(i+start)%sizeof(rx_buffer)];
		start = (i+start)%sizeof(rx_buffer);
		return max_count;
	}
	int F0UART::readline(void *data, int max_count)
	{
		char *p = (char*)data;
		int _end = end;
		int j=0;
		int i;
		int size;
		int lastR = 0;
		if (_end == start)
			return -1;
		size = _end - start;
		if (size<0)
			size += sizeof(rx_buffer);
		if (max_count > size)
			max_count = size;
		for(i=0; i<max_count; i++)
		{
			p[i] = rx_buffer[(i+start)%sizeof(rx_buffer)];
			if (p[i] == '\n')
			{
				i++;
				start = (i+start)%sizeof(rx_buffer);
				return i;
			}
		}
		return -2;
	}
	int F0UART::peak(void *data, int max_count)
	{
		char *p = (char*)data;
		int _end = end;
		int j=0;
		int i;
		int size;
		int lastR = 0;
		if (_end == start)
			return 0;
		size = _end - start;
		if (size<0)
			size += sizeof(rx_buffer);
		if (max_count > size)
			max_count = size;
		for(i=0; i<max_count; i++)
			p[i] = rx_buffer[(i+start)%sizeof(rx_buffer)];
		return max_count;
	}
	int F0UART::dma_handle_tx_queue()
	{
		if (tx_dma_running)
			return 0;

		DMA_Cmd(tx_DMAy_Streamx, DISABLE);

		ongoing_tx_size = tx_end - tx_start;
		if (ongoing_tx_size == 0)
			return 0;
		if (ongoing_tx_size < 0)
			ongoing_tx_size = sizeof(tx_buffer) - tx_start;	// never cross the end of tx buffer
		
		tx_DMAy_Streamx->CNDTR = ongoing_tx_size;
		tx_DMAy_Streamx->CMAR = (uint32_t)tx_buffer + tx_start;

		DMA_Cmd(tx_DMAy_Streamx, ENABLE);

		tx_dma_running = true;

		//ERROR("!!%d!!", ongoing_tx_size);

		return 0;
	}
	
	int F0UART::flush()
	{
		while(tx_dma_running)
			;
		
		return 0;
	}
	

	//For usart1:
	void F0UART::dma_irq()
	{	
		tx_start = (tx_start + ongoing_tx_size) % sizeof(tx_buffer);
		DMA_ClearFlag(DMA1_FLAG_TC2);
		tx_dma_running = false;
		dma_handle_tx_queue();
	}
	void F0UART::usart_irq(void)
	{
		int c = -1;
		if(USART_GetFlagStatus(USARTx,USART_FLAG_RXNE)==SET)
			c = USART_ReceiveData(USARTx);
		
		USARTx->ICR = USART_FLAG_ORE;
		
		if ((end+1)&(sizeof(rx_buffer)-1) != start)
		{
			rx_buffer[end] = c;
			end++;
			end = sizeof(rx_buffer)-1;
		}
	}
	
	void F0UART::destroy()
	{
		USART_Cmd(USARTx, DISABLE);
		USART_ITConfig(USARTx, USART_IT_RXNE, DISABLE);
		DMA_Cmd(tx_DMAy_Streamx, DISABLE);
	}
}
