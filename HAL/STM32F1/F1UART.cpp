#include "F1UART.h"
#include <stdint.h>
#include <Protocol/common.h>
#include <stm32F10x_gpio.h>
#include <stm32F10x_rcc.h>
#include <misc.h>

namespace STM32F1
{
	static F1UART * uart_table[6] = {0};

	extern "C" void USART1_IRQHandler(void)
	{
		if (uart_table[0])
			uart_table[0]->RXIRQHandler();
	}
	extern "C" void USART2_IRQHandler(void)
	{
		if (uart_table[1])
			uart_table[1]->RXIRQHandler();
	}
	extern "C" void USART3_IRQHandler(void)
	{
		if (uart_table[2])
			uart_table[2]->RXIRQHandler();
	}
	extern "C" void DMA1_Channel4_IRQHandler()
	{
		if (uart_table[0])
			uart_table[0]->TXIRQHandler();
	}
	extern "C" void DMA1_Channel7_IRQHandler()
	{
		if (uart_table[1])
			uart_table[1]->TXIRQHandler();
	}
	extern "C" void DMA1_Channel2_IRQHandler()
	{
		if (uart_table[2])
			uart_table[2]->TXIRQHandler();
	}
	
	F1UART::F1UART(USART_TypeDef * USARTx):start(0),end(0),tx_start(0),tx_end(0),ongoing_tx_size(0),tx_dma_running(false),rx_dma_running(false),ongoing_rx_start(0),ongoing_rx_size(0)
	{		
		this->USARTx=USARTx;		
		GPIO_InitTypeDef GPIO_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		
		//choose which uart to initialize:
		if(USART1 == USARTx)
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
			
			
			//Set Uart1 bind to GPIOA pin9|pin10
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO_Init(GPIOA, &GPIO_InitStructure);			
			
			// NVIC config
			NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);			

			uart_table[0] = this;
		}
		
		if(USART2 == USARTx)
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
			
			
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;			
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO_Init(GPIOA, &GPIO_InitStructure);			
			
			// NVIC config
			NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

			uart_table[1] = this;
		}
		
		if(USART3 == USARTx)
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
			
			
			//Set Uart1 bind to GPIOA pin9|pin10
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO_Init(GPIOB, &GPIO_InitStructure);			
			
			// NVIC config
			NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

			uart_table[2] = this;
		}
		
		set_baudrate(115200);
		dma_tx_init();
	}
	int F1UART::set_baudrate(int baudrate)
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
	void F1UART::dma_tx_init()
	{
		if(USART1 == USARTx)
		{
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
			tx_DMAy_Streamx=DMA1_Channel4;
			tc_flag = DMA1_FLAG_TC4;
			
			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		if(USART2 == USARTx)
		{
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
			tx_DMAy_Streamx=DMA1_Channel7;
			tc_flag = DMA1_FLAG_TC7;
			
			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		if(USART3 == USARTx)
		{
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
			tx_DMAy_Streamx=DMA1_Channel2;
			tc_flag = DMA1_FLAG_TC2;
			
			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		
		DMA_InitTypeDef DMA_InitStructure = {0};
		DMA_DeInit(tx_DMAy_Streamx);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USARTx->DR));
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
		
		DMA_ITConfig(tx_DMAy_Streamx, DMA_IT_TC, ENABLE);
		USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
	}
	
	int F1UART::write(const void *buf, int count)
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
	int F1UART::available()
	{
		int _end = end;
		int size = _end - start;
		if (size<0)
			size += sizeof(rx_buffer);
		return size;
	}
	int F1UART::read(void *data, int max_count)
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
	int F1UART::readline(void *data, int max_count)
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
	int F1UART::peak(void *data, int max_count)
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
	int F1UART::dma_handle_tx_queue()
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
	
	int F1UART::flush()
	{
		while(tx_dma_running)
			;
		
		return 0;
	}
	

	//For usart1:
	void F1UART::TXIRQHandler()
	{	
		tx_start = (tx_start + ongoing_tx_size) % sizeof(tx_buffer);
		DMA_ClearFlag(tc_flag);
		tx_dma_running = false;
		dma_handle_tx_queue();
	}
	void F1UART::RXIRQHandler()
	{
		int c = -1;
		if(USART_GetFlagStatus(USARTx,USART_IT_RXNE)==SET || USART_GetFlagStatus(USARTx,USART_IT_ORE)==SET)
		{
			c = USARTx->SR;
			c = USART_ReceiveData(USARTx);
		}
		
		//USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		USARTx->SR = (uint16_t)~0x20;
		if (((end+1)%sizeof(rx_buffer) != start))
		{
			rx_buffer[end] = c;
			end++;
			end %= sizeof(rx_buffer);
		}
	}
	
	void F1UART::destroy()
	{
		USART_Cmd(USARTx, DISABLE);
		USART_ITConfig(USARTx, USART_IT_RXNE, DISABLE);
		DMA_Cmd(tx_DMAy_Streamx, DISABLE);
	}
}
