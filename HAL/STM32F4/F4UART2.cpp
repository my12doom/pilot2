#include "F4UART2.h"
#include <stdint.h>
#include "stm32f4xx.h"
#include <Protocol/common.h>

namespace STM32F4
{
	static F4UART2 * uart_table[6] = {0};

	// UART1
	extern "C" void USART1_IRQHandler(void)
	{
		if (uart_table[0])
			uart_table[0]->uart_irq();
	}
	extern "C" void DMA2_Stream7_IRQHandler()
	{
		if (uart_table[0])
			uart_table[0]->tx_dma_irq();
	}
	extern "C" void DMA2_Stream2_IRQHandler()	// uart1rx
	{
		if (uart_table[0])
			uart_table[0]->rx_dma_irq();
	}

	// UART2
	extern "C" void USART2_IRQHandler(void)
	{
		if (uart_table[1])
			uart_table[1]->uart_irq();
	}
	extern "C" void DMA1_Stream6_IRQHandler()
	{
		if (uart_table[1])
			uart_table[1]->tx_dma_irq();
	}
	extern "C" void DMA1_Stream5_IRQHandler()	// uart2rx
	{
		if (uart_table[1])
			uart_table[1]->rx_dma_irq();
	}
		
	F4UART2::F4UART2(USART_TypeDef * USARTx):start(0),end(0),tx_start(0),tx_end(0),ongoing_tx_size(0),tx_dma_running(false)
	{		
		this->USARTx=USARTx;
		
		GPIO_InitTypeDef GPIO_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		//choose which uart will be inital:
		if(USART1 == USARTx)
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
			//Set Uart4 bind to GPIOA pin9|pin10
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
			// NVIC config
			NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
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
			USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

			if (uart_table[0])
				printf("overwriting UART1\n");
			uart_table[0] = this;
		}
		
		if(USART2 == USARTx)
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			
			//USART2: GPIOA pin2|pin3
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

			// NVIC config
			NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);		
			
			USART_InitStructure.USART_BaudRate = 115200;
			USART_InitStructure.USART_WordLength = USART_WordLength_8b;
			USART_InitStructure.USART_StopBits = USART_StopBits_1;
			USART_InitStructure.USART_Parity = USART_Parity_No ;
			USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
			USART_Init(USART2, &USART_InitStructure); 
			USART_Cmd(USART2, ENABLE);
			USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

			if (uart_table[1])
				printf("overwriting UART2\n");
			uart_table[1] = this;
		}
		dma_init();
	}
	int F4UART2::set_baudrate(int baudrate)
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
		return 1;
	}
	void F4UART2::dma_init()
	{
		uint32_t tx_DMA_Channel;
		uint32_t rx_DMA_Channel;
		uint8_t tx_dma_irqn;
		uint8_t rx_dma_irqn;

		if(USART1 == USARTx)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
			tx_DMA_Channel = DMA_Channel_4;
			tx_dma_irqn = DMA2_Stream7_IRQn;
			tx_DMAy_Streamx = DMA2_Stream7;
			tx_tcif_flag = DMA_FLAG_TCIF7;

			rx_DMA_Channel = DMA_Channel_4;
			rx_dma_irqn = DMA2_Stream2_IRQn;
			rx_DMAy_Streamx = DMA2_Stream2;
			rx_tcif_flag = DMA_FLAG_TCIF2;
		}
		if(USART2 == USARTx)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			tx_DMA_Channel = DMA_Channel_4;
			tx_dma_irqn = DMA1_Stream6_IRQn;
			tx_DMAy_Streamx = DMA1_Stream6;
			tx_tcif_flag = DMA_FLAG_TCIF6;

			rx_DMA_Channel = DMA_Channel_4;
			rx_dma_irqn = DMA1_Stream5_IRQn;
			rx_DMAy_Streamx = DMA1_Stream5;
			rx_tcif_flag = DMA_FLAG_TCIF5;
		}

		// nvic for both tx & rx.
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = tx_dma_irqn;  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		NVIC_InitStructure.NVIC_IRQChannel = rx_dma_irqn;  
		NVIC_Init(&NVIC_InitStructure);

		// DMA configuration for tx.
		DMA_InitTypeDef DMA_InitStructure = {0};
		DMA_DeInit(tx_DMAy_Streamx);
		DMA_InitStructure.DMA_Channel = tx_DMA_Channel;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USARTx->DR));
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&tx_buffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
		DMA_InitStructure.DMA_BufferSize = sizeof(tx_buffer);
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
		DMA_Init(tx_DMAy_Streamx, &DMA_InitStructure);
		DMA_ITConfig(tx_DMAy_Streamx,DMA_IT_TC,ENABLE);

		// DMA for rx.
		DMA_DeInit(rx_DMAy_Streamx);  
		DMA_InitStructure.DMA_Channel = rx_DMA_Channel; 
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USARTx->DR));
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&rx_dma_buffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = sizeof(rx_dma_buffer);
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
		DMA_Init(rx_DMAy_Streamx, &DMA_InitStructure);
		DMA_ITConfig(rx_DMAy_Streamx,DMA_IT_TC,ENABLE);
		
		// start rx dma at once
		DMA_Cmd(rx_DMAy_Streamx, ENABLE);

		// UART DMA
		USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
		USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
	}
	int F4UART2::write(const void *buf, int count)
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
	int F4UART2::available()
	{
		int _end = end;
		int size = _end - start;
		if (size<0)
			size += sizeof(rx_buffer);
		return size;
	}
	int F4UART2::read(void *data, int max_count)
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
	int F4UART2::readline(void *data, int max_count)
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
	int F4UART2::peak(void *data, int max_count)
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
	int F4UART2::dma_handle_tx_queue()
	{
		if (tx_dma_running)
			return 0;

		DMA_Cmd(tx_DMAy_Streamx, DISABLE);

		ongoing_tx_size = tx_end - tx_start;
		if (ongoing_tx_size == 0)
			return 0;
		if (ongoing_tx_size < 0)
			ongoing_tx_size = sizeof(tx_buffer) - tx_start;	// never cross the end of tx buffer
		
		tx_DMAy_Streamx->NDTR = ongoing_tx_size;
		tx_DMAy_Streamx->M0AR = (uint32_t)tx_buffer + tx_start;

		DMA_Cmd(tx_DMAy_Streamx, ENABLE);

		tx_dma_running = true;

		//ERROR("!!%d!!", ongoing_tx_size);

		return 0;
	}
	
	int F4UART2::flush()
	{
		while(tx_dma_running)
			;
		
		return 0;
	}
	
	void F4UART2::uart_irq(void)
	{
		if(USARTx->SR & USART_FLAG_ORE)
		{
			volatile int c = USART1->DR;		// strange way to clear RXNE/ORE/IDLE FLAG
			printf("ORE\n");
		}
		if(USARTx->SR & USART_FLAG_IDLE)
		{
			//printf("IDLE\n");
			volatile int c = USART1->DR;		// strange way to clear RXNE/ORE/IDLE FLAG
			rx_dma_reset();
		}
	}
	
	void F4UART2::tx_dma_irq()
	{	
		tx_start = (tx_start + ongoing_tx_size) % sizeof(tx_buffer);
		DMA_ClearFlag(tx_DMAy_Streamx, tx_tcif_flag);
		tx_dma_running = false;
		dma_handle_tx_queue();
	}

	void F4UART2::rx_dma_irq(void)
	{
		DMA_ClearFlag(rx_DMAy_Streamx, rx_tcif_flag);
		//printf("rx dma\n");
		rx_dma_reset();
	}

	int F4UART2::rx_dma_reset()
	{
		DMA_Cmd(rx_DMAy_Streamx, DISABLE);
		
		int n = sizeof(rx_dma_buffer) - rx_DMAy_Streamx->NDTR;
		//printf("dma:%d bytes\n", n);
		int m = start - end;
		if (m>0)
			m--;
		else
			m = end-start+sizeof(rx_buffer)-1;
		if (n>m)
			n = m;
		
		for(int i=0; i<n; i++)
			rx_buffer[(end+i) & (sizeof(rx_buffer)-1)] = rx_dma_buffer[i];
		
		end = (end+n) & (sizeof(rx_buffer)-1);
		
		rx_DMAy_Streamx->NDTR = sizeof(rx_dma_buffer);
		rx_DMAy_Streamx->M0AR = (uint32_t)rx_dma_buffer;

		DMA_Cmd(rx_DMAy_Streamx, ENABLE);
		
		return 0;
	}

	void F4UART2::destroy()
	{
		USART_Cmd(USARTx, DISABLE);
		USART_ITConfig(USARTx, USART_IT_RXNE, DISABLE);
		DMA_Cmd(tx_DMAy_Streamx, DISABLE);
	}
}
