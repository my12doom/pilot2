#include "F4UART.h"
#include <stdint.h>
#include "stm32f4xx.h"
#include <Protocol/common.h>

namespace STM32F4
{
	static F4UART * uart_table[6] = {0};

	// UART1
	extern "C" void DMA2_Stream7_IRQHandler()
	{
		if (uart_table[0])
			uart_table[0]->DMA2_Stream7_IRQHandler();
	}
	extern "C" void DMA2_Stream2_IRQHandler()	// uart1rx
	{
		if (uart_table[0])
			uart_table[0]->DMA2_Stream2_IRQHandler();
	}
	
	// UART2
	extern "C" void DMA1_Stream6_IRQHandler()
	{
		if (uart_table[1])
			uart_table[1]->DMA1_Stream6_IRQHandler();
	}
	extern "C" void DMA1_Stream5_IRQHandler()	// uart2rx
	{
		if (uart_table[1])
			uart_table[1]->DMA1_Stream5_IRQHandler();
	}

	// UART3
	extern "C" void DMA1_Stream3_IRQHandler()
	{
		if (uart_table[2])
			uart_table[2]->DMA1_Stream3_IRQHandler();
	}
	extern "C" void DMA1_Stream1_IRQHandler()	// uart3rx
	{
		if (uart_table[2])
			uart_table[2]->DMA1_Stream1_IRQHandler();
	}

	// UART4
	extern "C" void DMA1_Stream4_IRQHandler()
	{
		if (uart_table[3])
			uart_table[3]->DMA1_Stream4_IRQHandler();
	}
	extern "C" void DMA1_Stream2_IRQHandler()	// uart4rx
	{
		if (uart_table[3])
			uart_table[3]->DMA1_Stream2_IRQHandler();
	}

	F4UART::F4UART(USART_TypeDef * USARTx):start(0),end(0),tx_start(0),tx_end(0),ongoing_tx_size(0),tx_dma_running(false),rx_dma_running(false),ongoing_rx_start(0),ongoing_rx_size(0)
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
			
			USART_InitStructure.USART_BaudRate = 115200;
			USART_InitStructure.USART_WordLength = USART_WordLength_8b;
			USART_InitStructure.USART_StopBits = USART_StopBits_1;
			USART_InitStructure.USART_Parity = USART_Parity_No ;
			USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
			USART_Init(USART1, &USART_InitStructure); 
			USART_Cmd(USART1, ENABLE);

			if (uart_table[0])
				LOGE("overwriting UART1\n");
			uart_table[0] = this;
		}
		else if(USART2 == USARTx )
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
			//Set Uart4 bind to GPIOD pin5|pin6
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOD, &GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
			GPIO_Init(GPIOD, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
			GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
			
			USART_InitStructure.USART_BaudRate = 115200;
			USART_InitStructure.USART_WordLength = USART_WordLength_8b;
			USART_InitStructure.USART_StopBits = USART_StopBits_1;
			USART_InitStructure.USART_Parity = USART_Parity_No ;
			USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
			USART_Init(USART2, &USART_InitStructure); 
			USART_Cmd(USART2, ENABLE);

			if (uart_table[1])
				LOGE("overwriting UART2\n");
			uart_table[1] = this;
		}
		else if(USART3 == USARTx )
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
			//Set Uart4 bind to GPIOD pin8|pin9
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
			GPIO_Init(GPIOB, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
			GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
			
			USART_InitStructure.USART_BaudRate = 115200;
			USART_InitStructure.USART_WordLength = USART_WordLength_8b;
			USART_InitStructure.USART_StopBits = USART_StopBits_1;
			USART_InitStructure.USART_Parity = USART_Parity_No ;
			USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
			USART_Init(USART3, &USART_InitStructure); 
			USART_Cmd(USART3, ENABLE);

			if (uart_table[2])
				LOGE("overwriting UART3\n");
			uart_table[2] = this;
		}
		else if(UART4 == USARTx )	
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			//Set Uart4 bind to GPIOC pin0|pin1
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
			GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
			
			USART_InitStructure.USART_BaudRate = 115200;
			USART_InitStructure.USART_WordLength = USART_WordLength_8b;
			USART_InitStructure.USART_StopBits = USART_StopBits_1;
			USART_InitStructure.USART_Parity = USART_Parity_No ;
			USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
			USART_Init(UART4, &USART_InitStructure); 
			USART_Cmd(UART4, ENABLE);
			
			if (uart_table[3])
				LOGE("overwriting UART4\n");
			uart_table[3] = this;
		}
		else if(UART5 == USARTx )
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
			//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		}
		else if(USART6 == USARTx )
		{
			RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
			//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		}
		else if(UART7 == USARTx )
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);
			//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		}
		else if(UART8 == USARTx )
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8, ENABLE);
			//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		}
		
		dma_tx_init();
		dma_rx_init();

	}
	int F4UART::set_baudrate(int baudrate)
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
	void F4UART::dma_tx_init()
	{
		uint32_t tx_DMA_Channel;
		uint8_t DMA_Stream_IRQ;

		if(USART1 == USARTx)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
			tx_DMA_Channel=DMA_Channel_4;
			DMA_Stream_IRQ=DMA2_Stream7_IRQn;
			this->tx_DMAy_Streamx=DMA2_Stream7;
			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = DMA_Stream_IRQ;  
			//ToDO: change the Priority by using multi-usart:
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(USART2 == USARTx )
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			tx_DMA_Channel=DMA_Channel_4;
			DMA_Stream_IRQ=DMA1_Stream6_IRQn;
			this->tx_DMAy_Streamx=DMA1_Stream6;
			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = DMA_Stream_IRQ;  
			//ToDO: change the Priority by using multi-usart:
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(USART3 == USARTx )
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			tx_DMA_Channel=DMA_Channel_4;
			DMA_Stream_IRQ=DMA1_Stream3_IRQn;
			this->tx_DMAy_Streamx=DMA1_Stream3;
			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = DMA_Stream_IRQ;  
			//ToDO: change the Priority by using multi-usart:
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(UART4 == USARTx )	
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			tx_DMA_Channel=DMA_Channel_4;
			DMA_Stream_IRQ=DMA1_Stream4_IRQn;
			tx_DMAy_Streamx=DMA1_Stream4;
			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = DMA_Stream_IRQ;  
			//ToDO: change the Priority by using multi-usart:
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
		}
		else if(UART5 == USARTx )
		{
		}
		else if(USART6 == USARTx )
		{
		}
		else if(UART7 == USARTx )
		{
		}
		else if(UART8 == USARTx )
		{
		}
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
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
		DMA_Init(tx_DMAy_Streamx, &DMA_InitStructure);
		DMA_ITConfig(tx_DMAy_Streamx,DMA_IT_TC,ENABLE);
		USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE); 
	}



	void F4UART::dma_rx_init()
	{
		uint32_t rx_DMA_Channel;
		uint8_t DMA_Stream_IRQn;

		if(USART1 == USARTx)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
			rx_DMA_Channel=DMA_Channel_4;
			DMA_Stream_IRQn=DMA2_Stream2_IRQn;
			rx_DMAy_Streamx=DMA2_Stream2;
		}
		else if(USART2 == USARTx )
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			rx_DMA_Channel=DMA_Channel_4;
			DMA_Stream_IRQn=DMA1_Stream5_IRQn;
			rx_DMAy_Streamx=DMA1_Stream5;
		}
		else if(USART3 == USARTx )
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			rx_DMA_Channel=DMA_Channel_4;
			DMA_Stream_IRQn=DMA1_Stream1_IRQn;
			rx_DMAy_Streamx=DMA1_Stream1;
		}
		else if(UART4 == USARTx )
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			rx_DMA_Channel=DMA_Channel_4;
			DMA_Stream_IRQn=DMA1_Stream2_IRQn;
			rx_DMAy_Streamx=DMA1_Stream2;
		}
		else if(UART5 == USARTx )
		{
		}
		else if(USART6 == USARTx )
		{
		}
		else if(UART7 == USARTx )
		{
		}
		else if(UART8 == USARTx )
		{
		}

		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = DMA_Stream_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);


		DMA_InitTypeDef DMA_InitStructure = {0};
		DMA_DeInit(rx_DMAy_Streamx);  
		DMA_InitStructure.DMA_Channel = rx_DMA_Channel; 
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USARTx->DR));
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&rx_buffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize = sizeof(rx_buffer);
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
		USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
	}

	int F4UART::write(const void *buf, int count)
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
	int F4UART::available()
	{
		dma_handle_rx_queue();
		int _end = end;
		int size = _end - start;
		if (size<0)
			size += sizeof(rx_buffer);
		return size;
	}
	int F4UART::read(void *data, int max_count)
	{
		dma_handle_rx_queue();
		char *p = (char*)data;
		int _end = end;
		int j=0;
		int i;
		int size;
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
		dma_handle_rx_queue();
		return max_count;
	}
	int F4UART::readline(void *data, int max_count)
	{
		dma_handle_rx_queue();
		char *p = (char*)data;
		int _end = end;
		int j=0;
		int i;
		int size;
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
				dma_handle_rx_queue();
				return i;
			}
		}
		dma_handle_rx_queue();
		return -2;
	}
	int F4UART::peak(void *data, int max_count)
	{
		dma_handle_rx_queue();

		char *p = (char*)data;
		int _end = end;
		int j=0;
		int i;
		int size;
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
	int F4UART::dma_handle_tx_queue()
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

	int F4UART::dma_handle_rx_queue()
	{
		// update valid rx buffer range
		if (rx_dma_running)
		{
			end = ongoing_rx_start + ongoing_rx_size - rx_DMAy_Streamx->NDTR;
		}

		// trigger new rx dma request
		if (!rx_dma_running)
		{
			ongoing_rx_start = end;

			if (start <= end)
				ongoing_rx_size = start == 0 ? (sizeof(rx_buffer) - end - 1) : (sizeof(rx_buffer) - end);
			else
				ongoing_rx_size = start - end - 1;

			if (ongoing_rx_size > 0)
			{
				rx_DMAy_Streamx->NDTR = ongoing_rx_size;
				rx_DMAy_Streamx->M0AR = (uint32_t)rx_buffer + end;

				DMA_Cmd(rx_DMAy_Streamx, ENABLE);
				rx_dma_running = true;
				
				// clear "ORE" flag
				volatile int sr = USART1->SR;
				volatile int dr = USART1->DR;

				//printf("rx dma trigger: @%d, %d bytes\n", ongoing_rx_start, ongoing_rx_size);
			}
		}
		//printf("rx range:[%d,%d)\n", start, end);
	}
	
	int F4UART::flush()
	{
		while(tx_dma_running)
			;
		
		return 0;
	}
	
	// usart1 dma:
	void F4UART::DMA2_Stream7_IRQHandler()
	{	
		tx_start = (tx_start + ongoing_tx_size) % sizeof(tx_buffer);
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
		tx_dma_running = false;
		dma_handle_tx_queue();
	}

	void F4UART::DMA2_Stream2_IRQHandler()
	{
		//printf("usart1 rx dma\n");
		end = (ongoing_rx_start + ongoing_rx_size) % sizeof(rx_buffer);
		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
		rx_dma_running = false;
		dma_handle_rx_queue();
	}


	// usart2 dma:
	void F4UART::DMA1_Stream6_IRQHandler()
	{	
		tx_start = (tx_start + ongoing_tx_size) % sizeof(tx_buffer);
		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
		tx_dma_running = false;
		dma_handle_tx_queue();
	}

	void F4UART::DMA1_Stream5_IRQHandler()
	{	
		end = (ongoing_rx_start + ongoing_rx_size) % sizeof(rx_buffer);
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF6);
		rx_dma_running = false;
		dma_handle_rx_queue();
	}

	// usart3 dma:
	void F4UART::DMA1_Stream3_IRQHandler()
	{	
		tx_start = (tx_start + ongoing_tx_size) % sizeof(tx_buffer);
		DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
		tx_dma_running = false;
		dma_handle_tx_queue();
	}
	void F4UART::DMA1_Stream1_IRQHandler()
	{	
		end = (ongoing_rx_start + ongoing_rx_size) % sizeof(rx_buffer);
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
		rx_dma_running = false;
		dma_handle_rx_queue();
	}

	// uart4 dma:
	void F4UART::DMA1_Stream4_IRQHandler()
	{	
		tx_start = (tx_start + ongoing_tx_size) % sizeof(tx_buffer);
		DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
		tx_dma_running = false;
		dma_handle_tx_queue();
	}

	void F4UART::DMA1_Stream2_IRQHandler()
	{
		end = (ongoing_rx_start + ongoing_rx_size) % sizeof(rx_buffer);
		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
		rx_dma_running = false;
		dma_handle_rx_queue();
	}
	
	void F4UART::destroy()
	{
		USART_Cmd(USARTx, DISABLE);
		USART_ITConfig(USARTx, USART_IT_RXNE, DISABLE);
		DMA_Cmd(tx_DMAy_Streamx, DISABLE);
		DMA_Cmd(rx_DMAy_Streamx, DISABLE);
	}
}