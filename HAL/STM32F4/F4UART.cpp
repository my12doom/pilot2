#include "F4UART.h"
#include <stdint.h>
#include "stm32f4xx.h"
namespace STM32F4
{
	F4UART::F4UART(USART_TypeDef * USARTx):start(0),end(0),tx_start(0),tx_end(0),ongoing_tx_size(0),dma_running(0),end_sentence(0)
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
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
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
			USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
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
			// NVIC config
			NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
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
		}
		else if(USART3 == USARTx )
		{
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
			//Set Uart4 bind to GPIOD pin8|pin9
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOD, &GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
			GPIO_Init(GPIOD, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
			GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
			// NVIC config
			NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);		
			
			USART_InitStructure.USART_BaudRate = 115200;
			USART_InitStructure.USART_WordLength = USART_WordLength_8b;
			USART_InitStructure.USART_StopBits = USART_StopBits_1;
			USART_InitStructure.USART_Parity = USART_Parity_No ;
			USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
			USART_Init(USART3, &USART_InitStructure); 
			USART_Cmd(USART3, ENABLE);
			USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
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
			// NVIC config
			NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);		
			
			USART_InitStructure.USART_BaudRate = 115200;
			USART_InitStructure.USART_WordLength = USART_WordLength_8b;
			USART_InitStructure.USART_StopBits = USART_StopBits_1;
			USART_InitStructure.USART_Parity = USART_Parity_No ;
			USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
			USART_Init(UART4, &USART_InitStructure); 
			USART_Cmd(UART4, ENABLE);
			USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
			
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
		
		dma_init();

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
		USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
		return 1;
	}
	void F4UART::dma_init()
	{
		if(USART1 == USARTx)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
			this->DMA_Channel=DMA_Channel_4;
			this->DMA_Stream_IRQ=DMA2_Stream7_IRQn;
			this->DMAy_Streamx=DMA2_Stream7;
			this->RCC_AHB1Periph=RCC_AHB1Periph_DMA2;
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
			this->DMA_Channel=DMA_Channel_4;
			this->DMA_Stream_IRQ=DMA1_Stream6_IRQn;
			this->DMAy_Streamx=DMA1_Stream6;
			this->RCC_AHB1Periph=RCC_AHB1Periph_DMA1;
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
			this->DMA_Channel=DMA_Channel_4;
			this->DMA_Stream_IRQ=DMA1_Stream3_IRQn;
			this->DMAy_Streamx=DMA1_Stream3;
			this->RCC_AHB1Periph=RCC_AHB1Periph_DMA1;
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
			this->DMA_Channel=DMA_Channel_4;
			this->DMA_Stream_IRQ=DMA1_Stream4_IRQn;
			this->DMAy_Streamx=DMA1_Stream4;
			this->RCC_AHB1Periph=RCC_AHB1Periph_DMA1;
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
		
		DMA_DeInit(DMAy_Streamx);  
		DMA_InitStructure.DMA_Channel = DMA_Channel; 
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
		DMA_Init(DMAy_Streamx, &DMA_InitStructure);
		DMA_ITConfig(DMAy_Streamx,DMA_IT_TC,ENABLE);
		USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE); 
	}
	int F4UART::write(const void *buf, int count)
	{
		int i;
		const char *p = (const char*)buf;
		int new_pos = (tx_end +count) % sizeof(tx_buffer);
		int old_size = tx_end-tx_start < 0 ? tx_end-tx_start + sizeof(tx_buffer) : tx_end-tx_start;
		if (count + old_size > sizeof(tx_buffer))
		{
			dma_handle_queue();
			return 0;		// reject all data if buffer overrun
		}

		for(i=0; i<count; i++)
			tx_buffer[(tx_end+i)%sizeof(tx_buffer)] = p[i];

		tx_end = (tx_end+count)%sizeof(tx_buffer);

		count = tx_end-tx_start;
		if (count<0)
		count += sizeof(tx_buffer);

		dma_handle_queue();

		return count;
	}
	int F4UART::read(void *data, int max_count)
	{
		char *p = (char*)data;
		int _end_sentence = end_sentence;
		int j=0;
		int i;
		int size;
		int lastR = 0;
		if (_end_sentence == start)
			return -1;
		size = _end_sentence - start;
		if (size<0)
			size += sizeof(buffer);
		if (size >= max_count)
			return -2;
		for(i=start; i!= _end_sentence; i=(i+1)%sizeof(buffer))
		{
			if (buffer[i] == '\r')
			{
				if (lastR)
					p[j++] = buffer[i];
				lastR = !lastR;
			}

			p[j++] = buffer[i];
			if (buffer[i] == '\n')
			{
				i=(i+1)%sizeof(buffer);
				break;
			}
		}
		p[j] = 0;
		start = i;
		return j;
	}
	int F4UART::peak(void *data, int max_count)
	{
		char *p = (char*)data;
		int _end_sentence = end_sentence;
		int j=0;
		int i;
		int size;
		int lastR = 0;
		if (_end_sentence == start)
			return -1;
		size = _end_sentence - start;
		if (size<0)
			size += sizeof(buffer);
		if (size >= max_count)
			return -2;
		for(i=start; i!= _end_sentence; i=(i+1)%sizeof(buffer))
		{
			if (buffer[i] == '\r')
			{
				if (lastR)
					p[j++] = buffer[i];
				lastR = !lastR;
			}

			p[j++] = buffer[i];
			if (buffer[i] == '\n')
			{
				i=(i+1)%sizeof(buffer);
				break;
			}
		}
		p[j] = 0;
		return j;
	}
	int F4UART::dma_handle_queue()
	{
		
		if (dma_running)
			return 0;

		DMA_Cmd(DMAy_Streamx, DISABLE);

		ongoing_tx_size = tx_end - tx_start;
		if (ongoing_tx_size == 0)
			return 0;
		if (ongoing_tx_size < 0)
			ongoing_tx_size = sizeof(tx_buffer) - tx_start;
		
		DMAy_Streamx->NDTR = ongoing_tx_size;
		DMAy_Streamx->M0AR = (uint32_t)tx_buffer + tx_start;

		DMA_Cmd(DMAy_Streamx, ENABLE);

		dma_running = 1;

		//ERROR("!!%d!!", ongoing_tx_size);

		return 0;
	}
	
	//just for uart4:
	void F4UART::DMA1_Steam4_IRQHandler()
	{	
		tx_start = (tx_start + ongoing_tx_size) % sizeof(tx_buffer);
		DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
		dma_running = 0;
		dma_handle_queue();
	}
	void F4UART::UART4_IRQHandler(void)
	{
		int c = -1;
		if(USART_GetFlagStatus(UART4,USART_IT_RXNE)==SET || USART_GetFlagStatus(UART4,USART_IT_ORE_RX)==SET)
		{
			c = USART_ReceiveData(UART4);
		}
		
		//USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		UART4->SR = (uint16_t)~0x20;
		if (c>0)
		{
			buffer[end] = c;
			end++;
			end %= sizeof(buffer);
			if (c == '\n')
				end_sentence = end;

		}
	}
	//For usart1:
	void F4UART::DMA2_Steam7_IRQHandler()
	{	
		tx_start = (tx_start + ongoing_tx_size) % sizeof(tx_buffer);
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
		dma_running = 0;
		dma_handle_queue();
	}
	void F4UART::USART1_IRQHandler(void)
	{
		int c = -1;
		if(USART_GetFlagStatus(USART1,USART_IT_RXNE)==SET || USART_GetFlagStatus(USART1,USART_IT_ORE_RX)==SET)
		{
			c = USART_ReceiveData(USART1);
		}
		
		//USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		USART1->SR = (uint16_t)~0x20;
		if (c>0)
		{
			buffer[end] = c;
			end++;
			end %= sizeof(buffer);
			if (c == '\n')
				end_sentence = end;

		}
	}
	//just for usart3:
	void F4UART::DMA1_Steam3_IRQHandler()
	{	
		tx_start = (tx_start + ongoing_tx_size) % sizeof(tx_buffer);
		DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);//? 3|4?
		dma_running = 0;
		dma_handle_queue();
	}
	void F4UART::USART3_IRQHandler(void)
	{
		int c = -1;
		if(USART_GetFlagStatus(USART3,USART_IT_RXNE)==SET || USART_GetFlagStatus(USART3,USART_IT_ORE_RX)==SET)
		{
			c = USART_ReceiveData(USART3);
		}
		
		//USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		USART3->SR = (uint16_t)~0x20;
		if (c>0)
		{
			buffer[end] = c;
			end++;
			end %= sizeof(buffer);
			if (c == '\n')
				end_sentence = end;

		}
	}
	//just for usart2:
	void F4UART::DMA1_Steam6_IRQHandler()
	{	
		tx_start = (tx_start + ongoing_tx_size) % sizeof(tx_buffer);
		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);//? 3|4?
		dma_running = 0;
		dma_handle_queue();
	}
	void F4UART::USART2_IRQHandler(void)
	{
		int c = -1;
		if(USART_GetFlagStatus(USART2,USART_IT_RXNE)==SET || USART_GetFlagStatus(USART2,USART_IT_ORE_RX)==SET)
		{
			c = USART_ReceiveData(USART2);
		}
		
		//USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		USART3->SR = (uint16_t)~0x20;
		if (c>0)
		{
			buffer[end] = c;
			end++;
			end %= sizeof(buffer);
			if (c == '\n')
				end_sentence = end;

		}
	}
}