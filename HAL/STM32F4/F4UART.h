#pragma once
#include "IUART.h"
#include "stm32F4xx_usart.h"

namespace STM32F4
{
	class F4UART:public HAL::IUART
	{
	#define TX_BUFFER_SIZE 1024
	#define RX_BUFFER_SIZE 1500
	private:
		USART_TypeDef * USARTx;
		int baudrate;
		//DMA var:
		uint32_t DMA_Channel;
		uint8_t DMA_Stream_IRQ;
		DMA_Stream_TypeDef* DMAy_Streamx;
		uint32_t RCC_AHB1Periph;
		//USART buffer var:
		int start;
		int end;
		int tx_start;
		int tx_end;
		int ongoing_tx_size;
		int dma_running; 
		int end_sentence;
		char tx_buffer[TX_BUFFER_SIZE];
		char buffer[RX_BUFFER_SIZE];		// circular buffer
	public:
		F4UART(USART_TypeDef * USARTx);
		~F4UART(){};
		virtual int set_baudrate(int baudrate);
		virtual int write(const void *data, int count);
		virtual int read(void *data, int max_count);
		virtual int peak(void *data, int max_count);
		virtual int readline(void *data, int max_count);
		virtual int available();
			
		virtual void dma_init();
		virtual int dma_handle_queue();
			
		virtual void UART4_IRQHandler(void);
		virtual void DMA1_Steam4_IRQHandler();
			
		virtual void USART1_IRQHandler(void);
		virtual void DMA2_Steam7_IRQHandler();
			
		virtual void USART3_IRQHandler(void);
		virtual void DMA1_Steam3_IRQHandler();
			
		virtual void USART2_IRQHandler(void);
		virtual void DMA1_Steam6_IRQHandler();
		
		void destroy();
	};
}
