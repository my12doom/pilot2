#pragma once
#include <HAL/Interface/IUART.h>
#include "stm32F4xx_usart.h"

#define TX_BUFFER_SIZE 200
#define RX_BUFFER_SIZE 1024

namespace STM32F4
{
	class F4UART:public HAL::IUART
	{
	public:
		F4UART(USART_TypeDef * USARTx);
		~F4UART(){};

		virtual int set_baudrate(int baudrate);
		virtual int write(const void *data, int count);
		virtual int flush();
		virtual int read(void *data, int max_count);
		virtual int peak(void *data, int max_count);
		virtual int readline(void *data, int max_count);
		virtual int available();		
			
		void USART1_IRQHandler(void);
		void DMA2_Steam7_IRQHandler();
			
		void USART2_IRQHandler(void);
		void DMA1_Steam6_IRQHandler();

		void USART3_IRQHandler(void);
		void DMA1_Steam3_IRQHandler();
			
		void UART4_IRQHandler(void);
		void DMA1_Steam4_IRQHandler();
		
		void destroy();

	private:
		USART_TypeDef * USARTx;
		int baudrate;

		// TX DMA var:
		DMA_Stream_TypeDef* tx_DMAy_Streamx;
		volatile bool tx_dma_running;
		int tx_start;
		int tx_end;
		int ongoing_tx_size;

		// RX buffer:
		char rx_buffer[RX_BUFFER_SIZE];		// circular buffer
		int start;
		int end;
		int ongoing_rx_start;
		int ongoing_rx_size;
		volatile bool rx_dma_running;

		char tx_buffer[TX_BUFFER_SIZE];

		void dma_tx_init();
		int dma_handle_tx_queue();

		void dma_rx_init();
		int dma_handle_rx_queue();
	};
}
