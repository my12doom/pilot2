#pragma once
#include <HAL/Interface/IUART.h>
#include "stm32F4xx_usart.h"

#define TX_BUFFER_SIZE 200
#define RX_BUFFER_SIZE 200

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
		
		// tx dma
		void DMA2_Stream7_IRQHandler();	// uart1tx
		void DMA1_Stream6_IRQHandler();	// uart2tx
		void DMA1_Stream3_IRQHandler();	// uart3tx
		void DMA1_Stream4_IRQHandler();	// uart4tx

		// rx dma
		void DMA2_Stream2_IRQHandler();	// uart1rx
		void DMA1_Stream5_IRQHandler();	// uart2rx
		void DMA1_Stream1_IRQHandler();	// uart3rx
		void DMA1_Stream2_IRQHandler();	// uart4rx

		
		void destroy();

	private:
		USART_TypeDef * USARTx;
		int baudrate;

		// TX var:
		DMA_Stream_TypeDef* tx_DMAy_Streamx;
		char tx_buffer[TX_BUFFER_SIZE];
		volatile bool tx_dma_running;
		int tx_start;
		int tx_end;
		int ongoing_tx_size;
		void dma_tx_init();
		int dma_handle_tx_queue();

		// RX:
		DMA_Stream_TypeDef* rx_DMAy_Streamx;
		char rx_buffer[RX_BUFFER_SIZE];		// circular buffer
		int start;
		int end;
		int ongoing_rx_start;
		int ongoing_rx_size;
		volatile bool rx_dma_running;
		void dma_rx_init();
		int dma_handle_rx_queue();
	};
}
