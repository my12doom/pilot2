#pragma once
#include <HAL/Interface/IUART.h>
#include "stm32F4xx_usart.h"

#define TX_BUFFER_SIZE 384
#define RX_BUFFER_SIZE 1024
#define RX_DMA_BUFFER_SIZE 512

namespace STM32F4
{
	class F4UART2:public HAL::IUART
	{
	public:
		F4UART2(USART_TypeDef * USARTx);
		~F4UART2(){};

		virtual int set_baudrate(int baudrate);
		virtual int write(const void *data, int count);
		virtual int flush();
		virtual int read(void *data, int max_count);
		virtual int peak(void *data, int max_count);
		virtual int readline(void *data, int max_count);
		virtual int available();
		void destroy();
		
		void uart_irq();
		void tx_dma_irq();
		void rx_dma_irq();
			


	private:
		void dma_init();

		USART_TypeDef * USARTx;
		int baudrate;

		// TX DMA var:
		DMA_Stream_TypeDef* tx_DMAy_Streamx;
		uint32_t tx_tcif_flag;
		volatile bool tx_dma_running;
		int tx_start;
		int tx_end;
		int ongoing_tx_size;
		char tx_buffer[TX_BUFFER_SIZE];
		int dma_handle_tx_queue();

		// rx dma
		DMA_Stream_TypeDef* rx_DMAy_Streamx;
		uint32_t rx_tcif_flag;
		char rx_dma_buffer[RX_DMA_BUFFER_SIZE];		// circular buffer
		int rx_dma_reset();			// called by USART IDLE irq or DMA end irq, to actually recieve a block and put into user ring buffer,
									// and restart rx DMA again

		// RX buffer:
		char rx_buffer[RX_BUFFER_SIZE];		// circular buffer
		int start;
		int end;
	};
}
