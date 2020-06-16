#pragma once
#include <HAL/Interface/IUART.h>
#include <stdlib.h>
#include "stm32F4xx_usart.h"

#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 256
#define RX_DMA_BUFFER_SIZE 128

namespace STM32F4
{
	class F4UART2:public HAL::IUART
	{
	public:
		F4UART2(USART_TypeDef * USARTx, void *tx_buf_override = NULL, int tx_buf_override_size = 0, void *rx_buf_override = NULL, int rx_buf_override_size = 0);
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
			
		int set_buffer_override(void *tx_buf_override = NULL, int tx_buf_override_size = 0, void *rx_buf_override = NULL, int rx_buf_override_size = 0);
			


	private:
		void dma_init();

		USART_TypeDef * USARTx;
		int baudrate;

		// TX DMA var:
		DMA_Stream_TypeDef* tx_DMAy_Streamx;
		IRQn_Type tx_dma_irqn;
		uint32_t tx_tcif_flag;
		volatile bool tx_dma_running;
		int tx_start;
		int tx_end;
		int ongoing_tx_size;
		char _tx_buffer[TX_BUFFER_SIZE];		// default buffer
		char *tx_buffer;						// actural circular buffer, overridable.
		int dma_handle_tx_queue();
		int tx_buffer_size;

		// rx dma
		DMA_Stream_TypeDef* rx_DMAy_Streamx;
		IRQn_Type rx_dma_irqn;
		uint32_t rx_tcif_flag;
		uint32_t rx_htif_flag;
		char rx_dma_buffer[RX_DMA_BUFFER_SIZE];		// circular buffer
		int rx_dma_extract();		// called by USART IDLE irq or DMA end irq, to actually recieve a block and put into user ring buffer,
									// and restart rx DMA again
		int dma_read_ptr;									

		// RX buffer:
		char _rx_buffer[RX_BUFFER_SIZE];		// default circular buffer
		char *rx_buffer;						// actural circular buffer, overridable.
		int rx_buffer_size;
		int start;
		int end;
	};
}
