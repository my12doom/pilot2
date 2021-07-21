#pragma once
#include <HAL/Interface/IUART.h>
#include <stm32F10x_usart.h>
#include <stm32F10x_dma.h>

#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 1024

namespace STM32F1
{
	class F1UART:public HAL::IUART
	{
	public:
		F1UART(USART_TypeDef * USARTx);
		~F1UART(){};

		virtual int set_baudrate(int baudrate);
		virtual int write(const void *data, int count);
		virtual int flush();
		virtual int read(void *data, int max_count);
		virtual int peak(void *data, int max_count);
		virtual int readline(void *data, int max_count);
		virtual int available();		
			
		void RXIRQHandler();
		void TXIRQHandler();
			
		void destroy();

	private:
		USART_TypeDef *USARTx;
		int baudrate;

		// TX DMA var:
		DMA_Channel_TypeDef* tx_DMAy_Streamx;
		uint32_t tc_flag; 
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
	};
}
