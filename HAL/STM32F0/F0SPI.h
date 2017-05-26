#pragma once
#include <HAL/Interface/ISPI.h>
#include <stm32f0xx_spi.h>

namespace STM32F0
{
	class F0SPI: public HAL::ISPI
	{
	public:
		F0SPI();
		F0SPI(SPI_TypeDef* SPIx);
	    ~F0SPI(){};
		int init(SPI_TypeDef* SPIx);
			
		virtual int init();
		virtual int set_speed(int speed);	// speed in hz
		virtual int set_mode(int CPOL, int CPHA);// CPOL: 0 = Idle Low, 1 = Idle High; CPHA: 0 = capture at first edge, 1 = capture at second edge
		virtual	uint8_t txrx(uint8_t data);
		virtual uint8_t txrx2(const uint8_t *tx, uint8_t *rx, int len);
	protected:
		SPI_TypeDef* SPIx;
		SPI_InitTypeDef SPI_InitStructure;
		volatile uint8_t *DR;
	};
}
