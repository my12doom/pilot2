#pragma once
#include <HAL/Interface/ISPI.h>
#include <stm32f10x_spi.h>

namespace STM32F1
{
	class F1SPI: public HAL::ISPI
	{
	public:
		F1SPI();
		F1SPI(SPI_TypeDef* SPIx);
	    ~F1SPI(){};
		int init(SPI_TypeDef* SPIx);
			
		virtual int init();
		virtual int set_speed(int speed);	// speed in hz
		virtual int set_mode(int CPOL, int CPHA);// CPOL: 0 = Idle Low, 1 = Idle High; CPHA: 0 = capture at first edge, 1 = capture at second edge
		virtual	uint8_t txrx(uint8_t data);
	protected:
		SPI_TypeDef* SPIx;
		SPI_InitTypeDef SPI_InitStructure;
	};
}
