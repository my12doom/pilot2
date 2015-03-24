#include "SPI.h"
#include <stm32f4xx_spi.h>

using namespace HAL;
namespace STM32F4
{
	class F4SPI: public SPI
	{
	public:
		F4SPI();
		F4SPI(SPI_TypeDef* SPIx);
	    ~F4SPI(){};
		int init(SPI_TypeDef* SPIx);
			
		virtual int init();
		virtual int set_speed(int speed);	// speed in hz
		virtual int set_mode(int mode);		// see http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus for mode definition
		virtual	uint8_t txrx(uint8_t data);
	protected:
		SPI_TypeDef* SPIx;
		SPI_InitTypeDef SPI_InitStructure;
	};
}
