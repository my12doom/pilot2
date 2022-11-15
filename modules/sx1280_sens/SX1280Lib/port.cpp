#include <stdint.h>
#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/Interface/ISysTimer.h>

using namespace STM32F4;
static F4SPI spi(SPI1);



extern "C" 
{

#include "hw.h"
F4GPIO out_pins[] = 
{
	F4GPIO(RADIO_NSS_PORT, RADIO_NSS_PIN),
	F4GPIO(ANT_SW_PORT, ANT_SW_PIN),
	F4GPIO(RADIO_nRESET_PORT, RADIO_nRESET_PIN),
};

F4GPIO input_pins[] = 
{
	F4GPIO(RADIO_BUSY_PORT, RADIO_BUSY_PIN),
};

void HAL_Delay(uint32_t Delay)
{
	systimer->delayms(Delay);
}

int GpioRead(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin)
{
	return (GPIOx->IDR & GPIO_Pin) ? 1 : 0;
}

void SX1280HalIoInit()
{
	spi.set_mode(0,0);
	spi.set_speed(12000000);
	
	
	for(int i=0; i<sizeof(out_pins)/sizeof(out_pins[0]); i++)
		out_pins[i].set_mode(HAL::MODE_OUT_PushPull);

	for(int i=0; i<sizeof(input_pins)/sizeof(input_pins[0]); i++)
		input_pins[i].set_mode(HAL::MODE_IN);
	
}


void GpioWrite( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,  uint32_t value )
{
	GPIO_WriteBit(GPIOx, GPIO_Pin, (BitAction)value);
}
typedef void( GpioIrqHandler )( void );

void inter_cb(void *parameter, int flags)
{
	GpioIrqHandler *cb = (GpioIrqHandler *)parameter;
	cb();
}

void GpioSetIrq( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t prio,  GpioIrqHandler *irqHandler )
{
	F4Interrupt * p = new F4Interrupt;
	p->init(GPIOx, GPIO_Pin, HAL::interrupt_rising);
	p->set_callback(inter_cb, (void*)irqHandler);
}

void SpiInOut( uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size )
{
	//systimer->delayus(100);
	/*
	for(int i=0; i<size; i++)
	{
		uint8_t v = spi.txrx(txBuffer[i]);
		if (rxBuffer)
			rxBuffer[i] = v;
		systimer->delayus(10);
	}
	*/
	spi.txrx2(txBuffer, rxBuffer, size);
	//systimer->delayus(100);
}


}