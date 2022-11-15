#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4SysTimer.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4HSBulk.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4Timer.h>
#include <HAL/STM32F4/F4ADC.h>
#include <HAL/Interface/II2C.h>
#include <HAL/aux_devices/OLED_I2C.h>
#include <Protocol/common.h>

extern "C"
{
	#include "myRadio_gpio.h"
}


using namespace STM32F4;
using namespace HAL;

F4Interrupt io1_int;

F4GPIO pa_en(GPIOB, GPIO_Pin_7);
F4GPIO lna_en(GPIOA, GPIO_Pin_15);
F4GPIO ctr(GPIOB, GPIO_Pin_3);
F4GPIO csA(GPIOB, GPIO_Pin_9);

F4GPIO io1(GPIOC, GPIO_Pin_13);
F4GPIO io2(GPIOB, GPIO_Pin_14);

F4SPI spi(SPI1, true);

RADIO_GPIO_CALLBACK gpioCallback;

F4GPIO bio1(GPIOB, GPIO_Pin_5);

extern "C"
{

void io1_irq(void *parameter, int flags)
{
	if (gpioCallback)
		gpioCallback(1);
}
void myRadio_gpio_init(RADIO_GPIO_CALLBACK cb)
{
	gpioCallback = cb;

	bio1.set_mode(HAL::MODE_OUT_PushPull);
	ctr.set_mode(HAL::MODE_OUT_PushPull);
	pa_en.set_mode(HAL::MODE_OUT_PushPull);
	lna_en.set_mode(HAL::MODE_OUT_PushPull);
	csA.set_mode(HAL::MODE_OUT_PushPull);
	csA.write(1);

	spi.set_mode(0, 0);
	spi.set_speed(12000000);
	
	ctr.write(1);
	lna_en.write(0);
	pa_en.write(0);
	
	io1.set_mode(HAL::MODE_IN);
	
	io1_int.init(GPIOC, GPIO_Pin_13, HAL::interrupt_falling);
	io1_int.set_callback(io1_irq, NULL);
	
	uint8_t tmp[128] = {0};
	while(0)
		spi.txrx2(tmp, tmp+64, 64);
}
uint8_t READ_RF_A5133_GPIO1(void)
{
	return io1.read();
}

void RF_A5133_GPIO1_H(void)
{
	io1.write(1);
}
void RF_A5133_GPIO1_L(void)
{
	io1.write(0);
}

void BOARD_A5133_SCS_H(void)
{
	csA.write(1);
}
void BOARD_A5133_SCS_L(void)
{
	csA.write(0);
}

void RF_EXT_PA_TO_TX()
{
	pa_en.write(1);
	ctr.write(0);
	lna_en.write(0);	
}
void RF_EXT_PA_TO_RX()
{
	pa_en.write(0);
	ctr.write(1);
	lna_en.write(1);	
}
void RF_EXT_PA_TO_IDLE()
{
	pa_en.write(0);
	ctr.write(0);
	lna_en.write(0);	
}

void tst()
{
	bio1.write(1);
	systimer->delayus(1);
	bio1.write(0);
}


uint8_t myRadioSpi_rByte(void)
{
	return spi.txrx(0);
}
void myRadioSpi_wByte(uint8_t byteToWrite)
{
	spi.txrx(byteToWrite);
}

void myRadioSpi_trx(uint8_t *tx, uint8_t *rx, int count)
{
	spi.txrx2(tx, rx, count);
}

}
