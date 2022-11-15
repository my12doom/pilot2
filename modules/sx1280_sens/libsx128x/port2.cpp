#include <stdint.h>
#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/Interface/ISysTimer.h>
#include <stdio.h>

using namespace STM32F4;
static F4SPI spi(SPI1);
#include <stm32F4xx_gpio.h>

#define RADIO_NSS_PIN       GPIO_Pin_4
#define RADIO_NSS_PORT      GPIOA

#define ANT_SW_PIN          GPIO_Pin_0
#define ANT_SW_PORT         GPIOB

#define RADIO_nRESET_PIN    GPIO_Pin_2
#define RADIO_nRESET_PORT   GPIOB

#define RADIO_MOSI_PIN      GPIO_Pin_7
#define RADIO_MOSI_PORT     GPIOA

#define RADIO_MISO_PIN      GPIO_Pin_6
#define RADIO_MISO_PORT     GPIOA

#define RADIO_SCK_PIN       GPIO_Pin_5
#define RADIO_SCK_PORT      GPIOA

#define RADIO_BUSY_PIN      GPIO_Pin_11
#define RADIO_BUSY_PORT     GPIOA

//#define RADIO_DIOx_PIN      GPIO_Pin_1
//#define RADIO_DIOx_PORT     GPIOA

//#define USART_TX_PIN        GPIO_Pin_2
//#define USART_TX_PORT       GPIOA

//#define USART_RX_PIN        GPIO_Pin_3
//#define USART_RX_PORT       GPIOA

//#define LED_RX_PIN          GPIO_Pin_0
//#define LED_RX_PORT         GPIOC

//#define LED_TX_PIN          GPIO_Pin_1
//#define LED_TX_PORT         GPIOC

#define RADIO_DIO1_Pin      GPIO_Pin_1
#define RADIO_DIO1_GPIO_Port    GPIOA


extern "C" 
{

F4GPIO out_pins[] = 
{
	F4GPIO(RADIO_NSS_PORT, RADIO_NSS_PIN),
	F4GPIO(ANT_SW_PORT, ANT_SW_PIN),
	F4GPIO(RADIO_nRESET_PORT, RADIO_nRESET_PIN),
};

F4GPIO input_pins[] = 
{
	F4GPIO(RADIO_BUSY_PORT, RADIO_BUSY_PIN),
	F4GPIO(RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin),
};


#include <string.h>
uint8_t txtmp[40] = {0};
uint8_t rxtmp[40] = {0};
int32_t _SpiWrite( void* ctx, uint8_t *prefix, uint16_t prefix_len, uint8_t* out, uint16_t out_len )
{
	out_pins[0].write(0);
	memcpy(txtmp, prefix, prefix_len);
	memcpy(txtmp+prefix_len, out, out_len);
	spi.txrx2(txtmp, rxtmp, prefix_len+out_len);
	out_pins[0].write(1);
	printf("(%d)OUT:\n", int(systimer->gettime()));
	for(int i=0; i<prefix_len+out_len; i++)
		printf("%02x, ", txtmp[i]);
	printf("\n");
	return 0;
}
int32_t _SpiRead( void* ctx, uint8_t *prefix, uint16_t prefix_len, uint8_t* in, uint16_t in_len )
{
	out_pins[0].write(0);
	memset(txtmp, 0, prefix_len+in_len);
	memset(rxtmp, 0, sizeof(rxtmp));
	memcpy(txtmp, prefix, prefix_len);
	spi.txrx2(txtmp, rxtmp, prefix_len+in_len);
	out_pins[0].write(1);
	memcpy(in, rxtmp+prefix_len, in_len);

	printf("(%d)IN:\n", int(systimer->gettime()));
	for(int i=0; i<prefix_len+in_len; i++)
		printf("%02x, ", txtmp[i]);
	printf("\n");
	for(int i=0; i<prefix_len+in_len; i++)
		printf("%02x, ", rxtmp[i]);
	printf("\n");
	
	return 0;
}

int32_t set_reset_pin( void* ctx, bool value )
{
	out_pins[2].write(value);
	return 0;
}

int32_t reset_pin_set( void* ctx, bool value )
{
	out_pins[2].write(value);
	return 0;
}

int32_t get_busy( void* ctx )
{
	return input_pins[0].read();
}

int32_t get_dio( void* ctx )
{
	return input_pins[1].read();
}

void _DelayMs( void* ctx, uint32_t ms )
{
	systimer->delayms(ms);
}

#include "lib/sx1280.h"
SX1280_t *me;
void inter_cb(void *parameter, int flags)
{
	printf("(%d)IRQ\n", int(systimer->gettime()));
	SX1280OnDioIrq(me);
	printf("(%d)IRQ done\n", int(systimer->gettime()));
}

void SX1280HalIoInit(SX1280_t *sx1280)
{
	me = sx1280;
	spi.set_mode(0,0);
	spi.set_speed(1000000);
	
	
	
	for(int i=0; i<sizeof(out_pins)/sizeof(out_pins[0]); i++)
		out_pins[i].set_mode(HAL::MODE_OUT_PushPull);

	for(int i=0; i<sizeof(input_pins)/sizeof(input_pins[0]); i++)
		input_pins[i].set_mode(HAL::MODE_IN);
	
	F4Interrupt * p = new F4Interrupt;
	p->init(RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin, HAL::interrupt_rising);
	p->set_callback(inter_cb, (void*)0);
	
	sx1280->ctx = sx1280;
	sx1280->delay_ms = _DelayMs;
	sx1280->set_reset = set_reset_pin;
	sx1280->get_busy = get_busy;
	sx1280->spi_write = _SpiWrite;
	sx1280->spi_read = _SpiRead;
	
	sx1280->get_dio[0] = get_dio;
	sx1280->get_dio[1] = get_dio;
	sx1280->get_dio[2] = get_dio;
	sx1280->get_dio[3] = get_dio;
}

}