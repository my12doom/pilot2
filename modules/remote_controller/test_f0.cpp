#include <stdlib.h>
#include <HAL/Interface/ISysTimer.h>
#include <HAL/STM32F0/F0GPIO.h>
#include <HAL/STM32F0/F0SPI.h>
#include <HAL/STM32F0/F0Timer.h>
#include <HAL/STM32F0/F0Interrupt.h>
#include <utils/AES.h>
#include <utils/RIJNDAEL.h>
#include <utils/space.h>
#include <HAL/aux_devices/NRF24L01.h>

using namespace STM32F0;
using namespace HAL;
using namespace devices;

int dt;
AESCryptor2 aes;
uint8_t key[32] = "12345678";
uint8_t plain[16] = "HelloWorld";
uint8_t cipher[16];
uint8_t decrypted[16];
int test_AES()
{
	int64_t t = systimer->gettime();
	aes.decrypt(cipher, decrypted);
	aes.decrypt(cipher, decrypted);
	dt = systimer->gettime() - t;
	
	return 0;
}

F0GPIO led(GPIOB, GPIO_Pin_11);
F0GPIO A6(GPIOA, GPIO_Pin_6);
F0GPIO cs(GPIOB, GPIO_Pin_12);
F0GPIO ce(GPIOB, GPIO_Pin_3);
F0GPIO irq(GPIOA, GPIO_Pin_15);
F0GPIO dbg(GPIOB, GPIO_Pin_10);

void timer_cb(void *p)
{
	A6.write(true);
	A6.write(false);
	//led.toggle();
}

void int_cb(void *p, int flag)
{
	
}

F0Interrupt intr;

int  main()
{
	//FLASH_SetLatency(FLASH_Latency_0);
	space_init();
	aes.set_key(key, 256);
	aes.encrypt(plain, cipher);
	F0Timer timer(TIM2);
	led.write(true);
	led.set_mode(MODE_OUT_OpenDrain);
	A6.set_mode(MODE_OUT_PushPull);

	//timer.set_callback(timer_cb, NULL);
	timer.set_period(2000);
	intr.init(GPIOA, GPIO_Pin_7, HAL::interrupt_rising);
	intr.set_callback(int_cb, NULL);
	
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_ClockSecuritySystemCmd(ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_MCOConfig(RCC_MCOSource_SYSCLK);
	
	F0SPI spi(SPI2);
	systimer->delayus(16);
	
	F0Interrupt interrupt;
	interrupt.init(GPIOA, GPIO_Pin_15, interrupt_falling);
	
	NRF24L01 nrf;
	int c;
	dbg.write(true);
	dbg.set_mode(MODE_OUT_OpenDrain);

	while( (c = nrf.init(&spi, &cs, &ce)) != 0)
	{
		dbg.write(false);
		systimer->delayms(100);
		dbg.write(true);
		systimer->delayms(100);
	}

	while(1)
	{
		led.toggle();
		systimer->delayus(100);
	}
}
