#include <stdlib.h>
#include <HAL/Interface/ISysTimer.h>
#include <HAL/STM32F0/F0GPIO.h>
#include <HAL/STM32F0/F0SPI.h>
#include <HAL/STM32F0/F0Timer.h>
#include <HAL/STM32F0/F0Interrupt.h>
#include <utils/AES.h>
#include <utils/RIJNDAEL.h>
#include <utils/space.h>

using namespace STM32F0;
using namespace HAL;

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
void timer_cb(void *p)
{
	A6.write(true);
	A6.write(false);
}

void int_cb(void *p, int flag)
{
	led.toggle();
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

	timer.set_callback(timer_cb, NULL);
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
	
	/*
	//F0SPI spi(SPI1);
	spi.set_speed(25000000);
	spi.set_mode(0,0);
	uint8_t tx[30];
	for(int i=0; i<30; i++)
		tx[i] = i;
		
	uint8_t rx[30];
	spi.txrx2(tx, rx, 30);
	led.toggle();
	systimer->delayus(16);
	*/
	
	while(1)
	{
		//led.write(false);
		test_AES();//systimer->delayus(150);
		//led.write(true);
		test_AES();//systimer->delayus(150);
	}
}
