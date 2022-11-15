#include "board.h"

#include <HAL/STM32F1/F1SPI.h>
#include <HAL/STM32F1/F1GPIO.h>
#include <HAL/STM32F1/F1Interrupt.h>
#include <HAL/STM32F1/F1Timer.h>
#include <HAL/STM32F1/F1VCP.h>
#include <HAL/STM32F1/F1UART.h>
#include <HAL/Interface/ISysTimer.h>

using namespace STM32F1;
using namespace HAL;
F1VCP vcp;
F1UART uart(USART1);
F1GPIO led(GPIOC, GPIO_Pin_13);

int main()
{
	led.set_mode(HAL::MODE_OUT_PushPull);
	uart.set_baudrate(115200);
	
	char tmp[100];
	while(1)
	{
		led.write(systimer->gettime() % 1000000 < 500000);
		
		int c = uart.read(tmp, sizeof(tmp));
		if (c>0)
			vcp.write(tmp, c);
		c = vcp.read(tmp, sizeof(tmp));
		if (c>0)
			uart.write(tmp, c);
		watchdog_reset();
	}
}
