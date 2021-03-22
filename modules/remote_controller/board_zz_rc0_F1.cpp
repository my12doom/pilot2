#include "board.h"

#include <HAL/STM32F1/F1SPI.h>
#include <HAL/STM32F1/F1GPIO.h>
#include <HAL/STM32F1/F1Interrupt.h>
#include <HAL/STM32F1/F1SysTimer.h>
#include <misc.h>
#include <HAL/STM32F1/F1Timer.h>
#include <HAL/STM32F1/F1UART.h>
#include <stm32f10x.h>
#include <string.h>

#include "PPMOUT_f1.h"

using namespace STM32F1;
using namespace HAL;

int16_t adc_data[6] = {0};

F1GPIO _cs(GPIOA, GPIO_Pin_8);
F1GPIO _ce(GPIOA, GPIO_Pin_9);
F1GPIO _irq(GPIOA, GPIO_Pin_10);

F1GPIO _dbg(GPIOA, GPIO_Pin_4);
F1GPIO _dbg2(GPIOA, GPIO_Pin_5);

F1SPI _spi;
F1Interrupt _interrupt;
F1Timer _timer(TIM2);


STM32F1::F1UART f1uart(USART2);	

	
extern "C" void TIM2_IRQHandler(void)
{
	_timer.call_callback();
}

int board_init()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);		
	
	cs = &_cs;
	ce = &_ce;
	irq = &_irq;
	dbg = &_dbg;
	dbg2 = &_dbg2;
	SCL = NULL;
	SDA = NULL;
	spi = &_spi;
	interrupt = &_interrupt;
	timer = &_timer;
	sbus = &f1uart;
	
	//	
	_spi.init(SPI2);
	_interrupt.init(GPIOA, GPIO_Pin_10, interrupt_falling);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	static PPMOUT ppmout;
	ppm = &ppmout;
	
	f1uart.set_baudrate(115200);
		
	return 0;
}

static uint16_t le2be_uint16(uint16_t in)
{
    uint8_t msb = in >> 8;
    uint8_t lsb = in & 0xff;
    return msb | (lsb<<8);
}

void custom_output(uint8_t * payload, int payload_size, int latency)
{
	return;
	
	static int64_t last_output = 0;
	if (systimer->gettime() - last_output < 14000 || latency > 200000)
		return;
	last_output = systimer->gettime();
	
	uint8_t key = payload[12];
	
    struct
    {
        uint8_t fade;
        uint8_t system;
        uint16_t channels[7];
    } dsm_packet = {0};
    dsm_packet.fade = 0;
    dsm_packet.system = 0xB2;   // DSMX 11ms 2048
	
	// first 5 channel: analog
	//uint16_t values[7];// throttle, aileron, elevator, rudder, gear
	uint16_t * values = (uint16_t*)payload;
	dsm_packet.channels[0] = le2be_uint16((values[2]>>1) | (0<<11));
	dsm_packet.channels[1] = le2be_uint16((values[0]>>1) | (1<<11));
	dsm_packet.channels[2] = le2be_uint16((values[1]>>1) | (2<<11));
	dsm_packet.channels[3] = le2be_uint16((values[3]>>1) | (3<<11));
	dsm_packet.channels[4] = le2be_uint16((values[4]>>1) | (4<<11));
	
	// 6th channel: combination of buttons	
	dsm_packet.channels[5] = le2be_uint16(key | (5<<11));
	
    // 7th channel, 0xffff for compatability
    dsm_packet.channels[6] = 0xffff;
	
	f1uart.write(&dsm_packet, sizeof(dsm_packet));
}
