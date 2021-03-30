#include <stdio.h>

#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4SysTimer.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4HSBulk.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4Timer.h>
#include <HAL/STM32F4/F4ADC.h>
#include <HAL/STM32F4/F4UART2.h>
#include <HAL/STM32F4/F4UART.h>

#include <HAL/aux_devices/LMK04816.h>
#include <HAL/sensors/UartUbloxNMEAGPS.h>

using namespace HAL;
using namespace STM32F4;
using namespace LMK04816;

static F4SPI spi(SPI1);
F4GPIO cs(GPIOA, GPIO_Pin_3);
F4GPIO mcu_on(GPIOB, GPIO_Pin_3);
F4GPIO CP(GPIOC, GPIO_Pin_14);
F4GPIO D(GPIOC, GPIO_Pin_13);
F4GPIO LD(GPIOA, GPIO_Pin_4);
sensors::UartUbloxBinaryGPS gps;

F4Interrupt _int_mcu_button;

void LMK04816_write(uint32_t trx)
{
	cs.write(0);
	
	for(int i=0; i<4; i++)
	{
		uint8_t v = trx >> 24;
		trx <<= 8;
		//printf("%02x,", v);
		spi.txrx(v);
	}
	
	//printf("\n");

	cs.write(1);

}

void LMK04816_write(uint8_t reg, uint32_t regv)
{
	LMK04816_write((reg & 0x1f) | (regv & 0xffffffe0));
}


void button_callback(void *p, int flags)
{
	D.set_mode(HAL::MODE_OUT_PushPull);
	CP.set_mode(HAL::MODE_OUT_PushPull);
	D.write(1);
	CP.write(0);
	CP.write(1);
	CP.write(0);
	CP.set_mode(HAL::MODE_IN);
	D.set_mode(HAL::MODE_IN);

	//LMK04816_write(1, 1<<17);		// power down LMK04816

	mcu_on.write(0);
}

uint32_t LMK04816_read(uint8_t reg)
{
	R31 r31 = {31};
	r31.READBACK_ADDR = reg;
	r31.READBACK_LE = 1;

	uint32_t trx = reg2long(r31);

	cs.write(0);

	spi.txrx(trx>>24);
	spi.txrx(trx>>16);
	spi.txrx(trx>>8);
	spi.txrx(trx>>0);

	cs.write(1);
	
	spi.set_mode(0,1);
	uint32_t o = 0;
	for(int i=0; i<4; i++)
		o = (o<<8) | spi.txrx(0);
	spi.set_mode(0,0);


	return (o & 0xffffffe0) | reg;
}

uint32_t hex_tbl[32] = 
{
0x80160140,
0x80140140,
0x80140141,
0x80140142,
0x401400A3,
0x00140C84,
0x80140145,
0x00000006,
0xCC000007,
0x00CC0008,
0x55555549,
0x9C44410A,
0x0401100B,
0x1B0C006C,
0x2303E26D,
0x0000000E,
0x8000800F,
0xC1550410,
0x00000058,
0x02C9C419,
0xBFA8001A,
0x1000021B,
0x0010191C,
0x018000DD,
0x020000DE,
0x001F001F,
};

uint32_t lmk_regs[32];

F4UART2 uart(USART1);
int main()
{
	D.set_mode(HAL::MODE_OUT_PushPull);
	CP.set_mode(HAL::MODE_OUT_PushPull);
	mcu_on.set_mode(HAL::MODE_OUT_PushPull);
	mcu_on.write(1);
	_int_mcu_button.init(GPIOB, GPIO_Pin_4, interrupt_rising);
	_int_mcu_button.set_callback(button_callback, 0);
	
	D.write(1);
	CP.write(0);
	CP.write(1);
	CP.write(0);
	
	systimer->delayms(200);
	
	D.write(0);
	CP.write(0);
	CP.write(1);
	CP.write(0);
	
	CP.set_mode(HAL::MODE_IN);
	D.set_mode(HAL::MODE_IN);
	LD.set_mode(MODE_IN);
	printf("\n\n start!\n\n");


	gps.init(&uart, 115200);
	//gps.ioctl(sensors::IOCTL_SAT_MINIMUM, NULL);
	uart.set_baudrate(115200);


	spi.set_mode(0, 0);
	spi.set_speed(1000000);
	cs.set_mode(HAL::MODE_OUT_PushPull);
	cs.write(1);

	systimer->delayms(1);
	
	// reset
	LMK04816_write(0, 1<<17);
	systimer->delayms(1);
	
	for(int i=0; i<5; i++)
		LMK04816_write(i, 1<<31);

	LMK04816_write(5, 0);

	LMK04816_write(11, 0<<27);
	//LMK04816_write(R9);

	// Status_CLKin0_MUX as MISO
	R13 r13 = {13};
	r13.STATUS_CLKIN0_MUX = 6;
	r13.STATUS_CLKIN0_TYPE = 3;
	LMK04816_write(reg2long(r13));

	for(int i=0; i<32; i++)
		LMK04816_write(hex_tbl[i]);
	// readback
	for(int i=0; i<32; i++)
	{
		lmk_regs[i] = LMK04816_read(i);
		
		//printf("reg %d = %08x\n", i, lmk_regs[i]);
	}


	while(1)
	{
		char tmp[30];
		int n = uart.read(tmp, sizeof(tmp));
		//for(int i=0; i<n; i++)
		//	fputc(tmp[i], NULL);
		//LMK04816_read(9);
		systimer->delayms(100);

		bool ld = LD.read();

		printf("\rld=%d  ", ld);
	}
}


int log2(const void *packet, uint16_t tag, uint16_t size)
{}
	
#include <stdarg.h>
extern "C"
int log_printf(const char*format, ...)
{
	char buffer[512];
		
	va_list args;
	va_start (args, format);
	int count = vsprintf (buffer,format, args);
	va_end (args);
	
	if (count < 0)
		return count;
	
	printf(buffer);	
}
