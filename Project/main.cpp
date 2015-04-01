#include <BSP/Resources.h>
#include <stdio.h>
#include <Algorithm/ahrs.h>
#include <utils/log.h>

using namespace devices;

int main(void)
{	
	bsp_init_all();
	char a[20];
	while(1)
	{
		//manager.getUART("UART4")->write("12345\n",6);
		manager.getUART("UART3")->write("12345\n",6);
		//manager.getUART("UART3")->read((char *) a,5);
	}
}


struct __FILE { int handle; /* Add whatever you need here */ };
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

extern "C" int fputc(int ch, FILE *f)
{
	if (DEMCR & TRCENA) 
	{
		while (ITM_Port32(0) == 0);
		ITM_Port8(0) = ch;
	}
	return (ch);
}

