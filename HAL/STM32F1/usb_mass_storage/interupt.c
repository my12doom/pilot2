#include "usb_lib.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "../fat/sdcard.h"
#include "../mcu.h"
#ifdef STM32F10X_CL
void OTG_FS_IRQHandler(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
  STM32_PCD_OTG_ISR_Handler(); 
}
#endif

int sss = 0;
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	sss++;
	if (sss % 2 < 1)
		GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	else
		GPIO_SetBits(GPIOA, GPIO_Pin_8);
  USB_Istr();
}

void USB_HP_CAN1_TX_IRQHandler(void)
{
	//GPIO_ResetBits(GPIOA, GPIO_Pin_8);
  CTR_HP();
}
