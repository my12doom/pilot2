#include "F4HSBulk.h"
extern "C"
{
#include "HAL/STM32F4/usbHSBulk/usbd_cdc_core.h"
#include "HAL/STM32F4/usbHSBulk/usbd_usr.h"
#include "HAL/STM32F4/usbHSBulk/usb_conf.h"
#include "HAL/STM32F4/usbHSBulk/usbd_desc.h"
}

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

static STM32F4::F4HSBulk *ptr = NULL;


namespace STM32F4
{
	
F4HSBulk::F4HSBulk()
{
}

int F4HSBulk::ioctl(int op, void *p, int size)
{	
	if (op == set_callback)
	{
		if (!ptr)
		{
			ptr = this;
			cb = (F4HSBulk_event_cb)p;
			USBD_Init(&USB_OTG_dev, USB_OTG_HS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
		}
	}
	
	else if (op == tx_block)
	{
		tx_buf = p;
		DCD_EP_Tx(&USB_OTG_dev, CDC_IN_EP, (uint8_t*)p, size);
	}
	
	else if (op == rx_block)
	{
		rx_buf = p;
		DCD_EP_PrepareRx(&USB_OTG_dev, CDC_OUT_EP, (uint8_t*)p, size);
	}

	else if (op == tx_ctrl_block)
	{
		if (size > 0)
			USBD_CtlSendData (&USB_OTG_dev, (uint8_t*)p, size);
		else
			USBD_CtlReceiveStatus(&USB_OTG_dev);

		return 0;
	}
	
	return HAL::error_unsupported;
}

void F4HSBulk::destroy()
{
	USBD_DeInit(&USB_OTG_dev);
}


int F4HSBulk::F4cb(int event, void *p, int size)
{
	if (cb)
	{
		if (event == rx_done)
			p = rx_buf;
		if (event == tx_done)
			p = tx_buf;
		
		return cb(this, event, p, size);
	}
	
	return -1;
}

}

extern "C" int F4cb(int event, void *p, int size)
{
	if (ptr)
		return ptr->F4cb(event, p, size);
	
	return -1;
}
