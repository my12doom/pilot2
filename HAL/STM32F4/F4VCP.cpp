#include "F4VCP.h"
#include <stdint.h>
extern "C"
{
	#include "usb_comF4/cdc/usbd_cdc_vcp.h"
	#include <HAL/STM32F4/usb_comF4/cdc/usbd_cdc_core.h>
	#include <HAL/STM32F4/usb_comF4/core/usbd_usr.h>
	#include <HAL/STM32F4/usb_comF4/usb_conf/usbd_desc.h>
	#include <HAL/STM32F4/usb_comF4/usb_conf/usb_conf.h>

	#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
	#if defined ( __ICCARM__ ) //!< IAR Compiler 
	#pragma data_alignment=4
	#endif
	#endif // USB_OTG_HS_INTERNAL_DMA_ENABLED

	__ALIGN_BEGIN USB_OTG_CORE_HANDLE     USB_OTG_dev  __ALIGN_END ;
}
namespace STM32F4
{
	F4VCP::F4VCP()
	{
		USBD_Init(&USB_OTG_dev,
			USB_OTG_FS_CORE_ID,
			&USR_desc,
			&USBD_CDC_cb,
			&USR_cb);
		
		*(uint32_t*)0x50000804 &= ~(0x02);	// clear USB FS device soft disconnect bit
	}
	
	int F4VCP::set_baudrate(int baudrate)
	{
		return 0;
	}
	
	int F4VCP::available()
	{
		return VCP_available();;
	}
	
	int F4VCP::read(void *data, int max_count)
	{
		return VCP_get_data((uint8_t*)data, max_count);
	}
	
	int F4VCP::write(const void *data, int count)
	{
		VCP_DataTx((uint8_t*)data, count);
		
		return count;
	}
	
	int F4VCP::readline(void *data, int max_count)
	{
		return VCP_get_line((uint8_t*)data, max_count);
	}
	
	int F4VCP::peak(void *data, int max_count)
	{
		return VCP_peak((uint8_t*)data, max_count);
	}
	
	int F4VCP::flush()
	{
		return 0;
	}
	
	void F4VCP::destroy()
	{
	}
}
