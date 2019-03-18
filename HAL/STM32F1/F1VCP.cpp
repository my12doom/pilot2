#include "F1VCP.h"
#include <stdint.h>
extern "C"
{
#include "usb_com/hw_config.h"
#include "usb_com/usb_init.h"
#include "usb_com/usb_lib.h"
}

static STM32F1::F1VCP *that = NULL;

namespace STM32F1
{
	F1VCP::F1VCP()
	{
		Set_USBClock();
		USB_Interrupts_Config();
		USB_Init();
		that = this;
	}
	
	int F1VCP::set_baudrate(int baudrate)
	{
		return 0;
	}
	
	int F1VCP::available()
	{
		return rx_fifo.count();
	}
	
	int F1VCP::read(void *data, int max_count)
	{
		return rx_fifo.pop(data, max_count);
	}
	
	int F1VCP::write(const void *data, int count)
	{		
		return tx_fifo.put(data, count);
	}
	
	int F1VCP::readline(void *data, int max_count)
	{
		return 0;
	}
	
	int F1VCP::peak(void *data, int max_count)
	{
		return rx_fifo.peak(data, max_count);
	}
	
	int F1VCP::flush()
	{
		while(tx_fifo.count())
			;
		return 0;
	}
	
	void F1VCP::destroy()
	{
	}
}


static uint8_t rx_buf[BULK_MAX_PACKET_SIZE] = {1};
static uint8_t tx_buf[BULK_MAX_PACKET_SIZE] = {1};
int rx_buf_count = 0;
int tx_buf_count = 0;

extern "C" void EP3_OUT_Callback(void)
{
	rx_buf_count = USB_SIL_Read(EP3_OUT, rx_buf);
	
	if (that)
	{
		if (that->rx_fifo.available() > rx_buf_count)
		{
			that->rx_fifo.put(rx_buf, rx_buf_count);
			rx_buf_count = 0;
			SetEPRxValid(ENDP3);
		}
	}
}

int i = 0;
extern "C" void EP1_IN_Callback(void)
{
	if (that && GetEPTxStatus(ENDP1) != EP_TX_VALID)
	{
		if (that->tx_fifo.count())
		{
			tx_buf_count = that->tx_fifo.pop(tx_buf, BULK_MAX_PACKET_SIZE);
			USB_SIL_Write(EP1_IN, tx_buf, tx_buf_count);
			SetEPTxValid(ENDP1);
		}
	}
}

extern "C" void SOF_Callback(void)
{
	// handle any pending frame
	if (that && rx_buf_count)
	{
		if (that->rx_fifo.available() > rx_buf_count)
		{
			that->rx_fifo.put(rx_buf, rx_buf_count);
			rx_buf_count = 0;
			SetEPRxValid(ENDP3);
		}
	}
	
	//
	EP1_IN_Callback();
}
