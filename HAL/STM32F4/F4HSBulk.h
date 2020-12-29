#pragma once

#include <stdint.h>

enum F4HSBulk_Cmd
{
	set_callback = 0,	// set event callback and start usb enumeration
	tx_block = 1,		// transfer a new block, must be called after a tx_ready event
	rx_block = 2,		// queue a RX block buffer, must be called after a rx_ready event
	tx_ctrl_block = 3,	// queue a ctrl tx block, must be called after a control_IN event.
};

enum F4HSBulk_Event
{
	tx_ready = 0,	// ready to transfer new block
	tx_done = 1,	// block sent
	rx_ready = 2,	// ready to queue new block buffer
	rx_done = 3,	// new block arrived
	control_IN = 4, // control requested
	control_OUT = 5, // control requested
};

typedef struct
{
	uint8_t request_dir: 1;	// true: IN request, false : OUT request
	uint8_t request_type : 7;
	uint8_t request;
	uint16_t value;
	uint16_t index;
	uint16_t length;
	uint8_t *data;
} control_verndor_transfer;

#ifdef __cplusplus
#include <HAL/Interface/IBlockDevice.h>
namespace STM32F4
{
	class F4HSBulk;
	typedef int (*F4HSBulk_event_cb)(F4HSBulk *h, int event, void *p, int size);

	class F4HSBulk:public HAL::IBlockDevice
	{
	public:
		F4HSBulk();
		~F4HSBulk(){destroy();}

		// no conventional interface support due to memory performance issue
		virtual int write(const void *buf, int block_size){return HAL::error_unsupported;}
		virtual int read(void *buf, int max_block_size, bool remove = true){return HAL::error_unsupported;}
		virtual int available(){return HAL::error_unsupported;}

		// use ioctl cmds and callbacks to do transfers
		virtual int ioctl(int op, void *p, int size);

		void destroy();

		int F4cb(int event, void *p, int size);
		F4HSBulk_event_cb cb;
	protected:
		void *tx_buf;
		void *rx_buf;
	};
}
extern "C"{
#endif

int F4cb(int event, void *p, int size);

#ifdef __cplusplus
}
#endif
