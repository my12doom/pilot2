#pragma once

#include "device.h"
#include "autolock.h"

namespace NBFFT
{

class fx3_device: public device
{
public:
	fx3_device(){thread = INVALID_HANDLE_VALUE;}
	~fx3_device(){destroy();};
	virtual int init(data_callback cb);
	virtual int destroy();
	virtual int config(){bootloader_command = true; return 0;}
	virtual int tune(int64_t hz){return -1;}
	int get_sample_rate(){return 100e6;}
	virtual sample_type get_sample_type(){return complex_sample;}
	virtual int set_gains(uint8_t *gains){return 0;}
	virtual int get_gains(uint8_t *gains){return 0;}
	virtual int get_gains_count(){ return 1;}

protected:
	int fpga_configure(void * device);
	bool exiting;
	bool bootloader_command;
	HANDLE thread;
	_critical_section cs;
	data_callback cb;
	static DWORD WINAPI usb_worker_entry(LPVOID p){return ((fx3_device*)p)->usb_worker();}
	DWORD usb_worker();
};

}