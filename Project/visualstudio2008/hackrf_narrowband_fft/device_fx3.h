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
	virtual int config();
	virtual int tune(int64_t hz){return -1;}
	int get_sample_rate(){return 400e6;}
	virtual sample_type get_sample_type(){return real_sample;}
	virtual int set_gains(uint8_t *gains){return 0;}
	virtual int get_gains(uint8_t *gains){return 0;}
	virtual int get_gains_count(){ return 1;}
	virtual sample_quant get_sample_quant(){return sample_8bit;}

	enum REG_BANKS
	{
		BANK_FPGA = 0,
		BANK_ADC = 1,
	};
	int reg_read(uint8_t bank, uint16_t add);
	int reg_write(uint8_t bank, uint16_t add, uint8_t value);

protected:
	int fpga_configure(void * device);
	bool exiting;
	HANDLE thread;
	_critical_section cs_device;
	void *fx3device;
	data_callback cb;
	static DWORD WINAPI usb_worker_entry(LPVOID p){return ((fx3_device*)p)->usb_worker();}
	DWORD usb_worker();
};

}