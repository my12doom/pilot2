#pragma once

#include "device.h"
#include <windows.h>
#include "hackrf.h"

namespace NBFFT
{

class hackrf_device: public device
{
public:
	hackrf_device(){cb=0;_device = NULL;center_frequency = 2413.0E6;}
	~hackrf_device(){destroy();}
	virtual int init(data_callback cb);
	virtual int destroy();
	virtual int config(){return 0;}
	int tune(int64_t hz);


	int get_sample_rate(){return 20000000;}
	sample_quant get_sample_quant(){return sample_8bit;}
	int dynamic_range_db(){return 20*log10(256.0*6.6/0.05);}    // return max noise density SNR in db, assuming hackrf has ~0.05LSB rms noise

protected:
	int start_rx();
	int stop_rx();
	data_callback cb;
	bool started;
	bool sweep;
	void * _device;
	bool watchdog_run;
	DWORD last_rx_time;
	HANDLE h_watchdog;
	int64_t center_frequency;// = 2413.0E6;
	static DWORD WINAPI watchdog_entry(LPVOID p){return ((hackrf_device*)p)->watchdog();}
	static int rx_callback_entry(hackrf_transfer* transfer){return ((hackrf_device*)transfer->rx_ctx)->rx_callback(transfer);}
	int rx_callback(hackrf_transfer* transfer);
	DWORD watchdog();
};

}
