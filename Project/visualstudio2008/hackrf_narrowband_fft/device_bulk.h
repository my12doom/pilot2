#pragma once

#include "device.h"
#include <windows.h>
#include <stdint.h>
#include "libusb.h"
#include <F4SDR/F4SDR.h>

namespace NBFFT
{

class bulk_device: public device
{
public:
    bulk_device(){usb_event = INVALID_HANDLE_VALUE;InitializeCriticalSection(&cs_usb);}
    ~bulk_device(){destroy();DeleteCriticalSection(&cs_usb);};
    virtual int init(data_callback cb);
	virtual int init(sweep_callback cb, sweep_config config);
	bool support_sweep(){return true;}
    virtual int destroy();
    virtual int show_config_dialog();
	virtual int tune(int64_t hz);
    int get_sample_rate(){return 10e6;}
    virtual sample_type get_sample_type(){return complex_sample;}
	virtual int set_gains(uint8_t *gains);
	virtual int get_gains(uint8_t *gains);
	virtual int get_gains_count(){ return 1;}

	int control_io(bool tx, uint8_t request, uint16_t value, uint16_t index, uint8_t *data, uint16_t length);

protected:
    data_callback cb;
	sweep_callback sweep_cb;
	sweep_config _sweep_config;
	CRITICAL_SECTION cs_usb;
    HANDLE usb_thread;
    HANDLE rx_thread;
    HANDLE usb_event;
    bool working;
    void *usb_ctx;
    int16_t rx_data[256*1024];
    int rx_data_size;

	libusb_device_handle* dev_handle;

    static DWORD WINAPI rx_worker_entry(LPVOID p){return ((bulk_device*)p)->rx_worker();}
    static DWORD WINAPI usb_worker_entry(LPVOID p){return ((bulk_device*)p)->usb_worker();}
	static void LIBUSB_CALL libusb_transfer_callback_entry(struct libusb_transfer* usb_transfer){((bulk_device*)usb_transfer->user_data)->libusb_transfer_callback(usb_transfer);}
    DWORD rx_worker();
    DWORD usb_worker();
	void libusb_transfer_callback(struct libusb_transfer* usb_transfer);
	int handle_data(uint8_t *data, int actual_length);
};

}
