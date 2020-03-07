#pragma once

#include "device.h"
#include <windows.h>
#include <stdint.h>
#include "libusb.h"

namespace NBFFT
{

class bulk_device: public device
{
public:
    bulk_device(){usb_event = INVALID_HANDLE_VALUE;}
    ~bulk_device(){destroy();};
    virtual int init(data_callback cb);
    virtual int destroy();
    virtual int config(){return 0;}
    int get_sample_rate(){return 5120000;}

protected:
    data_callback cb;
    HANDLE usb_thread;
    HANDLE rx_thread;
    HANDLE usb_event;
    bool working;
    void *usb_ctx;
    int16_t rx_data[256*1024];
    int rx_data_size;

    static DWORD WINAPI rx_worker_entry(LPVOID p){return ((bulk_device*)p)->rx_worker();}
    static DWORD WINAPI usb_worker_entry(LPVOID p){return ((bulk_device*)p)->usb_worker();}
	static void LIBUSB_CALL libusb_transfer_callback_entry(struct libusb_transfer* usb_transfer){((bulk_device*)usb_transfer->user_data)->libusb_transfer_callback(usb_transfer);}
    DWORD rx_worker();
    DWORD usb_worker();
	void libusb_transfer_callback(struct libusb_transfer* usb_transfer);
	int handle_data(uint8_t *data, int actual_length);
};

}
