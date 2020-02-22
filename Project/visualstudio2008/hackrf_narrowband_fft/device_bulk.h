#pragma once

#include "device.h"
#include <windows.h>
#include <stdint.h>

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
    int16_t rx_data[65536];
    int rx_data_size;

    static DWORD WINAPI rx_worker_entry(LPVOID p){return ((bulk_device*)p)->rx_worker();}
    static DWORD WINAPI usb_worker_entry(LPVOID p){return ((bulk_device*)p)->usb_worker();}
    DWORD rx_worker();
    DWORD usb_worker();
};

}
