#include "device.h"
#include <windows.h>
#include <stdint.h>
#include "tinymt32.h"

namespace NBFFT
{

class dummy_device: public device
{
public:
    dummy_device(){};
    ~dummy_device(){destroy();};
    virtual int init(data_callback cb);
    virtual int destroy();
    virtual int config(){return 0;}
    int get_sample_rate(){return 10000000;}
    sample_type get_sample_type(){return real_sample;}

protected:
    data_callback cb;
    HANDLE thread;
    bool working;
    float phase[3];
    int16_t data[32768];
    float dataf[32768];
	tinymt32_t tinymt;

    DWORD worker();
    static DWORD WINAPI entry(LPVOID p){return ((dummy_device*)p)->worker();}
};

}
