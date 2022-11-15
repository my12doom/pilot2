#pragma once

#include <math.h>
#include <stdint.h>


namespace NBFFT
{

enum sample_type
{
    complex_sample,
    real_sample,
};

enum sample_quant
{
    sample_8bit = 0,
    sample_16bit = 1,
    sample_float = 2,
};

typedef int (*data_callback)(void *buf, int len);

class device
{
public:
    virtual ~device(){}
    virtual int init(data_callback cb) = 0;
    virtual int destroy() = 0;
    virtual int config() = 0;
	virtual int tune(int64_t hz){return -1;}
    virtual int get_sample_rate() = 0;
	virtual int set_gains(uint8_t *gains){return -1;}
	virtual int get_gains(uint8_t *gains){return -1;}
	virtual int get_gains_count(){ return 0;}
    virtual sample_type get_sample_type(){return complex_sample;}
	virtual sample_quant get_sample_quant(){return sample_16bit;}
    virtual int dynamic_range_db(){return 130;}     // return max noise density SNR in db
};

}
