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

enum device_error_code
{
	sweep_unsupported = -100,
};

typedef struct
{
	int64_t center_freq;
} sweep_info;

typedef struct
{
	int64_t freq_start;
	int64_t freq_step;
	int points;
} sweep_config;

typedef int (*data_callback)(void *buf, int len);
typedef int (*sweep_callback)(void *buf, int len, sweep_info info);

class device
{
public:
    virtual ~device(){}
    virtual int init(data_callback cb) = 0;
	virtual int init(sweep_callback cb, sweep_config config){return sweep_unsupported;}
	virtual bool support_sweep(){return false;}
    virtual int destroy() = 0;
    virtual int show_config_dialog() = 0;
	virtual int tune(int64_t hz){return -1;}
    virtual int get_sample_rate() = 0;
	virtual int set_gains(uint8_t *gains){return -1;}
	virtual int get_gains(uint8_t *gains){return -1;}
	virtual int get_gains_count(){ return 0;}
    virtual sample_type get_sample_type(){return complex_sample;}
	virtual sample_quant get_sample_quant(){return sample_16bit;}
    virtual int noise_density(){return 130;}     // return noise density in dbFS/hz
};

}
