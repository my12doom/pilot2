#include "device.h"
#include <windows.h>
#include <stdint.h>
#include "tinymt32.h"

typedef struct WAV_HEADER_STRUCT 
{
	BYTE  riff[4];            // must be "RIFF"
	DWORD len;                // #bytes + 44 - 8
	BYTE  cWavFmt[8];         // must be "WAVEfmt"
	DWORD dwHdrLen;

	WAVEFORMATEX wex;		  // wex.cbSize = 'da' =  0x6461

	WORD symbol_ta;			  // symbol_ta = 'ta' = 0x7461
	DWORD dwDataLen;          // #bytes

}WAV_HEADER;

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
	float scale;

    DWORD worker();
    static DWORD WINAPI entry(LPVOID p){return ((dummy_device*)p)->worker();}
};

class wav_device: public device
{
public:
	wav_device(wchar_t *filename);
	~wav_device(){destroy();};
	virtual int init(data_callback cb);
	virtual int destroy();
	virtual int config(){return 0;}
	int get_sample_rate(){return hdr.wex.nSamplesPerSec;}
	sample_type get_sample_type(){return hdr.wex.nChannels == 2 ? complex_sample : real_sample;}
	sample_quant get_sample_quant(){return quant;}
	int dynamic_range_db(){return 120;}     // return max noise density SNR in db

protected:
	sample_quant quant;
	float sample_rate;
	WAV_HEADER hdr;
	bool working;
	HANDLE thread;
	data_callback cb;
	FILE * f;
	DWORD worker();
	DWORD worker2();
	static DWORD WINAPI entry(LPVOID p){return ((wav_device*)p)->worker();}};


}
