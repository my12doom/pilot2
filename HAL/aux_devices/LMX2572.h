#pragma once

// LMX2572 as a basic frequency synthesizer
// only basic function implemented: ref&output freqency and output power config, lock detect.
// default regs generated by TICS Pro
// hardware: 2nd order loop filter in datasheet.
// default configuration: 40Mhz single ended ref in, 23.4375Mhz output on RFoutA

#include <HAL/Interface/ISPI.h>
#include <HAL/Interface/IGPIO.h>

typedef struct
{
	int64_t freq;
	uint8_t cap;
	uint8_t amp;
} LM2572_calibrate;

class LMX2572
{
public:
	LMX2572();
	~LMX2572();
	int init(HAL::ISPI *spi, HAL::IGPIO *cs);
	int set_ref(uint32_t ref_freq, bool doubler, int pre_R, int multiplier, int R, bool diff = false);
	int set_freq(uint64_t freq, bool sync_en = false);
	int sync_start(HAL::IGPIO *sync_pin);
	int set_output(bool enableA, int powerA, bool enableB = false, int powerB = 63);
	bool is_locked();

	// calibrate 
	bool calibrate();
	void get_calibration_coeff(float *coeff);
	void set_calibration_coeff(const float *coeff);

//protected:
	void write_reg(uint8_t address, uint16_t data);
	uint16_t read_reg(uint8_t address);

	HAL::ISPI *spi;
	HAL::IGPIO *cs;

	uint32_t pfd_freq;
	uint16_t regs[126];
	uint64_t last_vco_sel_freq;
	enum
	{
		cali_none = 0,
		cali_running = 1,
		cali_done = 2,
	} calibration;			// -1: not calibration, 0: calibrating, 1: calibratated
	float cap_coeff[18];
};
