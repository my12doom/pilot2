#pragma once

#include <HAL/Interface/II2C.h>

class si5351
{
public:

    enum output_type
    {
        output_XTAL = 0,
        output_CLKIN = 1,
        output_RESERVED = 2,
        output_pll = 3,
    };

    enum drive_current
    {
        _2ma = 0,
        _4ma = 1,
        _6ma = 2,
        _8ma = 3,
    };

    enum pll_freq_src
    {
        XTAL = 0,
        CLKIN = 1,        
    };

    si5351(float crystal = 25.0f);
    ~si5351();

    int set_ref_freq(float freq);
    int set_i2c(HAL::II2C *i2c, uint8_t address = 0xc0);
    int set_pll(int id, double vco_freq, int freq_src = XTAL, int input_divider = 1);
    int set_output_freq(int id, int freq_src, double divider);      // freq_src: 0: PLLA, 1:PLLB
    int set_output(int id, bool power_down, bool invert, output_type output_type = output_pll, drive_current drive_current = _2ma);

protected:
    uint8_t address;
    HAL::II2C *i2c;
    float ref;
    double vco_freq[2];
};

int test_si5351(HAL::II2C *i2c);
