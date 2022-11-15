#include "ad9361_config.h"
#include "ad9361.h"
#include <HAL/STM32F4/F4SysTimer.h>
#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <math.h>
#include <stdio.h>
#include <inttypes.h>
#include <limits.h>

using namespace STM32F4;
using namespace HAL;

F4SPI spi3(SPI3);
F4GPIO cs(GPIOD, GPIO_Pin_2);
uint8_t reg;
uint8_t regs[200];
uint8_t tx[3];
uint8_t rx[3];

static uint64_t do_div(uint64_t* n, uint64_t base);
static uint32_t int_sqrt(uint32_t x);
static int32_t ad9361_rx_bb_analog_filter_calib(uint32_t rx_bb_bw, uint32_t bbpll_freq);
static int32_t ad9361_rx_tia_calib(uint32_t bb_bw_Hz);
static int32_t ad9361_rx_adc_setup(uint32_t bbpll_freq, uint32_t adc_sampl_freq_Hz);
static int32_t ad9361_run_calibration(uint32_t mask);
	
#define PRIu32        "u"
#define PRIu64      __PRISCN64 "u"
#define min(x, y)								(((x) < (y)) ? (x) : (y))
#define min_t(type, x, y)						(type)min((type)(x), (type)(y))
#define max(x, y)								(((x) > (y)) ? (x) : (y))
#define max_t(type, x, y)						(type)max((type)(x), (type)(y))
#define clamp(val, min_val, max_val)			(max(min((val), (max_val)), (min_val)))
#define DIV_ROUND_CLOSEST(x, divisor)			(((x) + (divisor) / 2) / (divisor))
#define DIV_ROUND_UP(x, y)						(((x) + (y) - 1) / (y))
#define dev_dbg printf
uint8_t ad936x_read_reg(uint16_t address)
{
	tx[0] = ((address>>8)&0x3) | ((1-1)<<4);
	tx[1] = address&0xff;	
	
	cs.write(false);
	spi3.txrx2(tx, rx, 3);
	cs.write(true);

	return rx[2];
}

void ad936x_write_reg(uint16_t address, uint8_t value)
{
	tx[0] = ((address>>8)&0x3) | ((1-1)<<4) | 0x80;
	tx[1] = address&0xff;
	tx[2] = value;	
	
	cs.write(false);
	spi3.txrx2(tx, rx, 3);
	cs.write(true);
}

void ad936x_write_reg_bits(uint16_t address, uint8_t mask, uint8_t value)
{
	int offset = 0;
	for(int i=0; i<8; i++)
	{
		if ( (mask>>i) & 1)
		{
			offset = i;
			break;
		}
	}
	uint8_t v = ad936x_read_reg(address);
	v &= ~mask;
	v |= (value<<offset)&mask;
	
	ad936x_write_reg(address, v);
}

int ad9363_test()
{
	spi3.set_mode(0,1);
	cs.set_mode(MODE_OUT_PushPull);
	cs.write(true);
		
	for(int i=0; i<100; i++)
	{
		regs[i] = ad936x_read_reg(i);
	}
	
	ad936x_write_reg(REG_CTRL, CTRL_ENABLE);
	ad936x_write_reg(REG_BANDGAP_CONFIG0, MASTER_BIAS_TRIM(0x0E)); // Enable Master Bias
	ad936x_write_reg(REG_BANDGAP_CONFIG1, BANDGAP_TEMP_TRIM(0x0E)); // Set Bandgap Trim
	ad936x_write_reg(0x03A, 39);			// ref clock = 40Mhz
	ad936x_write_reg(0x002, 0xDF);			// 2T
	ad936x_write_reg(0x003, 0xDF);			// 2R
	ad936x_write_reg(0x073, 60);			// 15db attenuation for TX1
	ad936x_write_reg(0x075, 60);			// 15db attenuation for TX2
	ad936x_write_reg(0x004, 0x30);			// use TX1A+TX2A + RX1C+RX2C
	ad936x_write_reg(0x2AB, 0x7);	
	ad936x_write_reg(0x2AC, 0xff);	// TX/RX RF PLL ref = Fref*2
	ad936x_write_reg(REG_CLOCK_ENABLE, DIGITAL_POWER_UP | CLOCK_ENABLE_DFLT | BBPLL_ENABLE | XO_BYPASS); // Enable Clocks
	ad936x_write_reg(0x0A, 0x12);		// BB PLL: CLKOUT=REF, CLK_OUT enable, BB PLL divider = 3(ADC CLK = BBPLL/8)

	systimer->delayms(20);
	
	// Set BBPLL Frequency: 983.040000 (61.44*16)
	ad936x_write_reg(0x045, 0x00);	// Set BBPLL reflclk scale to REFCLK
	ad936x_write_reg(0x046, 0x03);	// Set BBPLL Loop Filter Charge Pump current
	ad936x_write_reg(0x048, 0xE8);	// Set BBPLL Loop Filter C1, R1
	ad936x_write_reg(0x049, 0x5B);	// Set BBPLL Loop Filter R2, C2, C1
	ad936x_write_reg(0x04A, 0x35);	// Set BBPLL Loop Filter C3, 0xR2
	ad936x_write_reg(0x04B, 0xE0);	// Allow calibration to occur and set cal count to 1024 for max accuracy
	ad936x_write_reg(0x04E, 0x10);	// Set calibration clock to REFCLK/4 for more accuracy
	ad936x_write_reg(0x043, 0x29);	// BBPLL Freq Word (Fractional[7:0])
	ad936x_write_reg(0x042, 0x5C);	// BBPLL Freq Word (Fractional[15:8])
	ad936x_write_reg(0x041, 0x12);	// BBPLL Freq Word (Fractional[23:16])
	ad936x_write_reg(0x044, 0x18);	// BBPLL Freq Word (Integer[7:0])
	ad936x_write_reg(0x03F, 0x05);	// Start BBPLL Calibration
	ad936x_write_reg(0x03F, 0x01);	// Clear BBPLL start calibration bit
	ad936x_write_reg(0x04C, 0x86);	// Increase BBPLL KV and phase margin
	ad936x_write_reg(0x04D, 0x01);	// Increase BBPLL KV and phase margin
	ad936x_write_reg(0x04D, 0x05);	// Increase BBPLL KV and phase margin
	
	int64_t timeout = systimer->gettime() + 200000;
	while(!(ad936x_read_reg(0x05E)&0x80) && systimer->gettime() < timeout)
		;
	if (!(ad936x_read_reg(0x05E)&0x80))
		return -1;	// BB PLL error
	
	// parallel port config
	ad936x_write_reg(0x010, 0xC8);	// default + 2T2R timing + RX_FRAME pulse
	ad936x_write_reg(0x012, 0x22);	// Parral port config: full duplex dual port, SDR CMOS
	
	//************************************************************
	// Setup RF PLL non-frequency-dependent registers
	//************************************************************
	ad936x_write_reg(0x261, 0x00);	// Set Rx LO Power mode
	ad936x_write_reg(0x2A1, 0x00);	// Set Tx LO Power mode
	ad936x_write_reg(0x248, 0x0B);	// Enable Rx VCO LDO
	ad936x_write_reg(0x288, 0x0B);	// Enable Tx VCO LDO
	ad936x_write_reg(0x246, 0x02);	// Set VCO Power down TCF bits
	ad936x_write_reg(0x286, 0x02);	// Set VCO Power down TCF bits
	ad936x_write_reg(0x249, 0x8E);	// Set VCO cal length
	ad936x_write_reg(0x289, 0x8E);	// Set VCO cal length
	ad936x_write_reg(0x23B, 0x80);	// Enable Rx VCO cal
	ad936x_write_reg(0x27B, 0x80);	// Enable Tx VCO cal
	ad936x_write_reg(0x243, 0x0D);	// Set Rx prescaler bias
	ad936x_write_reg(0x283, 0x0D);	// Set Tx prescaler bias
	ad936x_write_reg(0x23D, 0x00);	// Clear Half VCO cal clock setting
	ad936x_write_reg(0x27D, 0x00);	// Clear Half VCO cal clock setting

	ad936x_write_reg(0x015, 0x0C);	// Set Dual Synth mode bit
	ad936x_write_reg(0x014, 0x05);	// Set Force ALERT State bit
	ad936x_write_reg(0x013, 0x01);	// Set ENSM FDD mode
	systimer->delayms(1);// waits 1 ms

	reg = ad936x_read_reg(0x244);
	ad936x_write_reg(0x23D, 0x14);	// Start RX CP cal
	ad936x_write_reg(0x27D, 0x14);	// Start TX CP cal
	
	timeout = systimer->gettime() + 200000;
	while((!(ad936x_read_reg(0x244)&0x80) || !(ad936x_read_reg(0x284)&0x80))
			&& systimer->gettime() < timeout)
		;

	reg = ad936x_read_reg(0x244);
	reg = ad936x_read_reg(0x284);
	if (!(ad936x_read_reg(0x244)&0x80) || !(ad936x_read_reg(0x284)&0x80))
		return -1;		// TX/RX CP error
	
	// Setup Rx Frequency-Dependent Syntheisizer Registers
	ad936x_write_reg(0x23A, 0x4A);	// Set VCO Output level[3:0]
	ad936x_write_reg(0x239, 0xC0);	// Set Init ALC Value[3:0] and VCO Varactor[3:0]
	ad936x_write_reg(0x242, 0x17);	// Set VCO Bias Tcf[1:0] and VCO Bias Ref[2:0]
	ad936x_write_reg(0x238, 0x40);	// Set VCO Cal Offset[3:0]
	ad936x_write_reg(0x245, 0x00);	// Set VCO Cal Ref Tcf[2:0]
	ad936x_write_reg(0x251, 0x00);	// Set VCO Varactor Reference[3:0]
	ad936x_write_reg(0x250, 0x70);	// Set VCO Varactor Ref Tcf[2:0] and VCO Varactor Offset[3:0]
	ad936x_write_reg(0x23B, 0xA4);	// Set Synth Loop Filter charge pump current (Icp)
	ad936x_write_reg(0x23E, 0xD4);	// Set Synth Loop Filter C2 and C1
	ad936x_write_reg(0x23F, 0xDF);	// Set Synth Loop Filter  R1 and C3
	ad936x_write_reg(0x240, 0x09);	// Set Synth Loop Filter R3

	// Setup Tx Frequency-Dependent Syntheisizer Registers
	ad936x_write_reg(0x27A, 0x4A);	// Set VCO Output level[3:0]
	ad936x_write_reg(0x279, 0xC0);	// Set Init ALC Value[3:0] and VCO Varactor[3:0]
	ad936x_write_reg(0x282, 0x17);	// Set VCO Bias Tcf[1:0] and VCO Bias Ref[2:0]
	ad936x_write_reg(0x278, 0x40);	// Set VCO Cal Offset[3:0]
	ad936x_write_reg(0x285, 0x00);	// Set VCO Cal Ref Tcf[2:0]
	ad936x_write_reg(0x291, 0x00);	// Set VCO Varactor Reference[3:0]
	ad936x_write_reg(0x290, 0x70);	// Set VCO Varactor Ref Tcf[2:0] and VCO Varactor Offset[3:0]
	ad936x_write_reg(0x27B, 0xA4);	// Set Synth Loop Filter charge pump current (Icp)
	ad936x_write_reg(0x27E, 0xD4);	// Set Synth Loop Filter C2 and C1
	ad936x_write_reg(0x27F, 0xDF);	// Set Synth Loop Filter  R1 and C3
	ad936x_write_reg(0x280, 0x09);	// Set Synth Loop Filter R3
	
	// Setup Frequency
	double frequency = 2300;		// Mhz
	double fref = 80.0;				// Mhz
	int Nint = floor(frequency/fref);
	int Nfrac = floor(8388593*(frequency/fref-Nint) + 0.5);
	
	//ad936x_write_reg(0x233, 0xF8);	// Write Rx Synth Fractional Freq Word[7:0]
	//ad936x_write_reg(0x234, 0xFF);	// Write Rx Synth Fractional Freq Word[15:8]
	//ad936x_write_reg(0x235, 0x3F);	// Write Rx Synth Fractional Freq Word[22:16]
	ad936x_write_reg(0x233, 0x00);	// Write Rx Synth Fractional Freq Word[7:0]
	ad936x_write_reg(0x234, 0x00);	// Write Rx Synth Fractional Freq Word[15:8]
	ad936x_write_reg(0x235, 0x00);	// Write Rx Synth Fractional Freq Word[22:16]
	ad936x_write_reg(0x232, 0x00);	// Write Rx Synth Integer Freq Word[10:8]
	ad936x_write_reg(0x231, 120);	// Write Rx Synth Integer Freq Word[7:0]
	
	ad936x_write_reg(0x273, 0);	// Write Tx Synth Fractional Freq Word[7:0]
	ad936x_write_reg(0x274, 0);	// Write Tx Synth Fractional Freq Word[15:8]
	ad936x_write_reg(0x275, 8);	// Write Tx Synth Fractional Freq Word[22:16]
	ad936x_write_reg(0x272, 0x00);	// Write Tx Synth Integer Freq Word[10:8]
	ad936x_write_reg(0x271, 123);	// Write Tx Synth Integer Freq Word[7:0] (starts VCO cal)
	ad936x_write_reg(0x005, 0x01);	// Set LO divider setting: TX: /2, RX: /4
	
	/*
	ad936x_write_reg(0x233, Nfrac&0xff);	// Write Rx Synth Fractional Freq Word[7:0]
	ad936x_write_reg(0x234, (Nfrac>>8)&0xff);	// Write Rx Synth Fractional Freq Word[15:8]
	ad936x_write_reg(0x235, (Nfrac>>16)&0xff);	// Write Rx Synth Fractional Freq Word[22:16]
	ad936x_write_reg(0x232, (Nint>>8)&3);	// Write Rx Synth Integer Freq Word[10:8]
	ad936x_write_reg(0x231, 0x51);	// Write Rx Synth Integer Freq Word[7:0]
	ad936x_write_reg(0x005, 0x00);	// Set LO divider setting
	ad936x_write_reg(0x273, 0xF8);	// Write Tx Synth Fractional Freq Word[7:0]
	ad936x_write_reg(0x274, 0xFF);	// Write Tx Synth Fractional Freq Word[15:8]
	ad936x_write_reg(0x275, 0x3F);	// Write Tx Synth Fractional Freq Word[22:16]
	ad936x_write_reg(0x272, 0x00);	// Write Tx Synth Integer Freq Word[10:8]
	ad936x_write_reg(0x271, 0x51);	// Write Tx Synth Integer Freq Word[7:0] (starts VCO cal)
	ad936x_write_reg(0x005, 0x00);	// Set LO divider setting
	*/
	
	
	systimer->delayms(1);

	// analog & ADC setup
	uint32_t bbpll_freq = 983040000;
	uint32_t bandwidth = 3000000;
	ad9361_rx_bb_analog_filter_calib(bandwidth, bbpll_freq);
	ad9361_rx_tia_calib(bandwidth);
	ad9361_rx_adc_setup(bbpll_freq, 7680000);
	
	reg = ad936x_read_reg(0x247);	// Check RX RF PLL lock status (0x247[1]==1 is locked)
	reg = ad936x_read_reg(0x287);	// Check TX RF PLL lock status (0x287[1]==1 is locked)

	reg = ad936x_read_reg(0x017);
	
	ad936x_write_reg(0x014, 0x20);	// clear Force ALERT State bit and go to FDD
	//ad936x_write_reg(0x014, 0x00);	// clear Force ALERT State bit and go to wait

	uint8_t v = 0;
	while(1)
	{
		//v++;
		systimer->delayms(100);
		ad936x_write_reg(0x014, 0x00);	// clear Force ALERT State bit and go to wait
		systimer->delayms(1);
		reg = ad936x_read_reg(0x017);	// Check TX RF PLL lock status (0x287[1]==1 is locked)
		ad936x_write_reg(0x273, 0x00);	// Write Tx Synth Fractional Freq Word[7:0]
		ad936x_write_reg(0x274, 0x00);	// Write Tx Synth Fractional Freq Word[15:8]
		//ad936x_write_reg(0x275, 0x00);	// Write Tx Synth Fractional Freq Word[22:16]
		ad936x_write_reg(0x273, v&0x7f);	// Write Tx Synth Fractional Freq Word[22:16]
		ad936x_write_reg(0x272, 0x00);	// Write Tx Synth Integer Freq Word[10:8]
		ad936x_write_reg(0x271, 145);	// Write Tx Synth Integer Freq Word[7:0] (starts VCO cal)
		ad936x_write_reg(0x014, 0x07);	// go alert
		systimer->delayus(200);
		ad936x_write_reg(0x014, 0x20);	// FDD
	}
	
	while(0)
	{
		reg = ad936x_read_reg(0x00E);	// read termperature
		systimer->delayms(100);
	}
	
	// gain
	ad936x_write_reg(0x109, 0);
	

	reg = ad936x_read_reg(REG_REF_DIVIDE_CONFIG_1);
	reg = ad936x_read_reg(REG_REF_DIVIDE_CONFIG_2);
	reg = ad936x_read_reg(0x248);
	reg = ad936x_read_reg(0x05E);
	
	
	return 0;
}


/**
 * Setup the RX ADC.
 * @param bbpll_freq The BBPLL frequency [Hz].
 * @param adc_sampl_freq_Hz The ADC sampling frequency [Hz].
 * @return 0 in case of success, negative error code otherwise.
 */
static int32_t ad9361_rx_adc_setup(uint32_t bbpll_freq, uint32_t adc_sampl_freq_Hz)
{
	uint32_t scale_snr_1e3, maxsnr, sqrt_inv_rc_tconst_1e3, tmp_1e3,
		scaled_adc_clk_1e6, inv_scaled_adc_clk_1e3, sqrt_term_1e3,
		min_sqrt_term_1e3, bb_bw_Hz;
	uint64_t tmp, invrc_tconst_1e6;
	uint8_t data[40];
	uint32_t i;
	int32_t ret;

	uint8_t c3_msb = ad936x_read_reg(REG_RX_BBF_C3_MSB);
	uint8_t c3_lsb = ad936x_read_reg(REG_RX_BBF_C3_LSB);
	uint8_t r2346 = ad936x_read_reg(REG_RX_BBF_R2346);
	int rxbbf_div = ad936x_read_reg(REG_RX_BBF_TUNE_DIVIDE) | ((ad936x_read_reg(REG_RX_BBF_TUNE_CONFIG)&1)<<8);

	/*
	* BBBW = (BBPLL / RxTuneDiv) * ln(2) / (1.4 * 2PI )
	* We assume ad9361_rx_bb_analog_filter_calib() is always run prior
	*/

	tmp = bbpll_freq * 10000ULL;
	do_div(&tmp, 126906UL * rxbbf_div);
	bb_bw_Hz = tmp;

	printf("%s : BBBW %"PRIu32" : ADCfreq %"PRIu32,
		__func__, bb_bw_Hz, adc_sampl_freq_Hz);

	printf("c3_msb 0x%X : c3_lsb 0x%X : r2346 0x%X : ",
		c3_msb, c3_lsb, r2346);

	bb_bw_Hz = clamp(bb_bw_Hz, 200000UL, 28000000UL);

	if (adc_sampl_freq_Hz < 80000000)
		scale_snr_1e3 = 1000;
	else
		scale_snr_1e3 = 1585; /* pow(10, scale_snr_dB/10); */

	if (bb_bw_Hz >= 18000000) {
		invrc_tconst_1e6 = (160975ULL * r2346 *
			(160 * c3_msb + 10 * c3_lsb + 140) *
			(bb_bw_Hz)* (1000 + (10 * (bb_bw_Hz - 18000000) / 1000000)));

		do_div(&invrc_tconst_1e6, 1000UL);

	}
	else {
		invrc_tconst_1e6 = (160975ULL * r2346 *
			(160 * c3_msb + 10 * c3_lsb + 140) *
			(bb_bw_Hz));
	}

	do_div(&invrc_tconst_1e6, 1000000000UL);

	if (invrc_tconst_1e6 > ULONG_MAX)
		printf("invrc_tconst_1e6 > ULONG_MAX");

	sqrt_inv_rc_tconst_1e3 = int_sqrt((uint32_t)invrc_tconst_1e6);
	maxsnr = 640 / 160;
	scaled_adc_clk_1e6 = DIV_ROUND_CLOSEST(adc_sampl_freq_Hz, 640);
	inv_scaled_adc_clk_1e3 = DIV_ROUND_CLOSEST(640000000,
		DIV_ROUND_CLOSEST(adc_sampl_freq_Hz, 1000));
	tmp_1e3 = DIV_ROUND_CLOSEST(980000 + 20 * max_t(uint32_t, 1000U,
		DIV_ROUND_CLOSEST(inv_scaled_adc_clk_1e3, maxsnr)), 1000);
	sqrt_term_1e3 = int_sqrt(scaled_adc_clk_1e6);
	min_sqrt_term_1e3 = min_t(uint32_t, 1000U,
		int_sqrt(maxsnr * scaled_adc_clk_1e6));

	printf("invrc_tconst_1e6 %llu, sqrt_inv_rc_tconst_1e3 %"PRIu32,
		invrc_tconst_1e6, sqrt_inv_rc_tconst_1e3);
	printf("scaled_adc_clk_1e6 %"PRIu32", inv_scaled_adc_clk_1e3 %"PRIu32,
		scaled_adc_clk_1e6, inv_scaled_adc_clk_1e3);
	printf("tmp_1e3 %"PRIu32", sqrt_term_1e3 %"PRIu32", min_sqrt_term_1e3 %"PRIu32,
		tmp_1e3, sqrt_term_1e3, min_sqrt_term_1e3);

	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0x24;
	data[4] = 0x24;
	data[5] = 0;
	data[6] = 0;

	tmp = -50000000 + 8ULL * scale_snr_1e3 * sqrt_inv_rc_tconst_1e3 *
		min_sqrt_term_1e3;
	do_div(&tmp, 100000000UL);
	data[7] = min_t(uint64_t, 124U, tmp);

	tmp = (invrc_tconst_1e6 >> 1) + 20 * inv_scaled_adc_clk_1e3 *
		data[7] / 80 * 1000ULL;
	do_div(&tmp, invrc_tconst_1e6);
	data[8] = min_t(uint64_t, 255U, tmp);

	tmp = (-500000 + 77ULL * sqrt_inv_rc_tconst_1e3 * min_sqrt_term_1e3);
	do_div(&tmp, 1000000UL);
	data[10] = min_t(uint64_t, 127U, tmp);

	data[9] = min_t(uint32_t, 127U, ((800 * data[10]) / 1000));
	tmp = ((invrc_tconst_1e6 >> 1) + (20 * inv_scaled_adc_clk_1e3 *
		data[10] * 1000ULL));
	do_div(&tmp, invrc_tconst_1e6 * 77);
	data[11] = min_t(uint64_t, 255U, tmp);
	data[12] = min_t(uint32_t, 127U, (-500000 + 80 * sqrt_inv_rc_tconst_1e3 *
		min_sqrt_term_1e3) / 1000000UL);

	tmp = -3 * (long)(invrc_tconst_1e6 >> 1) + inv_scaled_adc_clk_1e3 *
		data[12] * (1000ULL * 20 / 80);
	do_div(&tmp, invrc_tconst_1e6);
	data[13] = min_t(uint64_t, 255, tmp);

	data[14] = 21 * (inv_scaled_adc_clk_1e3 / 10000);
	data[15] = min_t(uint32_t, 127U, (500 + 1025 * data[7]) / 1000);
	data[16] = min_t(uint32_t, 127U, (data[15] * tmp_1e3) / 1000);
	data[17] = data[15];
	data[18] = min_t(uint32_t, 127U, (500 + 975 * data[10]) / 1000);
	data[19] = min_t(uint32_t, 127U, (data[18] * tmp_1e3) / 1000);
	data[20] = data[18];
	data[21] = min_t(uint32_t, 127U, (500 + 975 * data[12]) / 1000);
	data[22] = min_t(uint32_t, 127, (data[21] * tmp_1e3) / 1000);
	data[23] = data[21];
	data[24] = 0x2E;
	data[25] = (128 + min_t(uint32_t, 63000U, DIV_ROUND_CLOSEST(63 *
		scaled_adc_clk_1e6, 1000)) / 1000);
	data[26] = min_t(uint32_t, 63U, 63 * scaled_adc_clk_1e6 / 1000000 *
		(920 + 80 * inv_scaled_adc_clk_1e3 / 1000) / 1000);
	data[27] = min_t(uint32_t, 63, (32 * sqrt_term_1e3) / 1000);
	data[28] = data[25];
	data[29] = data[26];
	data[30] = data[27];
	data[31] = data[25];
	data[32] = data[26];
	data[33] = min_t(uint32_t, 63U, 63 * sqrt_term_1e3 / 1000);
	data[34] = min_t(uint32_t, 127U, 64 * sqrt_term_1e3 / 1000);
	data[35] = 0x40;
	data[36] = 0x40;
	data[37] = 0x2C;
	data[38] = 0x00;
	data[39] = 0x00;

	for (i = 0; i < 40; i++) {
		ad936x_write_reg(0x200 + i, data[i]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int32_t ad9361_rx_bb_analog_filter_calib(uint32_t rx_bb_bw, uint32_t bbpll_freq)
{
	uint32_t target;
	uint8_t tmp;
	int32_t ret;

	dev_dbg("%s : rx_bb_bw %"PRIu32" bbpll_freq %"PRIu32,
		__func__, rx_bb_bw, bbpll_freq);

	rx_bb_bw = clamp(rx_bb_bw, 200000UL, 28000000UL);

	/* 1.4 * BBBW * 2PI / ln(2) */
	target = 126906UL * (rx_bb_bw / 10000UL);
	int rxbbf_div = min_t(uint32_t, 511UL, DIV_ROUND_UP(bbpll_freq, target));

	/* Set RX baseband filter divide value */
	ad936x_write_reg(REG_RX_BBF_TUNE_DIVIDE, rxbbf_div);
	ad936x_write_reg_bits(REG_RX_BBF_TUNE_CONFIG, 1, rxbbf_div >> 8);

	/* Write the BBBW into registers 0x1FB and 0x1FC */
	ad936x_write_reg(REG_RX_BBBW_MHZ, rx_bb_bw / 1000000UL);

	tmp = DIV_ROUND_CLOSEST((rx_bb_bw % 1000000UL) * 128, 1000000UL);
	ad936x_write_reg(REG_RX_BBBW_KHZ, min_t(uint8_t, 127, tmp));

	ad936x_write_reg(REG_RX_MIX_LO_CM, RX_MIX_LO_CM(0x3F)); /* Set Rx Mix LO CM */
	ad936x_write_reg(REG_RX_MIX_GM_CONFIG, RX_MIX_GM_PLOAD(3)); /* Set GM common mode */

	/* Enable the RX BBF tune circuit by writing 0x1E2=0x02 and 0x1E3=0x02 */
	ad936x_write_reg(REG_RX1_TUNE_CTRL, RX1_TUNE_RESAMPLE);
	ad936x_write_reg(REG_RX2_TUNE_CTRL, RX2_TUNE_RESAMPLE);

	/* Start the RX Baseband Filter calibration in register 0x016[7] */
	/* Calibration is complete when register 0x016[7] self clears */
	 ret = ad9361_run_calibration(RX_BB_TUNE_CAL);

	/* Disable the RX baseband filter tune circuit, write 0x1E2=3, 0x1E3=3 */
	ad936x_write_reg(REG_RX1_TUNE_CTRL,
		RX1_TUNE_RESAMPLE | RX1_PD_TUNE);
	ad936x_write_reg(REG_RX2_TUNE_CTRL,
		RX2_TUNE_RESAMPLE | RX2_PD_TUNE);

	return ret;
}

static int32_t ad9361_rx_tia_calib(uint32_t bb_bw_Hz)
{
	uint32_t Cbbf, R2346;
	uint64_t CTIA_fF;

	uint8_t reg1EB = ad936x_read_reg(REG_RX_BBF_C3_MSB);
	uint8_t reg1EC = ad936x_read_reg(REG_RX_BBF_C3_LSB);
	uint8_t reg1E6 = ad936x_read_reg(REG_RX_BBF_R2346);
	uint8_t reg1DB, reg1DF, reg1DD, reg1DC, reg1DE, temp;

	dev_dbg("%s : bb_bw_Hz %"PRIu32, __func__, bb_bw_Hz);

	bb_bw_Hz = clamp(bb_bw_Hz, 200000UL, 20000000UL);

	Cbbf = (reg1EB * 160) + (reg1EC * 10) + 140; /* fF */
	R2346 = 18300 * RX_BBF_R2346(reg1E6);

	CTIA_fF = Cbbf * R2346 * 560ULL;
	do_div(&CTIA_fF, 3500000UL);

	if (bb_bw_Hz <= 3000000UL)
		reg1DB = 0xE0;
	else if (bb_bw_Hz <= 10000000UL)
		reg1DB = 0x60;
	else
		reg1DB = 0x20;

	if (CTIA_fF > 2920ULL) {
		reg1DC = 0x40;
		reg1DE = 0x40;
		temp = min(127U, DIV_ROUND_CLOSEST((uint32_t)CTIA_fF - 400, 320U));
		reg1DD = temp;
		reg1DF = temp;
	}
	else {
		temp = DIV_ROUND_CLOSEST((uint32_t)CTIA_fF - 400, 40U) + 0x40;
		reg1DC = temp;
		reg1DE = temp;
		reg1DD = 0;
		reg1DF = 0;
	}

	ad936x_write_reg(REG_RX_TIA_CONFIG, reg1DB);
	ad936x_write_reg(REG_TIA1_C_LSB, reg1DC);
	ad936x_write_reg(REG_TIA1_C_MSB, reg1DD);
	ad936x_write_reg(REG_TIA2_C_LSB, reg1DE);
	ad936x_write_reg(REG_TIA2_C_MSB, reg1DF);

	return 0;
}

static int32_t ad9361_run_calibration(uint32_t mask)
{
	ad936x_write_reg(REG_CALIBRATION_CTRL, mask);

	dev_dbg("%s: CAL Mask 0x%08x", __func__, mask);
	
	int64_t timeout = systimer->gettime() + 20000;
	while ( (ad936x_read_reg(REG_CALIBRATION_CTRL) & mask) && systimer->gettime() < timeout)
		;
	
	int32_t ndone = ad936x_read_reg(REG_CALIBRATION_CTRL) & mask;
	if (ndone)
		dev_dbg("%s: CAL Mask 0x%08x timed out", __func__, mask);
	
	return ndone;
}

static uint64_t do_div(uint64_t* n, uint64_t base)
{
	uint64_t mod = 0;

	mod = *n % base;
	*n = *n / base;

	return mod;
}

static uint32_t int_sqrt(uint32_t x)
{
	uint32_t b, m, y = 0;

	if (x <= 1)
		return x;

	m = 1UL << (32 - 2);
	while (m != 0) {
		b = y + m;
		y >>= 1;

		if (x >= b) {
			x -= b;
			y += m;
		}
		m >>= 2;
	}

	return y;
}