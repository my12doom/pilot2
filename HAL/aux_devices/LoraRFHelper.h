#pragma once

#include <math.h>

// reference: http://www.rfwireless-world.com/calculators/LoRa-Sensitivity-Calculator.html
// sensitivity = -174 + 10log10(BW) + NF + SNR

// input: BW: bandwidth in khz
//		  SF: spread factor, 7 ~ 12
//		  NF: frontedn noise figure
// returns:
//		  sensitivity in dbm


static float lora_calc_sensitivity(int BW, int SF, float NF = 6.0f)
{
	if (SF < 6 || SF > 12)
		return 0;

	static const float snr[] = { -5.0f, -7.5f, -10.0f, -12.5f, -15.0f, -17.5f, -20.0f };

	return -174.0f + 10 * log10(BW*1000.0f) + NF + snr[SF-6];
}


// reference: http://www.rfwireless-world.com/calculators/LoRa-Data-Rate-Calculator.html
// Rb = SF * [4/(4+CR)] / [2^SF/BW] * 1000
// input: BW: bandwidth in khz
//		  CR: coderate, 1 for 4/5, 2 for 2/3, 3 for 4/7, 4 for 1/2
//		  SF: spread factor, 6 ~ 12

static int lora_calc_datarate(int BW, int CR, int SF)
{
	int power = 1 << SF;

	return SF  * 1000 * BW * 4 /(4+CR) / power ;
}
// reference: https://www.semtech.com/uploads/documents/LoraDesignGuide_STD.pdf
//			  https://www.loratools.nl/#/airtime
// input: BW: bandwidth in khz
//		  CR: coderate, 1 for 4/5, 2 for 2/3, 3 for 4/7, 4 for 1/2
//		  SF: spread factor, 6 ~ 12
//		  payload_bytes

// output: air time in micro-second
static int lora_calc_airtime(int BW, int CR, int SF, int payload_bytes, int preamble_setting = 8)
{
	int symbol_time = (1 << SF) * 1000 / BW;
	int preamble_symbol_time = (preamble_setting+4) * symbol_time + symbol_time / 4;
	int payload_symbol_count = payload_bytes * 8 - 4 * SF + 44;
	payload_symbol_count = (payload_symbol_count + SF * 4 - 1) / (SF * 4) * (CR + 4) + 8;

	return preamble_symbol_time + payload_symbol_count * symbol_time;
}

// input: rate: rate in bps
//		  payload_bytes
// output: air time in micro-second
static int gfsk_calc_airtime(int rate,int payload_bytes)
{
	int extra_bytes = 1 + 2 + 2 + 4;	// 2byte preamble, 2byte CRC, 4byte tail(bug of SX1278), 1byte ramp
	return (payload_bytes+extra_bytes) * 8 * 1000000 / rate;
}
