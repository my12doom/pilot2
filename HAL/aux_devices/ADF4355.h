#pragma once

#include <stdint.h>

#define DECLARE_OPERATOR operator uint32_t(){return *(uint32_t*)this;}\
	uint32_t& operator =(uint32_t v){*(uint32_t*)this = v;return *(uint32_t*)this;}

struct adf4355_reg0
{
	unsigned ADDR:4;
	unsigned N_INT:16;
	unsigned PRESCALER:1;
	unsigned AUTOCAL:1;
	unsigned reserved:10;
	
	DECLARE_OPERATOR
};

struct adf4355_reg1
{
	unsigned ADDR:4;
	unsigned N_FRAC:24;
	unsigned reserved:4;
	
	DECLARE_OPERATOR
};

struct adf4355_reg2
{
	unsigned ADDR:4;
	unsigned MOD2:14;
	unsigned FRAC2:14;
	
	DECLARE_OPERATOR
};

struct adf4355_reg3
{
	unsigned ADDR:4;
	unsigned PHASE:24;
	unsigned PHASE_ADJUST:1;
	unsigned PHASE_RESYNC:1;
	unsigned SDLOAD_RESET:1;
	unsigned reserved:1;
	
	DECLARE_OPERATOR
};

struct adf4355_reg4
{
	unsigned ADDR:4;
	unsigned COUNTER_RESET:1;
	unsigned CP_3STATE:1;
	unsigned POWER_DOWN:1;
	unsigned PD_POLARITY:1;
	unsigned MUX_LOGIC:1;
	unsigned REF_MODE:1;
	unsigned CURRENT:4;
	unsigned DOUBLE_BUFF:1;
	unsigned R:10;
	unsigned RDIV2:1;
	unsigned R_DOUBLER:1;
	unsigned MUXOUT:3;
	unsigned reserved:2;
	
	DECLARE_OPERATOR
};

struct adf4355_reg5
{
	unsigned ADDR:4;
	unsigned reserved:28;	
	
	DECLARE_OPERATOR
};

struct adf4355_reg6
{
	unsigned ADDR:4;
	unsigned rf_power:2;
	unsigned rf_enable:1;
	unsigned rf_power_aux:2;
	unsigned rf_enable_aux:1;
	unsigned dbl_output_n:1;
	unsigned mute_till_lock:1;
	unsigned reserved2:1;
	unsigned CP_bleed_current:8;
	unsigned rf_divider:3;
	unsigned fb_select:1;
	unsigned reserved3:4;
	unsigned neg_bleed:1;
	unsigned gated_bleed:1;
	unsigned reserved4:1;

	DECLARE_OPERATOR
};

struct adf4355_reg7
{
	unsigned ADDR:4;
	unsigned LD_MODE:1;
	unsigned FRAC_LD_PRECISION:2;
	unsigned LOL_MODE:1;
	unsigned LD_CYCLE:2;
	unsigned reserved0:2;
	unsigned vco_readback:3;
	unsigned reserved1:10;
	unsigned LE_SYNC:1;
	unsigned reserved:6;

	DECLARE_OPERATOR
};

struct adf4355_reg8
{
	unsigned ADDR:4;
	unsigned reserved:28;

	DECLARE_OPERATOR
};

struct adf4355_reg9
{
	unsigned ADDR:4;
	unsigned LOCK_TIMEOUT:5;
	unsigned ALC_TIMEOUT:5;
	unsigned TIMEOUT:10;
	unsigned VCO_BAND_DIV:8;

	DECLARE_OPERATOR
};

struct adf4355_reg10
{
	unsigned ADDR:4;
	unsigned ADC_EN:1;
	unsigned ADC_CONV:1;
	unsigned ADC_CLK_DIV:8;
	unsigned reserved:12;
	unsigned VCO_READ:3;
	unsigned VCO_WRITE:3;

	DECLARE_OPERATOR
};

struct adf4355_reg11
{
	unsigned ADDR:4;
	unsigned vco_band:8;
	unsigned vco_bias:4;
	unsigned vco_core:4;
	unsigned reserved:28;
	
	DECLARE_OPERATOR
};

struct adf4355_reg12
{
	unsigned ADDR:4;
	unsigned reserved:12;
	unsigned RESYNC_CLK:16;

	DECLARE_OPERATOR
};
