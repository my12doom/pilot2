#pragma once

#include <stdint.h>

#define F4SDR_VENDOR_ID 0x111b
#define F4SDR_PRODUCT_ID 0x1234

namespace F4SDR
{
enum F4SDR_ops
{

	erase_fpga_rom = 0x1,	
	write_fpga_rom = 0x2,
	check_fpga_rom = 0x3,
	reconfigure_fpga = 0x4,

	ping = 0x08,		// for flash erase completion check.
	stream_enable = 0x09,	// enable or disable streaming, useful for flushing stream queue.

	register_write = 0x14,
	register_read = 0x15,

	tune = 0x26,
	set_gain = 0x27,
	get_gain = 0x28,
	set_path = 0x29,

	set_sweep_start = 0x30,
	set_sweep_step = 0x31,
	set_sweep_points = 0x32,	// set points to 0 to disable sweeping
	set_sweep_blocks = 0x33,

	reset_mcu = 0xA3,
};

enum F4SDR_registers
{

};

enum frontend_sel
{
	FE_LNA = 1,
	FE_BYPASS = 2,
	FE_LNA58 = 4,
};

enum IF_sel
{
	IF_800 = 0,
	IF_900 = 1,
};

enum IF_amp
{
	IF_AMP_BYPASS = 0,
	IF_AMP = 1,
};
typedef struct
{
	uint8_t zero : 1;
	uint8_t IF_select : 1;
	uint8_t IF_sel_override : 1;
	uint8_t resv1 : 1;
	uint8_t IF_amp : 1;
	uint8_t front_end : 3;
} reg_rf_path;

typedef struct
{
	int64_t center_freq;
	int16_t block_index;
} sweep_header;

}
