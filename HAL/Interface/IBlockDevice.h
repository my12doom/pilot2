#pragma once

#include <stdint.h>

namespace HAL
{
	// abstract class for block device.
	// for read only or write only device, simply return an error in write()/read() function.

	enum block_error
	{
		error_unsupported = -5,
		error_queue_full = -6,
		error_buffer_too_small = -7,
		error_no_more_blocks = -8,
	};

	class IBlockDevice
	{
	public:
		// write a block
		// class implementation is responsible for queueing blocks, or reject incoming blocks by returning an error.
		// returns num bytes written, negative values for error.
		virtual int write(const void *buf, int block_size) = 0;

		// read a block from rx queue, remove block from the queue if remove == true.
		// returns num bytes read, negative values for error.
		virtual int read(void *buf, int max_block_size, bool remove = true) = 0;

		// query num available blocks in rx queue, negative values for error.
		virtual int available() = 0;
	};
}