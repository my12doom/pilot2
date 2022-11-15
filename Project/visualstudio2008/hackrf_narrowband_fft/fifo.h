#pragma once

#define _CRT_SECURE_NO_DEPRECATE

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <Windows.h>

// This allocates a "magic ring buffer" that is mapped twice, with the two
// copies being contiguous in (virtual) memory. The advantage of this is
// that this allows any function that expects data to be contiguous in
// memory to read from (or write to) such a buffer. It also means that
// block reads/writes never need to be split into two halves, and makes
// wraparound handling quite cheap.
//
// The flipside is that allocating such a beast is a bit dicey (see
// comments below) and subject to various restrictions (e.g. on Windows,
// the size of such a buffer must be a multiple of 64k). So the usual
// disclaimer applies: code responsibly, with great power comes great
// responsibility, and when you use this code to shoot yourself in the
// foot it might blow off your face instead (what with the wraparound
// and all).
class MagicRingBuffer
{
	HANDLE mapping;
	size_t size;
	uint8_t *baseptr;
	size_t read_pos;
	size_t write_pos;

public:
	MagicRingBuffer()
		: mapping(0), size(0), baseptr(0), read_pos(0), write_pos(0)
	{
	}

	MagicRingBuffer(size_t ring_size)
		: mapping(0), size(0), baseptr(0), read_pos(0), write_pos(0)
	{
		alloc(ring_size);
	}

	~MagicRingBuffer()
	{
		free();
	}

	// Allocate a magic ring buffer at a given target address.
	//   ring_size      size of one copy of the ring; must be a multiple of 64k.
	//   desired_addr   location where you'd like it.
	void *alloc_at(size_t ring_size, void *desired_addr=0);

	// This function will allocate a magic ring buffer at a system-determined base address.
	void *alloc(size_t ring_size, int num_retries=5);

	// Frees the allocated region again.
	void free();

	//
	int push_back(const uint8_t *data, size_t count, bool whole = true);
	
	//
	int pop_front(uint8_t *data, size_t count, bool whole = true);

	//
	const uint8_t * get_read_ptr();

	// 
	uint8_t * get_write_ptr();

	//
	size_t free_space();

	//
	size_t used_space();

private:

	// Determine a viable target address of "size" memory mapped bytes by
	// allocating memory using VirtualAlloc and immediately freeing it. This
	// is subject to a potential race condition, see notes above.
	void *determine_viable_addr(size_t size);
};

int test_fifo();