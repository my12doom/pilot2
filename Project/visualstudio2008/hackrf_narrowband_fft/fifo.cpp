#include "fifo.h"

// Allocate a magic ring buffer at a given target address.
//   ring_size      size of one copy of the ring; must be a multiple of 64k.
//   desired_addr   location where you'd like it.
void *MagicRingBuffer::alloc_at(size_t ring_size, void *desired_addr/*=0*/)
{
	// if we already hold one allocation, refuse to make another.
	if (baseptr)
		return 0;

	// is ring_size a multiple of 64k? if not, this won't ever work!
	if ((ring_size & 0xffff) != 0)
		return 0;

	// try to allocate and map our space
	size_t alloc_size = ring_size * 2;
	if (!(mapping = CreateFileMappingA(INVALID_HANDLE_VALUE, 0, PAGE_READWRITE, (unsigned long long)alloc_size >> 32, alloc_size & 0xffffffffu, 0)) ||
		!(baseptr = (uint8_t *)MapViewOfFileEx(mapping, FILE_MAP_ALL_ACCESS, 0, 0, ring_size, desired_addr)) ||
		!MapViewOfFileEx(mapping, FILE_MAP_ALL_ACCESS, 0, 0, ring_size, (char *)desired_addr + ring_size))
	{
		// something went wrong - clean up
		free();
	}
	else // success!
		size = ring_size;

	return baseptr;
}

// This function will allocate a magic ring buffer at a system-determined base address.
void *MagicRingBuffer::alloc(size_t ring_size, int num_retries/*=5*/)
{
	free();
	void *ptr = 0;
	while (!ptr && num_retries-- != 0)
	{
		void *target_addr = determine_viable_addr(ring_size * 2);
		if (target_addr)
			ptr = alloc_at(ring_size, target_addr);
	}

	return ptr;
}

// Frees the allocated region again.
void MagicRingBuffer::free()
{
	if (baseptr)
	{
		UnmapViewOfFile(baseptr);
		UnmapViewOfFile(baseptr + size);
		baseptr = 0;
	}

	if (mapping)
	{
		CloseHandle(mapping);
		mapping = 0;
	}

	size = 0;
	read_pos = 0;
	write_pos = 0;
}

// Determine a viable target address of "size" memory mapped bytes by
// allocating memory using VirtualAlloc and immediately freeing it. This
// is subject to a potential race condition, see notes above.
void *MagicRingBuffer::determine_viable_addr(size_t size)
{
	void *ptr = VirtualAlloc(0, size, MEM_RESERVE, PAGE_NOACCESS);
	if (!ptr)
		return 0;

	VirtualFree(ptr, 0, MEM_RELEASE);
	return ptr;
}

//
const uint8_t * MagicRingBuffer::get_read_ptr()
{
	return baseptr + read_pos;
}

uint8_t * MagicRingBuffer::get_write_ptr()
{
	return baseptr + write_pos;
}

// 
int MagicRingBuffer::push_back(const uint8_t *data, size_t count, bool whole /*= true*/)
{
	if (free_space() < count)
	{
		if (whole)
			return -1;
		count = free_space();
	}

	if (data)
		memcpy(baseptr+write_pos, data, count);

	write_pos += count;

	return count;
}

// 
int MagicRingBuffer::pop_front(uint8_t *data, size_t count, bool whole/* = true*/)
{
	if (used_space() < count)
	{
		if (whole)
			return -1;

		count = used_space();
	}

	if (data)
		memcpy(data, baseptr + read_pos, count);

	read_pos += count;
	if (read_pos > size)
	{
		read_pos -= size;
		write_pos -= size;
	}

	return count;
}

//
size_t MagicRingBuffer::free_space()
{
	return size - used_space();
}

//
size_t MagicRingBuffer::used_space()
{
	return write_pos - read_pos;
}


int test_fifo()
{
	static const int ringsize = 64*1024;

	MagicRingBuffer mrb;
	char *buf = (char *)mrb.alloc(ringsize);
	if (!buf)
	{
		printf("MRB allocation failed!\n");
		return 0;
	}

	// Okay, now get ready for a magic trick!
	memset(buf, 0, ringsize * 2); // clear it all to zeroes

	strcpy(buf + ringsize - 3, "Hello world!");
	printf("%s\n", buf + ringsize - 3);
	printf("%s\n", buf);

	// nothing up my sleeve!

	return 0;
}
