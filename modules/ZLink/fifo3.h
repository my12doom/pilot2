#pragma once

#include <stdint.h>

template<int buffer_size>
class FIFO
{
public:
	FIFO():start(0),end(0){}
	~FIFO(){}

	int pop(void *out, int maxcount)
	{
		char *p = (char*)out;
		int size = count();
		if (maxcount > size)
			return -1;
		for(int i=0; i<maxcount; i++)
			p[i] = buffer[(i+start)%buffer_size];
		start = (maxcount+start)%buffer_size;
		return maxcount;
	}
	
	int peak(void *out, int maxcount)
	{
		char *p = (char*)out;
		int size = count();
		if (maxcount > size)
			maxcount = size;
		for(int i=0; i<maxcount; i++)
			p[i] = buffer[(i+start)%buffer_size];
		return maxcount;
	}
	
	int put(const void *data, int count)
	{
		if (available() < count)
			return -1;

		const char *p = (const char*)data;
		for(int i=0; i<count; i++)
			buffer[(end+i)%buffer_size] = p[i];
		
		end = (end + count) % buffer_size;

		return count;
	}
	
	int count()
	{
		int _end = end;
		int size = _end - start;
		if (size<0)
			size += buffer_size;
		return size;
	}
	
	int available()
	{
		return buffer_size - count() - 1;
	}

protected:
	uint8_t buffer[buffer_size];
	volatile int start;
	volatile int end;
};
