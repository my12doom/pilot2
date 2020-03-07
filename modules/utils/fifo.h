#pragma once

#include <stdlib.h>

template<class T, int max_elements>
class CircularQueue
{
public:
	CircularQueue():
	start(0),
	end(0)
	{

	}
	~CircularQueue()
	{

	}

	// push a element to end of the queue
	// return 0 if success
	//        -1 on error (queue full)
	int push(const T &v)
	{
		if (count() >= max_elements - 1)
			return -1;

		elements[(end)%max_elements] = v;

		end = (end+1)%max_elements;

		return 0;
	}

	// pop and return the first element out of the queue.
	// return 0 if success and *out modified if valid pointer passed.
	//        -1 if no element available and *out remain untouched.
	int pop(T *out)
	{
		if (count() == 0)
		{
			return -1;
		}

		if (out)
			*out = elements[start];

		start = (start+1)%max_elements;

		return 0;
	}


	// peak the elements at the specified index of queues.
	// return 0 if success and *out if modified.
	//        -1 if element not found and *out remain untouched or invaild out pointer.
	int peek(int index, T*out)
	{
		if (index >= count())
		{
			return -1;
		}

		if (!out)
			return -1;

		*out = elements[(start+index)%max_elements];
		return 0;
	}

	// return max continuous elements from the top of the queue, *out is modified to pointer of the first element
	// return 0 if there is no elements left, and *out is modified to NULL
	int peek2(int max_count, T**out)
	{
		if (!out)
			return 0;

		if(count() == 0)
		{
			*out = 0;
			return 0;
		}

		int continuous_count = (count() + start < max_count) ? count() : (max_count - start);

		*out = &elements[start];
		return continuous_count;
	}


	// pop and discards n elements at the top of the queue.
	// return number of elements removed.
	int pop_n(int count)
	{
		if (count > this->count())
			count = this->count();

		start = (start + count)%max_elements;

		return count;			
	}

	// remove all elements
	void clear()
	{
		start = end = 0;
	}

	int count()
	{
		volatile int size = end - start;
		if (size<0)
			size += max_elements;
		return size;
	}

	int left()
	{
		return max_elements - count() - 1;
	}

protected:
	T elements[max_elements];
	volatile int start;
	volatile int end;
};
