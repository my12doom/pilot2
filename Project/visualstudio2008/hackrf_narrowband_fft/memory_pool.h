#pragma once

#include <list>
#include "autolock.h"

class memory_pool
{
public:
	memory_pool(int _max_items);
	memory_pool();
	~memory_pool();

	void *alloc(size_t size);
	void free(const void *p);
	void release();

protected:
	std::list<size_t*> free_list;
	_critical_section _cs_list;
	size_t max_items;
};
