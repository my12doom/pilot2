#include "memory_pool.h"

using namespace std;

memory_pool::memory_pool(int _max_items)
{
	max_items = _max_items;
}
memory_pool::memory_pool()
{
	max_items = 1024;
}
memory_pool::~memory_pool()
{
 	release();
}

void *memory_pool::alloc(size_t size)
{
	_autolock lck(&_cs_list);
	for(list<size_t*>::iterator i = free_list.begin(); i!=free_list.end();)
	{
		size_t *s = (size_t*)*i;
		if (*s >= size)
		{
			free_list.erase(i);
			return &s[1];
		}
		else
		{
			i++;
		}
	}

	size_t *p = (size_t*)malloc(size + sizeof(size_t) + 16);
// 	printf("malloc:%08x, size %d\n", p, size);
	*p = size;
	return p+1;
}

void memory_pool::free(const void *p)
{
	size_t *s = (size_t*)p;
	_autolock lck(&_cs_list);
	free_list.push_front(s-1);
	
	if (free_list.size() > max_items)
	{
		list<size_t*>::iterator first = free_list.end();
		first --;
		::free(*first);
		free_list.erase(first);
	}

}

void memory_pool::release()
{
	_autolock lck(&_cs_list);
// 	printf("pool: %d/%d\n", free_list.size(), max_items);
	int n = 0;
	for(list<size_t*>::iterator i = free_list.begin(); i!=free_list.end(); i++)
	{
// 		size_t *p = (size_t*)*i;
// 		printf("free:%08x, size = %d\n", p, p[0]);
		::free(*i);
	}

	free_list.clear();
}
