#pragma once

#include "autolock.h"
#include <stdint.h>
#include <Windows.h>
#include <queue>
#include <list>
#include "sinc.h"
#include <intrin.h>
#include "memory_pool.h"

class waveform_drawer
{
public:
	waveform_drawer(int width, int height);
	waveform_drawer();
	~waveform_drawer();

	int set_size(int width, int height);
	int draw16(const int16_t *data, int count)
	{
		int ly;
		for(unsigned int i=0; i<count; i++)
		{
			int x = i*width/count;
			int y = (32768-data[i]) * height / 65536;

			if (i == 0)
				ly = y;

			int _min = min(ly, y);
			int dy = abs(y-ly);
			float alpha = 3276.70f / (dy+3);

#if 0
				canvas[y+height*x] += alpha;
#else
			float *p = &canvas[_min+height*x];
			int count = dy+1;
			__m128 mv = _mm_set_ps1(alpha);
			for(int i=0; i<(count&(~3)); i+=4)
			{
				__m128 v = _mm_loadu_ps(p);
				v = _mm_add_ps(v, mv);
				_mm_storeu_ps(p, v);

				p+=4;
			}

			count &= 3;

			for(int i=0; i<count; i++)
				p[i] += alpha;
			ly = y;
#endif
		}

		this->count++;

		return 0;
	}

	float * get_data();
	int get_count();
	int reset();

	int width;
	int height;

protected:
	float *canvas;
	int count;
};


class threaded_drawer
{
public:
	threaded_drawer(int width, int height, int thread_count);
	~threaded_drawer();
	
	int draw16(const int16_t *data, int count, void *cb = NULL, int zoom = 1, int16_t trig = 0);	// TODO: callbacks
	int snapshot();
	int commit();
	int extract(float *out);

	bool wait_for_idle(int timeout = INFINITE);

	int max_works;

protected:
	static const int MAX_THREAD = 16;

	enum draw_type
	{
		_draw32,
		_draw16,
		_snapshot,
	};

	typedef struct  
	{
		draw_type type;
		const void *data;
		int count;
		void *cb;
		bool del;
		int zoom;
		int16_t trig;
	} work_entry;

	typedef struct  
	{
		float *data;
		int count;
		int frameid;
		int thread;
	} merger_entry;

	_critical_section works_cs[MAX_THREAD];
	std::queue<work_entry> works[MAX_THREAD];

	_critical_section merger_cs;
	std::list<merger_entry> merger;
	int frameid;
	int batch_size;
	int next_worker;

	_critical_section drawer_cs[MAX_THREAD];
	waveform_drawer drawers[MAX_THREAD];
	HANDLE work_handle[MAX_THREAD];
	HANDLE idle_handle[MAX_THREAD];
	HANDLE threads[MAX_THREAD];
	int thread_count;
	bool destroy;
	DWORD tids[MAX_THREAD];
	static DWORD WINAPI thread_entry(LPVOID p){return ((threaded_drawer*)p)->thread();}
	DWORD thread();
};
