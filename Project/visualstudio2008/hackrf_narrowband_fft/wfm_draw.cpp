#include "wfm_draw.h"
memory_pool wavefrm_pool(1024);
memory_pool zoom_pool(32);
memory_pool img_pool(16);

waveform_drawer::waveform_drawer(int width, int height)
{
	canvas = NULL;
	set_size(width, height);
}
waveform_drawer::waveform_drawer()
{
	canvas = NULL;

}
waveform_drawer::~waveform_drawer()
{
	if (canvas)
		img_pool.free(canvas);
}

int waveform_drawer::set_size(int width, int height)
{
	if (canvas)
		img_pool.free(canvas);

	this->width = width;
	this->height = height;

	canvas = (float*)img_pool.alloc(sizeof(float)*width*height);
	reset();

	return 0;
}



float * waveform_drawer::get_data()
{
	return canvas;
}
int waveform_drawer::get_count()
{
	return count;
}
int waveform_drawer::reset()
{
	count = 0;
	memset(canvas, 0, width * height * sizeof(float));

	return 0;
}

threaded_drawer::threaded_drawer(int width, int height, int thread_count)
{
	this->thread_count = thread_count;
	next_worker = 0;
	batch_size = 50;
	destroy = false;
	max_works = batch_size * 2;
	frameid = 0;


	memset(tids, 0, sizeof(tids));

	for(int i=0; i<thread_count; i++)
	{
		idle_handle[i] = CreateEvent(NULL, TRUE, FALSE, NULL);		// initially all thread idle
		work_handle[i] = CreateEvent(NULL, TRUE, FALSE, NULL);		// initially no work

		drawers[i].set_size(width, height);
		threads[i] = CreateThread(NULL, NULL, thread_entry, this, NULL, &tids[i]);
	}
}

threaded_drawer::~threaded_drawer()
{
	destroy = true;
	for(int i=0; i<thread_count; i++)
		SetEvent(work_handle[i]);

	WaitForMultipleObjects(thread_count, threads, TRUE, INFINITE);

	for(int i=0; i<thread_count; i++)
	{
		CloseHandle(idle_handle[i]);
		CloseHandle(work_handle[i]);
	}

	wavefrm_pool.release();
	img_pool.release();
 	zoom_pool.release();
}

int threaded_drawer::draw16(const int16_t *data, int count, void *cb/* = NULL*/, int zoom /* = 1 */, int16_t trig)	// TODO: callbacks
{
	next_worker = (next_worker + 1) % thread_count;

	max_works = max(2, batch_size * 8 * 1024 / count);


	if (thread_count > 1)
	{
		_autolock lck(&works_cs[next_worker]);
		if (works[next_worker].size() > max_works)
			return -1;

		int16_t *blk = (int16_t*)wavefrm_pool.alloc(sizeof(int16_t) * count);//new int16_t[count];
		memcpy(blk, data, count * 2);
		work_entry item = {_draw16, blk, count, cb, blk, zoom, trig};
		works[next_worker].push(item);
		if (works[next_worker].size() > batch_size)
			commit();
	}
	else
	{
		_autolock lck(&drawer_cs[0]);
		drawers[0].draw16(data, count);
	}
	return 0;
}

int threaded_drawer::commit()
{
	for(int i=0; i<thread_count; i++)
		SetEvent(work_handle[i]);
	return 0;
}

int threaded_drawer::extract(float *out)
{
	int width = drawers[0].width;
	int height = drawers[0].height;
	int count = 0;
	int max_snapshots = thread_count * 10;

	_autolock lck(&merger_cs);
	bool drop_frame = merger.size() > max_snapshots;

	if (merger.size() < thread_count)
		return 0;

	int id = merger.begin()->frameid;
	int matched_id = -1;

	do 
	{
		bool found[MAX_THREAD] = {false};
		for(std::list<merger_entry>::iterator i = merger.begin(); i != merger.end(); i++ )
		{
			if (i->frameid == id && i->thread >= 0 && i->thread < MAX_THREAD)
				found[i->thread] = true;
		}

		bool match = true;
		for(int i=0; i<thread_count; i++)
		{
			if (!found[i])
			{
				match = false;
				break;
			}
		}

		if (match)
		{
			matched_id = id;
			if (!drop_frame)
				break;
		}
		id ++;
	}
	while (matched_id + 1 == id);


	if (matched_id >= 0)
	{
		bool first_piece = true;
		//memset(frm, 0, sizeof(float)*frm_width*frm_height);

		for(std::list<merger_entry>::iterator i = merger.begin(); i != merger.end();  )
		{
			if (i->frameid < matched_id)
			{
				//printf("drop piece %d of %d\n", i->thread, i->id);
				img_pool.free(i->data);
				merger.erase(i++);
			}
			else if (i->frameid == matched_id && i->thread >= 0 && i->thread < MAX_THREAD)
			{
				count += i->count;
				if (i->count)
				{
					for(int j=0; j<width*height; j+=4)
					{
						__m128 v = _mm_loadu_ps(i->data+j);
						if (!first_piece)
						{
							__m128 v2 = _mm_loadu_ps(out+j);
							v = _mm_add_ps(v, v2);
						}
						_mm_storeu_ps(out+j, v);
					}

					first_piece = false;
				}
				img_pool.free(i->data);
				merger.erase(i++);
			}
			else
			{
				i ++;
			}

		}

// 		printf("out: id = %d, count = %d, left = %d\n", id, count, merger.size());
	}

	// error fix!
	if (matched_id < 0 && drop_frame)
	{
		for(std::list<merger_entry>::iterator i = merger.begin(); i != merger.end();  )
		{
			img_pool.free(i->data);
			merger.erase(i++);
		}
	}

	return count;
}

int threaded_drawer::snapshot()
{
	for(int i=0; i<thread_count; i++)
	{
		_autolock lck(&works_cs[i]);
		work_entry item = {_snapshot, NULL, frameid};
		works[i].push(item);
	}

	return frameid++;
}

bool threaded_drawer::wait_for_idle(int timeout/* = INFINITE*/)
{
	DWORD rtn = WaitForMultipleObjects(thread_count, idle_handle, TRUE, timeout);
	return rtn != WAIT_TIMEOUT;
}

#define _mm_cmpge_epu16(a, b) _mm_cmpeq_epi16(_mm_max_epu16(a, b), a)

static int int16tofloat(float *dst, const int16_t*src, int count)
{
	__m128i m8000 = _mm_set1_epi16(0x8000);
	for(int i=0; i<count; i+=8)
	{
		__m128i m1 = _mm_loadu_si128((__m128i*)(src+i));

		__m128i mh = _mm_cmpge_epu16(m1, m8000);
		__m128i m2 = _mm_unpackhi_epi16(m1, mh);
		m1 = _mm_unpacklo_epi16(m1, mh);

		__m128 m1f = _mm_cvtepi32_ps(m1);
		__m128 m2f = _mm_cvtepi32_ps(m2);

		_mm_storeu_ps(dst+i+0, m1f);
		_mm_storeu_ps(dst+i+4, m2f);
	}

	return 0;
}

static int floattoint16(int16_t *dst, const float *src, int count)
{
	for(int i=0; i<count; i+=8)
	{
		__m128 m1f = _mm_loadu_ps(src+i);
		__m128 m2f = _mm_loadu_ps(src+i+4);

		__m128i m1 = _mm_cvtps_epi32(m1f);
		__m128i m2 = _mm_cvtps_epi32(m2f);
		m1 = _mm_packs_epi32(m1, m2);

		_mm_storeu_si128((__m128i*)(dst+i), m1);
	}

	return 0;
}

static int floattoint16r(int16_t *dst, const float *src, int count)
{
	for(int i=0; i<count; i++)
		dst[i] = src[i];

	return 0;
}

DWORD threaded_drawer::thread()
{
	int tid = GetThreadId(GetCurrentThread());
	int id = -1;
	for(int i=0; i<MAX_THREAD; i++)
		if (tids[i] == tid)
			id = i;

	if (id <0)
		return -1;

	sinc_intepolator sinc;

idle:
	
	WaitForSingleObject(work_handle[id], INFINITE);

prework:
	if (destroy)
		return 0;
	Sleep(1);


get_work:
	// get work
	int work_count;
	work_entry local_works[1024];
	work_count = 0;

	{
		_autolock lck(&works_cs[id]);
		if (works[id].empty())
		{
			ResetEvent(work_handle[id]);
			SetEvent(idle_handle[id]);
			goto prework;
		}
		else
		{
			ResetEvent(idle_handle[id]);
		}

		while (work_count < batch_size && !works[id].empty())
		{
			local_works[work_count++] = works[id].front();
			works[id].pop();
		}
	}

	// now work
	{
		_autolock lck(&drawer_cs[id]);
		int width = drawers[id].width;
		int height = drawers[id].height;
		float *tmp = (float*)zoom_pool.alloc(width*320*sizeof(float));
		int16_t *tmp16 = (int16_t*)zoom_pool.alloc(sizeof(int16_t)*width*320);

		for(int i=0; i<work_count; i++)
		{
			if (local_works[i].type == _draw16)
			{
				int count = local_works[i].count;
				const int16_t *data = (const int16_t*)local_works[i].data;
				int zoom = local_works[i].zoom;

				if (zoom <= 1)
				{
					// direct draw
					drawers[id].draw16(data, count);
				}
				else
				{
					// zoom up
					int ratio = min(zoom, 8);
					int out_count = count*ratio;
					int in_count = out_count / zoom;
					int16_t *p = tmp16 + out_count/2 - count/2;
					int16_t trig = local_works[i].trig;
					{
						static _critical_section fft_lock;
						_autolock lck(&fft_lock);
						sinc.set_size(in_count, out_count);
					}

					int16tofloat(tmp, data + count/2 - in_count/2, in_count);
					sinc.apply(tmp, tmp);
					floattoint16(tmp16, tmp, out_count);

					// fine trigger
					int pos = 0;
					for(int i=-zoom*2; i<=zoom*2; i++)
						if (/*p[i-1+count/2] < trig &&*/ p[i+count/2] > trig)
						{
							pos = i;
							break;
						}

					drawers[id].draw16(p+pos, count);
				}
			}

			else if (local_works[i].type == _snapshot)
			{
				// TODO
// 				printf(" snap %d of thread %d, count=%d \n", local_works[i].count, id, drawers[id].get_count());
				float * data = (float*)img_pool.alloc(sizeof(float)*width*height);
				memcpy(data, drawers[id].get_data(), width * height * sizeof(float));
				merger_entry snap = {data, drawers[id].get_count(), local_works[i].count, id};
				_autolock lck(&merger_cs);
				if (merger.size() < 100)
					merger.push_back(snap);
				else
					img_pool.free(data);

				drawers[id].reset();
			}

			if (local_works[i].del)
				wavefrm_pool.free(local_works[i].data);
		}


		zoom_pool.free(tmp);
		zoom_pool.free(tmp16);
	}

	if (destroy)
		return 0;

	goto get_work;
}
