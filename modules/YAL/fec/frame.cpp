#include "frame.h"
#include <memory.h>

void release_frame(frame *f)
{
	if (f && f->payload)
		free(f->payload);
	if (f)
		free(f);
}
frame *alloc_frame(int payload_size, int frame_id/* = 0*/, bool integrality/* = false*/)
{
	frame *o = (frame*)malloc(sizeof(frame));
	o->payload = (uint8_t*)malloc(payload_size);
	o->payload_size = payload_size;
	o->frame_id = frame_id;
	o->integrality = integrality;

	return o;
}