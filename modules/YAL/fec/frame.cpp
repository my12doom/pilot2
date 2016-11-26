#include "frame.h"
#include <stdlib.h>

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
frame *clone_frame(const frame *f)
{
	if (!f)
		return NULL;

	frame * o = alloc_frame(f->payload_size, f->frame_id, f->integrality);
	if (!o)
		return o;

	memcpy(o->payload, f->payload, f->payload_size);

	return o;
}