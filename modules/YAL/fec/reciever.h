#include "frame.h"

class reciever
{
public:	
	reciever();
	reciever(IFrameReciever *cb);
	~reciever();
	int put_packet(const void *packet, int size);

protected:
	void init();
	int assemble_and_out();

	raw_packet packets[256];
	int current_frame_id;
	int current_packet_count;
	int last_payload_size;
	IFrameReciever *cb;
};

