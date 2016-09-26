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

	raw_packet *packets;
	int current_frame_id;
	int current_packet_count;
	IFrameReciever *cb;
};

