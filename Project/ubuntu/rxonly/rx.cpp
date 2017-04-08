#include <stdio.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <HAL/rk32885.1/Apcap.h>
#include <YAL/fec/reciever.h>
#include <YAL/fec/frame.h>
#include <vector>
#include <pthread.h>

using namespace androidUAV;
using namespace std;

class cb : public IFrameReciever
{
public:
	cb()
	{
		pthread_mutex_init(&_cs, NULL);
	}
	~cb()
	{
		pthread_mutex_destroy(&_cs);
	}
	int handle_event()
	{
		return 0;
	}
	int handle_frame(const frame &f)
	{
 		//printf("frame.v=%d, %d/%d bytes\n", f.integrality, *(int*)f.payload, f.payload_size);
		frame * _frame = clone_frame(&f);

		pthread_mutex_lock(&_cs);

		frames.push_back(_frame);

		pthread_mutex_unlock(&_cs);
		return 0;
	}
	frame * get_frame()
	{
		pthread_mutex_lock(&_cs);

		if (frames.size() == 0)
		{
			pthread_mutex_unlock(&_cs);
			return NULL;
		}

		frame * f = frames[0];
		frames.erase(frames.begin());

		pthread_mutex_unlock(&_cs);

		return f;
	}
	pthread_mutex_t _cs;
	vector<frame*> frames;
};

static int64_t getus()
{    
   struct timespec tv;  
   clock_gettime(CLOCK_MONOTONIC, &tv);    
   return (int64_t)tv.tv_sec * 1000000 + tv.tv_nsec/1000;    
}

int main(int argc,char** argv)
{
	cb * frame_cache = new cb();
	reciever *rec = new reciever(frame_cache);

	APCAP_RX rx("wlan0", 0);

	int64_t last_fps_show = getus();
	int valid = 0;
	int invalid = 0;
	int wifi_byte_counter = 0;
	int frame_byte_counter = 0;

	while(1)
	{
		// statics
		if (getus() > last_fps_show + 1000000)
		{
			last_fps_show = getus();
			fprintf(stderr, "%d+%d=%dfps, %d/%d Kbyte/s(%.2f%%), %ddbm\n", valid, invalid, valid + invalid, 
				frame_byte_counter/1024, wifi_byte_counter/1024, 100.0*wifi_byte_counter/frame_byte_counter, rx.get_latest_rssi());

			valid = 0;
			invalid = 0;
			wifi_byte_counter = 0;
			frame_byte_counter = 0;
		}

		if (rx.available() == 0)
		{
			usleep(10000);
			continue;
		}

		// feed reciever with pcap packets
		uint8_t data[4096];
		int size = 0;
		while((size=rx.read(data, 4096)) > 0)
		{
			rec->put_packet(data, size);
			wifi_byte_counter += size;
		}

		frame * f = frame_cache->get_frame();
		if (!f)
		{
			usleep(10000);
			continue;
		}

		frame_byte_counter += f->payload_size;

		if (!f->integrality)
		{
			invalid ++;
			release_frame(f);
			continue;
		}

		int frame_size = *(int*)f->payload;
		uint8_t * frame_data = (uint8_t*)f->payload+4;

		//fwrite(frame_data, 1, frame_size, stdout );
		fflush(stdout);

		if (frame_size>f->payload_size-4)
			continue;

		valid ++;		
		release_frame(f);
	}

	return 0;
}
