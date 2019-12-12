#include <stdio.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <HAL/rk32885.1/Apcap.h>
#include <YAL/fec/reciever.h>
#include <YAL/fec/frame.h>
#include <vector>
#include <pthread.h>
#include <math.h>
#include <YAL/fec/sender.h>
#include <Protocol/crc32.h>

using namespace androidUAV;
using namespace std;

// function prototypes
int H264_slicer(const char *file);

// global variables
bool verbose = false;

typedef struct
{
	uint8_t *nal;
	int size;
	uint8_t nal_type;
} H264_entry;

H264_entry frames[500000] = {0};
int frame_count=0;


static int64_t getus()
{    
   struct timespec tv;  
   clock_gettime(CLOCK_MONOTONIC, &tv);    
   return (int64_t)tv.tv_sec * 1000000 + tv.tv_nsec/1000;    
}

int main_tx(int argc, char **argv)
{
	char interface[1024];
	if (argc>1)
		strcpy(interface, argv[1]);

	bool low_rate = false;
	for(int i=0; i<argc; i++)
		if (strcmp(argv[i], "-l") == 0)
			low_rate = true;


	APCAP_TX tx(interface, 0);

	int64_t last_tx = 0;
	FrameSender sender;
	sender.set_block_device(&tx);
	if (!low_rate)
	{
		tx.set_mcs_bw(1, 40);
		sender.config(PACKET_SIZE, 0.5);
	}
	else
	{
		tx.set_mcs_bw(0, 20);
		sender.config(PACKET_SIZE, 4.0);
	}

	H264_slicer(low_rate ? "test.264" : "DJI_0033L.264");

	uint8_t *buf = new uint8_t[655360];

	for(int i=0; ; i=(i+1)%frame_count)
	{
		if (getus() < last_tx + 33333)
		{
			while(getus() < last_tx + 33333)
				usleep(2000);
		}
		else
		{
			printf("not enough air rate\n");
		}

		last_tx = getus();
		sender.send_frame(frames[i].nal, frames[i].size);

	}

	return 0;
}


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


int main(int argc,char** argv)
{
	cb * frame_cache = new cb();
	reciever *rec = new reciever(frame_cache);

	char interface[1024] = "wlx70f11c17eedc";
	if (argc>1)
		strcpy(interface, argv[1]);


	char tmp[200];
	sprintf(tmp, "service network-manager stop", interface);
	system(tmp);
	sprintf(tmp, "ifconfig %s down", interface);
	system(tmp);
	sprintf(tmp, "iw dev %s set monitor otherbss fcsfail", interface);
	system(tmp);
	sprintf(tmp, "ifconfig %s up", interface);
	system(tmp);
	sprintf(tmp, "iw dev %s set channel 100 HT40+", interface);
	system(tmp);

	for(int i=0; i<argc; i++)
	{
		if (strstr(argv[i], "-t"))
			return main_tx(argc, argv);
	}
	APCAP_RX rx(interface, 0);

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

			if (verbose)
			{
				printf("(%d	)feed: %d bytes\n", int(getus()/1000000), size);
				for(int i=0; i<size; i++)
					printf("%02x,", data[i]);
				printf("\n");
			}
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


int H264_slicer(const char *file)
{
	FILE * f = fopen(file, "rb");
	if (!f)
		return -1;

	fseek(f, 0, SEEK_END);
	int size = ftell(f);
	uint8_t *p = new uint8_t[size];
	fseek(f, 0, SEEK_SET);
	int got = fread(p, 1, size, f);
	fclose(f);

	frame_count = 0;

	uint32_t delimiter = 0x010000;
	frames[0].nal = p;
	frames[0].nal_type = 5;
	for (int i = 0; i < size - 4; i++)
	{
		uint32_t p32 = *(uint32_t*)(p + i);
		int nal_type = p[i + 3] & 0x1f;
		if ((p32&0xffffff) == delimiter && (nal_type == 1 || nal_type == 5))
		{
			frames[frame_count+1].nal_type = nal_type;
			frames[frame_count+1].nal = p + i;
			
			if (frame_count > 0)
				frames[frame_count].size = p + i - frames[frame_count].nal;
			else
				frames[frame_count].size = i;

			frame_count++;
			//printf("%d, %d\n", i, frames[frame_count-1].size);
		}
	}

	frames[frame_count-1].size = p + size - frames[frame_count-1].nal;

	int Isize = 0;
	for (int i = 0; i < frame_count; i++)
		if (frames[i].nal_type == 5)
			Isize += frames[i].size;

	printf("ISize:%d, %.2f%%\n", Isize, Isize * 100.0f / size);


	f = fopen("h264size.csv", "wb");
	for (int i = 0; i < frame_count; i++)
	{
		fprintf(f, "%d,%d,%d\n", i, frames[i].size, frames[i].nal_type);
	}
	fclose(f);

	return 0;
}
