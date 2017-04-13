#include <stdio.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <HAL/rk32885.1/Apcap.h>
#include <YAL/fec/reciever.h>
#include <YAL/fec/oRS.h>
#include <YAL/fec/frame.h>
#include <vector>
#include <pthread.h>

using namespace androidUAV;
using namespace std;
int testRSSpeed();

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

int test_speed()
{
	rsEncoder enc;
	rsDecoder dec;
	
	uint8_t data[15];
	uint8_t codeword[30];
	enc.init(15);
	dec.init(15);
	
	for(int i=0; i<15; i++)
		codeword[i] = data[i] = i;
		
	enc.append_data(data, 15);
	enc.output(codeword+15);
	
	int count = 0;
	
	for(int64_t t = getus(); getus() < t+1000000;)
	{
		uint8_t codeword2[30];
		memcpy(codeword2+3, codeword+3, 25);
		
		//printf("dec=%lld\n", getus());
		dec.correct_errors_erasures(codeword2, 30, 0, NULL);
		//printf("dec=%lld\n", getus());
		
		for(int i=0; i<15; i++)
		{
			if (codeword2[i] !=i)
			{
				printf("error dec\n");
				return -1;
			}
		}
		
		count++;
	}
	
	printf("count=%d\n", count);
	
	return 0;
}


int main(int argc,char** argv)
{
//	testRSSpeed();

	cb * frame_cache = new cb();
	reciever *rec = new reciever(frame_cache);

	APCAP_RX rx("wlan0", 0);

	int64_t last_fps_show = getus();
	int valid = 0;
	int invalid = 0;
	int wifi_byte_counter = 0;
	int frame_byte_counter = 0;
	int keyframe = 0;

	while(1)
	{
		// statics
		if (getus() > last_fps_show + 1000000)
		{
			last_fps_show = getus();
			fprintf(stderr, "%d+%d=%dfps, %dI frame, %d/%d Kbyte/s(%.2f%%), %ddbm\n", valid, invalid, valid + invalid, keyframe,
				frame_byte_counter/1024, wifi_byte_counter/1024, 100.0*wifi_byte_counter/frame_byte_counter, rx.get_latest_rssi());

			valid = 0;
			invalid = 0;
			wifi_byte_counter = 0;
			frame_byte_counter = 0;
			keyframe = 0;
		}

		if (rx.available() == 0)
		{
			usleep(10000);
			continue;
		}

		// feed reciever with pcap packets
		uint8_t data[4096];
		int size = 0;
		for(int i=rx.available(); i>0; i--)
		{
			size=rx.read(data, 4096);
			rec->put_packet(data, size);
			wifi_byte_counter += size;
		}
		
		//printf("112\n");

		frame * f = frame_cache->get_frame();
		if (!f)
		{
			usleep(10000);
			continue;
		}

		if (!f->integrality)
		{
			invalid ++;
			release_frame(f);
			continue;
		}

		int frame_size = *(int*)f->payload;
		uint8_t * frame_data = (uint8_t*)f->payload+4;

		if (argc>1)
		{
			fwrite(frame_data, 1, frame_size, stdout );
			fflush(stdout);
		}

		if (frame_size>f->payload_size-4)
			continue;

		if ((frame_data[4] & 0x1f) == 7)
			keyframe ++;
		valid ++;
		frame_byte_counter += f->payload_size;
		release_frame(f);
	}

	return 0;
}

int GetTickCount()
{
	return	getus()/1000;
}

#include <assert.h>
#include <stdlib.h>

using namespace std;
int testRSSpeed()
{
	printf("testRSSpeed\n");
	init_exp_table();
	for(int i=0; i<256; i++)
	{
		assert(gexp[i] == gexp[i+255]);
	}	

	static bool f = false;
	if (f)
		return 0;
	f = true;

	const int NPAR = 23;

	int erasures[NPAR];
	int nerasures = 0;

	int msg_length = NPAR * 10;

	unsigned char *msg = new unsigned char[msg_length];
	for(int i=0; i<msg_length; i++)
	{
		msg[i] = i;
	}

	rsEncoder enc(23);
	int l = GetTickCount();
	int byteDone = 0;
	for (int i=0; i<100000; i++)
	{
		byteDone += msg_length;
		enc.resetData();
		enc.append_data(msg, msg_length);

		if (GetTickCount()-l>0 && (i % 5000 == 0) )
		{
			printf("\rEncode speed:%d KByte/s, %d.", byteDone/ (GetTickCount()-l), byteDone);
			fflush(stdout);
		}
	}

	printf("\rEncode speed:%d KByte/s.", byteDone/ max(1,(GetTickCount()-l)));

	printf("\ndone\n");

	//return 0;
decode_test:
	unsigned char parity[NPAR];
	enc.output(parity);

	unsigned char data[256];

	rsDecoder dec(NPAR);
	byteDone = 0;
	l=GetTickCount();
	for (int i=0; i<1000000; i++)
	{
		byteDone += msg_length;
		nerasures=0;
		memcpy(data, msg, msg_length);
		memcpy(data+msg_length, parity, NPAR);

		for(int j=5; j<5+NPAR; j++)
		{
			data[j] ^= 0x33;
			erasures[nerasures++] = j;
		}

		/*
		data[9] ^= 0x33;
		data[55] ^= 0x33;
		data[75] ^= 0x33;

		erasures[nerasures++] = 5;
		erasures[nerasures++] = 9;
		erasures[nerasures++] = 55;
		erasures[nerasures++] = 75;
		*/


		//int result = dec.decode_data(data, msg_length+NPAR);
		dec.correct_errors_erasures(data, NPAR+msg_length, nerasures, erasures);
		if (memcmp(msg, data, msg_length) !=0)printf("\nsome error..\n");
		if (GetTickCount()-l>0 && (i % 5000 == 0) )
		{
			printf("\rDecode speed:%d KByte/s.", byteDone/ (GetTickCount()-l));
			fflush(stdout);
		}
	}
	printf("\ndone\n");

	//clear_genpoly_cache();
	delete [] msg;
	//getch();
	exit(0);
}


