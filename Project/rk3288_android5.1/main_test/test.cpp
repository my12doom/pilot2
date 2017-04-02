#include <HAL/rk32885.1/AVideo.h>
#include "camera.h"
#include "encoder.h"
#include <libyuv.h>
#include "myx264.h"
#include <YAL/fec/sender.h>

using namespace sensors;
using namespace devices;

static int64_t getus()
{    
   struct timespec tv;  
   clock_gettime(CLOCK_MONOTONIC, &tv);    
   return (int64_t)tv.tv_sec * 1000000 + tv.tv_nsec/1000;    
}


#include <HAL/rk32885.1/Apcap.h>
using namespace androidUAV;

int test_pcap_block_device()
{	
	printf("test_pcap_block_device\n");


	APCAP_TX tx("wlan0", 0);
	FrameSender sender;
	sender.set_block_device(&tx);

	printf("31\n");
	android_video_encoder enc;
	enc.init(640, 360, 250000);
	for(int i=0; i<10; i++)
	{
		uint8_t *ooo = NULL;
		int encoded_size = enc.get_encoded_frame(&ooo);
		printf("%d\n", i);
		void *live = enc.get_next_input_frame_pointer();
		printf("%d\n", __LINE__);
		enc.encode_next_frame();
	}
	int frame_count = 0;
	//RK3288Video c;
	//c.init("/dev/video0");
	RK3288Camera51 c;
	c.init(0);
	printf("33\n");
	x264 enc_soft;
	enc_soft.init(640, 360, 250);
	uint8_t * yv12 = new uint8_t[640*480*3/2];
	memset(yv12, 0x80, 640*480*3/2);	// grey, UV=0x80=neutual
	uint8_t * frame_with_size = new uint8_t[1024*1024]; 	// 1Mbyte ought to be enough

	FILE * f = fopen("/data/on.h264", "wb");

	printf("streaming start\n");

	int64_t t = getus();
	while(1)
	{
		// drain live streaming
		uint8_t *ooo = NULL;
		int encoded_size = enc.get_encoded_frame(&ooo);
		if (encoded_size > 0 && ooo)
		{
			int nal_type = ooo[4] & 0x1f;

			//printf("live streaming: %d, %d\n", encoded_size, nal_type);
			memcpy(frame_with_size+4, ooo, encoded_size);
			*(int*)frame_with_size = encoded_size;
			sender.send_frame(frame_with_size, encoded_size+4);			
		}

		// capture new frames
		uint8_t *p = NULL;
		int s = c.get_frame(&p);
		if (s == 0)
		{
			// got frame, copy it out
			if (frame_count == 0)
				t = getus();
			printf("frame:%d, %dfps\n", frame_count, int64_t(frame_count)*1000000/(getus()-t));
			frame_count++;
			memcpy(yv12, p, 640*360);
			c.release_frame(p);

			// feed live streaming encoder
			void *live = enc.get_next_input_frame_pointer();
			if (live)
			{
				memcpy(live, yv12, 640*360*3/2);
				enc.encode_next_frame();
			}
		}
		else
		{
			usleep(1000);
		}
	}

	while(1)
	{
		uint8_t data[10240];


		sender.send_frame(data, sizeof(data));
	}

	return 0;
}


int camera_init()
{
	return 0;

	RK3288Camera51 c;
	c.init(0);

	uint8_t *p = NULL;

	printf("camera start\n");

	uint8_t * nv12 = new uint8_t[640*480*3/2];
	uint8_t * yv12 = new uint8_t[640*480*3/2];

	int64_t v = getus();
	for(int i=0; i<50; i++);
	libyuv::NV12ToI420(nv12, 640, nv12+640*480, 640, yv12, 640, yv12+640*480, 320, yv12+640*480*5/4, 320, 640, 480);
	v = getus() - v;
	printf("nv12 to i420: %d\n", int(v));
	
	delete [] nv12;
	delete [] yv12;

	frame_format fmt;
	c.get_frame_format(&fmt);
	printf("fmt:%dx%d\n", fmt.width, fmt.height);

	v = -1;
	int frame_count = 0;

	uint8_t *y = new uint8_t[1920*1080];
	uint8_t *uv = new uint8_t[960*540*2];
	uint8_t *yuv360 = new uint8_t[640*360*3/2];

	android_video_encoder enc;
	x264 enc_soft;
	enc_soft.init(640, 360, 250);
	enc.init(640, 360, 250000);
	FILE * f = fopen("/data/enc.h264", "wb");

	printf("live streaming encoder\n");

	/*
	android_video_encoder enc2;
	enc2.init(1920, 1080, 3000000);
	FILE * f2 = fopen("/data/hi.h264", "wb");

	printf("hi res encoder\n");

	*/

	// low bandwidth H264 encoder test
	FILE * fyuv = fopen("/data/640.yuv", "rb");

	int64_t t = getus();
	for(int i=0; i<0; i++)
	{
		int j = i % 120;
		if (j>60)
			j = 120 - j;
		fseek(fyuv, j*640*360*3/2 + j*40, SEEK_SET);
		fread(yuv360, 1, 640*360*3/2, fyuv);


		t = getus();

		// drain encoder
		/*
		uint8_t *ooo = NULL;
		int encoded_size = enc.get_encoded_frame(&ooo);
		if (encoded_size > 0 && ooo)
		{
			int nal_type = ooo[4] & 0x1f;

			fwrite(ooo, 1, encoded_size, f);
			fflush(f);
			printf("(hardware)live streaming: %d, %d, %dus\n", encoded_size, i, int(getus() - t));
			t = getus();
		}

		// feed live streaming encoder
		void *live = enc.get_next_input_frame_pointer();
		if (live)
		{
			fread(live, 1, 640*360*3/2, fyuv);
			enc.encode_next_frame();
		}
		*/

		uint8_t nal_out[100000];
		bool IDR = false;
		int nal_size = enc_soft.encode_a_frame(yuv360, nal_out, &IDR);
		int nal_type = nal_out[4] & 0x1f;

		fwrite(nal_out, 1, nal_size, f);
		//fwrite(yuv360, 1, 640*360*3/2, f);

		printf("live streaming: %d, %d, %dus\n", nal_size, i, int(getus() - t));
	}

	//fflush(f);
	//fclose(f);
	//exit(1);

	while(1)
	{
		// drain live streaming
		uint8_t *ooo = NULL;
		int encoded_size = enc.get_encoded_frame(&ooo);
		if (encoded_size > 0 && ooo)
		{
			int nal_type = ooo[4] & 0x1f;

			printf("live streaming: %d, %d\n", encoded_size, nal_type);
			fwrite(ooo, 1, encoded_size, f);
			fflush(f);
		}

		/*

		// drain hi res
		ooo = NULL;
		encoded_size = enc2.get_encoded_frame(&ooo);
		if (encoded_size > 0 && ooo)
		{
			printf("hi res %d\n", encoded_size);
			fwrite(ooo, 1, encoded_size, f2);
			fflush(f2);
		}

		*/

		// capture new frames
		int s = c.get_frame(&p);
		if (s == 0)
		{
			// got frame, copy it out
			if (v == -1)
				v = getus()-1;

			printf("frame:%d, %dfps\n", frame_count, int64_t(frame_count)*1000000/(getus()-v));
			frame_count++;

			c.release_frame(p);
			/*
			memcpy(y, p, 1920*1080);

			libyuv::SplitUVPlane(p + 1920*1080, 1920, uv, 960, uv + 1920*1080/4, 960, 960, 540);

			// scale down for live streaming
			int64_t t = getus();
			libyuv::I420Scale(y, 1920, uv, 960, uv + 1920*1080/4, 960, 1920, 1080,
				yuv360, 640, yuv360 + 640*360, 320, yuv360 + 640*360*5/4, 320, 640, 360, libyuv::kFilterBilinear);
			t = getus() - t;
			//printf("scale=%d\n", int(t));
			*/

			// feed live streaming encoder
			void *live = enc.get_next_input_frame_pointer();
			if (live)
			{
				memcpy(live, p, 640*360*3/2);
				enc.encode_next_frame();
			}

			// feed high res encoder
			/*
			void *hi = enc2.get_next_input_frame_pointer();
			if (hi)
			{
				memcpy(hi, y, 1920*1080);
				memcpy((uint8_t*)hi+1920*1080, uv, 1920*1080/2);
				enc2.encode_next_frame();
			}
			*/
		}

		else
		{
			usleep(10000);
		}

	}

	return 0;
}