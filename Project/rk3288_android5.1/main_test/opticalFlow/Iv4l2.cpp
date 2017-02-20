#include <fcntl.h>
#include <stdio.h>
#include "Iv4l2.h"
#include "cameraCtrl.h"
#include "checkCapablities.h"

int videofd  = 0;
struct v4l2_capability videoParameter;
struct v4l2_fmtdesc	   videoSupportFmt;
struct v4l2_format     videoFmt;
struct v4l2_requestbuffers reqBuff;
int				type;
struct v4l2_buffer *v4l2buf;
uint32_t 				pixFormat;
IBuf buffers[BUFNUM];
int videoWidth_g = 0;
int videoHeight_g = 0;

extern int mCamFd;

using namespace android;
android_video_encoder enc2;

int main(int argc,char **argv)
{
	int ret,framecount = 0;
#ifdef RECORDUAV
		FILE *ff = fopen("/data/enc.h264", "wb");
		enc2.init(VIDEO_WIDTH, VIDEO_HEIGHT, 2500000);
#endif
	//video DriverCapability test
	//capablityGet(VIDEODEV);
	
	videofd = open(VIDEODEV,O_RDWR|O_CLOEXEC);
	ret = cameraCreate(videofd);
	if(ret < 0)
	{
		LOGE("create camera failed\n");
		return -1;
	}
	cameraStart(VIDEO_WIDTH,VIDEO_HEIGHT);
	
	
	mCamFd = videofd;

	while(1)
	{
		uint8_t  *pdata;
		ret = capture(&pdata);
		
		if(ret == 0 && pdata)
		{
#ifdef RECORDUAV
			uint8_t *ooo = NULL;
			int encoded_size = enc2.get_encoded_frame(&ooo);
			if (encoded_size > 0 && ooo)
			{
				int nal_type = ooo[4] & 0x1f;
				fwrite(ooo, 1, encoded_size, ff);
				printf("write\n");
				//fflush(ff);
			}
#endif
			framecount++;
			printf("count %d\n",framecount);
#ifdef RECORDUAV
			void *live = enc2.get_next_input_frame_pointer();
			printf("count %d\n",framecount);
			if (live)
			{
				memcpy(live, pdata, VIDEO_WIDTH*VIDEO_HEIGHT);
				printf("memcpy\n");
				enc2.encode_next_frame();
			}
			
#endif
			ret = dropFrame();
			if(ret < 0)
				printf("drop error\n");
			usleep(20000);
		}
		else
		{
			printf("cap error\n");
		}
	}
	return 0;
}

int processImage(void *frame)
{
	if(!frame)
		return -1;
	printf("success \n");
	for(int i=0;i<10;i++)
	{
		uint8_t *ptr = (uint8_t *)frame + i;
		for(int j=0;j<10;j++)
		{
			printf("%d ",ptr[j]);
		}
		printf("\n");
	}
}

