//#include <cutils/memory.h>

#include <unistd.h>
//#include <utils/Log.h>

#include <binder/IPCThreadState.h>
//#include <binder/ProcessState.h>
//#include <binder/IServiceManager.h>

#include <android/native_window.h>
#include <gui/Surface.h>
#include <gui/SurfaceComposerClient.h>
#include <gui/ISurfaceComposer.h>
#include <ui/DisplayInfo.h>
#include <ui/GraphicBufferAllocator.h>
//#include <ui/Rect.h>
//#include <ui/Region.h>

#include <hardware/hwcomposer_defs.h>

#include <utils/String8.h>

#include <system/camera.h>
#include <camera/Camera.h>
#include <camera/ICamera.h>
#include <camera/CameraParameters.h>

#include <EGL/egl.h>
#include <GLES/gl.h>
#include <GLES/glext.h>

#include <media/openmax/OMX_IVCommon.h>
#include <media/openmax/OMX_Video.h>
#include <media/stagefright/foundation/AMessage.h>
#include <media/stagefright/foundation/ABuffer.h>
#include <media/stagefright/MediaCodec.h>
#include <media/stagefright/MediaCodecList.h>
#include <media/ICrypto.h>

#include <StagefrightRecorder.h>
#include <media/stagefright/MediaSource.h>
#include <media/stagefright/CameraSource.h>


// net
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <libyuv.h>

#include "encoder.h"


#include <YAL/fec/sender.h>

using namespace android;
using namespace libyuv;

// global
static const String16 processName("camera_test");
sp<Camera> camera;
CameraParameters params;
sp<MediaCodec> codec;
sp<IGraphicBufferProducer> bufferProducer;

//sp<BufferQueue> bq;// = new BufferQueue();
sp<Surface> stc;// = new Surface(bq);

sp<IGraphicBufferProducer> gbp;// = stc->getIGraphicBufferProducer();
sp<IGraphicBufferConsumer> gbc;// = stc->getIGraphicBufferProducer();
sp<CpuConsumer> cc;// = new CpuConsumer(bq, 1);

char ip[30] = "192.168.1.233";

typedef struct _BufferInfo {
	size_t mIndex;
	size_t mOffset;
	size_t mSize;
	int64_t mPresentationTimeUs;
	uint32_t mFlags;
} BufferInfo;


static int64_t getus()    
{    
   struct timespec tv;  
   clock_gettime(CLOCK_MONOTONIC, &tv);    
   return (int64_t)tv.tv_sec * 1000000 + tv.tv_nsec/1000;    
}

// functions
int udp_send_core(const void*data, int size);


int socket_descriptor = -1;
struct sockaddr_in address;
int port = 0xbbb;

int udp_init()
{
	socket_descriptor = socket(AF_INET, SOCK_DGRAM, 0);  
	if (socket_descriptor <0) 
	{      
		printf("error opening udp socket"); 
		return -1;
	}   

	memset(&address, 0, sizeof(address));
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = inet_addr(ip);
	address.sin_port = htons(port);

	return 0;
}

int udp_send_core(const void*data, int size)
{
	if (socket_descriptor < 0)
	{
		printf("udp fail\n");
		return -1;
	}

	//printf("tx %d\n", size);

	sendto(socket_descriptor, data, size, 0, (struct sockaddr *)&address, sizeof(address));
	return 0;
}
static int min(int a, int b)
{
	if (a>b)
		return b;
	return a;
}

int udp_send(const void*data, int size)
{
	int MTU = 1400;
	//printf("sending %d bytes\n", size);

	int sent = 0;
	for(int i=0; i<size; i+=MTU)
	{
		int pack_size = min(MTU, size-i);
		udp_send_core(data+i, pack_size);
		sent += pack_size;
		//printf("pack:%d bytes\n", pack_size);
	}

	//printf("sent %d bytes\n", sent);

	return 0;
}

int udp_ping(int p, int timeout_us)
{
	int64_t t = getus();
	char tmp[1400];
	memcpy(tmp, &p, 4);
	udp_send(tmp, sizeof(tmp));

	int64_t timeout = t + timeout_us;
	int size_of_address = sizeof(address);
	int size = recvfrom(socket_descriptor, (char*)tmp, sizeof(tmp), 0, (struct sockaddr *)&address, &size_of_address);

	printf("t:%d, %d\n", int(getus() - t), *((int*)tmp));

	return *(int*)tmp == p;
}


void send_frame_delimeter()
{
	static const char str[] =  "my12doom's udp H264 streamer.";
	static char *watermark = NULL;

	if (watermark == NULL)
	{
		watermark = (char*)malloc(5+strlen(str)+1);
		watermark[0] = 0;
		watermark[1] = 0;
		watermark[2] = 0;
		watermark[3] = 1;
		watermark[4] = 31;	// should be fine
		strcpy((char*)watermark+5, str);
	}

	udp_send(watermark, strlen(str)+5);
}

class FrameSender2 : public FrameSender
{
public:
	FrameSender2()
	{
	
	}
	~FrameSender2()
	{

	}
	int send_packet(const void *payload, int payload_size)
	{
		udp_send_core(payload, payload_size);

		return 0;
	}
};


int codec_init()
{
	// counting codec
	/*
	static const MediaCodecList * codec_list =  MediaCodecList::getInstance();
	for(int i=0; i<codec_list->countCodecs(); i++)
		if (codec_list->isEncoder(i))
			printf("codec:%s\n", codec_list->getCodecName(i));
	*/

	FILE * f = fopen("/data/out.h264", "wb");
	FILE * fyuv = fopen("/data/out.yuv", "wb");
	if (!f || !fyuv)
	{
		printf("error opening output file\n");
		return -1;
	}

	printf("1\n");
	status_t err;

	sp<ALooper> looper = new ALooper;
    looper->setName("codec_looper");
    looper->start();
	printf("2\n");
	codec = MediaCodec::CreateByType(looper, "video/avc", true);

	printf("codec=%08x\n", codec.get());

	AString name;
	codec->getName(&name);
	printf("codec name = %s\n", name.c_str());

	sp<AMessage> format = new AMessage;
	format->setInt32("width", 1920);
	format->setInt32("height", 1080);
	format->setString("mime", "video/avc");
	format->setInt32("color-format", OMX_COLOR_FormatYUV420Planar);
	format->setInt32("bitrate", 2500000);
	//format->setInt32("bitrate-mode", OMX_Video_ControlRateConstant);
	format->setFloat("frame-rate", 30);
	format->setInt32("i-frame-interval", 1);
	format->setInt32("profile", OMX_VIDEO_AVCProfileHigh);
	format->setInt32("level", OMX_VIDEO_AVCLevel31);

	err = codec->configure(format, NULL, NULL, MediaCodec::CONFIGURE_FLAG_ENCODE);
	//codec->setParameters();

	printf("err3=%d\n", err);

	sp<AMessage> format2;
	codec->getInputFormat(&format2);
	//printf("getInputFormat=%s\n", format2->);

	//err = codec->createInputSurface(&bufferProducer);

	//printf("err4=%d, codec gbp = %08x\n", err, bufferProducer.get());

	codec->start();

	int yuv_data_size = 1920*1080*3/2;
	uint8_t *yuv_data = new uint8_t[yuv_data_size];
	uint8_t *yuv_low = new uint8_t[640*360*3/2];

	Vector<sp<ABuffer> > inputBuffers;
	Vector<sp<ABuffer> > outputBuffers;
	codec->getInputBuffers(&inputBuffers);
	codec->getOutputBuffers(&outputBuffers);




	FrameSender2 sender;
	int64_t time = 0;
	float fps = 0;
	int size = 0;

	int udp_frame = 0;

	for(int i=0; i>=0; i++)
	{
		size_t index = 0;
		err = codec->dequeueInputBuffer(&index, -1);
		if (err != OK)
		{
			printf("error dequeueInputBuffer()\n");
			break;
		}

		int64_t cam_time = getus();

		// get latest frame
		CpuConsumer::LockedBuffer lb;
		status_t err1 = cc->lockNextBuffer(&lb);
		if (OK == err1)
		{
			printf("OK1\n");

			CpuConsumer::LockedBuffer lb2;
			status_t err2 = cc->lockNextBuffer(&lb2);

			if (OK == err2)
			{
				printf("re\n");
				cc->unlockBuffer(lb);
				lb = lb2;
				err1 = err2;
			}
		}
		else
		{
			printf("ERR1\n");

			while(cc->lockNextBuffer(&lb) != OK)
				usleep(100);
		}

		cam_time = getus() - cam_time;

		uint8_t *p = (uint8_t*)lb.data;
		//for(int x=0; x<1920*200; x++)
		//	p[x] = p[x]*(235-16)/255 + 16;
		printf("cam_time=%d, p00=%02x\n", int(cam_time), p[0]);

		//printf("new data :%dx%d, format:%d, ts=%d!\n", lb.stride, lb.height, lb.format, int(lb.timestamp));

		//printf("index = %d, err=%d\n", index, err);
		int64_t enc_time = getus();
		const sp<ABuffer> &dstBuffer = inputBuffers.itemAt(index);

		//printf("dstBuffer.capacity() = %d\n", dstBuffer->capacity());
		dstBuffer->setRange(0, yuv_data_size);

		// copy Y
		memcpy(dstBuffer->data(), lb.data, 1920*1080);

		// copy UV (use livyuv to split plane, due to RK3288 treat all YUV420 request as NV12 )
		SplitUVPlane(lb.data + 1920*1080, 1920, dstBuffer->data() + 1920*1080, 960, dstBuffer->data() + 1920*1080*5/4, 960, 960, 540);

		cc->unlockBuffer(lb);

		// scale down to 640*360 for live streaming
		int64_t s = getus();
		I420Scale(dstBuffer->data(), 1920, dstBuffer->data() + 1920*1080, 960, dstBuffer->data() + 1920*1080*5/4, 960, 1920, 1080,
			yuv_low, 640, yuv_low + 640*360, 320, yuv_low + 640*360*5/4, 320, 640, 360, kFilterBilinear);
		s = getus() - s;

		printf("scale cost %d us\n", int(s));
		//fwrite(yuv_low, 1, 640*360*3/2, fyuv);


		
		//memset(dstBuffer->data()+1920*1080, 0x80, 1920*1080/2);
		//printf("dstBuffer.size() = %d\n", dstBuffer->size());
		//fwrite(lb.data, 1, 1920*1080*3/2, f);


		err = codec->queueInputBuffer(
				index,
				0,
				dstBuffer->size(),
				0,
				0);

		//err = codec->flush();

		if (err != OK)
		{
			printf("error queueInputBuffer(): %d\n", err);
			break;
		}

		BufferInfo info;
		err = codec->dequeueOutputBuffer(
				&info.mIndex,
				&info.mOffset,
				&info.mSize,
				&info.mPresentationTimeUs,
				&info.mFlags,
			    2000);

		int frame_size = 0;
		if (err == OK)
		{
			const sp<ABuffer> &outBuffer = outputBuffers.itemAt(info.mIndex);

			printf("outBuffer = %d, %d\n", outBuffer->offset(), outBuffer->size());
			fwrite(outBuffer->data(), 1, outBuffer->size(), f);

			codec->releaseOutputBuffer(info.mIndex);

			if (time == 0)
				time = getus();
			else
			{
				fps = i*1000000.0f/(getus()-time);
			}

			size += outBuffer->size();

			//udp_send(outBuffer->data(), outBuffer->size());
			//send_frame_delimeter();

			int * data = (int*)malloc(outBuffer->size()+4);
			*data = outBuffer->size();
			memcpy(data+1, outBuffer->data(), outBuffer->size());

			sender.send_frame(data, outBuffer->size()+4);

			free(data);

			udp_frame ++;

			frame_size = outBuffer->size();
		}

		enc_time = getus() - enc_time;

		printf("\rframe %d/%d, frame_size=%d, bitrate=%.3fMbps", i, udp_frame, int(enc_time), (float)size*8/(i/30.0f)/1000000.0f);
		fflush(stdout);
	}

	fclose(f);

	return 0;
}




int camera_init()
{
	camera = Camera::connect(0, processName, Camera::USE_CALLING_UID);
	printf("mCamera = %08x\n", camera.get());

	if (!camera.get())
	{
		printf("error opening camera %d\n");
		return -1;
	}
	params = camera->getParameters();

	BufferQueue::createBufferQueue(&gbp, &gbc);

	stc = new Surface(gbp);

	gbp = stc->getIGraphicBufferProducer();
	cc = new CpuConsumer(gbc, 1);

	printf("STC=%08x, gbp=%08x, cc=%08x\n", stc.get(), gbp.get(), cc.get());


	return 0;
}

int main(int argc, char** argv)
{
	if (argc > 1)
		strcpy(ip, argv[1]);

	printf("IP:%s\n", ip);

    // set up the thread-pool
    sp<ProcessState> proc(ProcessState::self());
    ProcessState::self()->startThreadPool();

    // create a client to surfaceflinger
    sp<SurfaceComposerClient> client = new SurfaceComposerClient();
    //DisplayoutBuffer display;
    //client->getDisplayoutBuffer(client->getBuiltInDisplay(HWC_DISPLAY_PRIMARY), &display);
    sp<IBinder> dtoken(SurfaceComposerClient::getBuiltInDisplay(ISurfaceComposer::eDisplayIdMain));
    DisplayInfo dinfo;
    //获取屏幕的宽高等信息
    status_t status = SurfaceComposerClient::getDisplayInfo(dtoken, &dinfo);
    printf("w=%d,h=%d,xdpi=%f,ydpi=%f,fps=%f,ds=%f\n", dinfo.w, dinfo.h, dinfo.xdpi, dinfo.ydpi, dinfo.fps, dinfo.density);
    if (status)
        return -1;
    sp<SurfaceControl> surfaceControl = client->createSurface(String8("testsurface"), dinfo.w, dinfo.h, PIXEL_FORMAT_RGBA_8888, 0);

    //new added
    sp<Surface> s = surfaceControl->getSurface();
    SurfaceComposerClient::openGlobalTransaction();
    surfaceControl->setLayer(0x40000000);
    surfaceControl->setPosition(0, 0);
    SurfaceComposerClient::closeGlobalTransaction();
    surfaceControl->show();

	camera_init();
	udp_init();

	/*
	for(int i=0; i<50000; i++)
	{
		printf("i=%d, ", i);
		udp_ping(i, 1000000);
	}
	*/

	params.setPreviewSize(1920, 1080);
	//params.setPictureSize(1920, 1080);
	params.setPreviewFormat(CameraParameters::PIXEL_FORMAT_YUV420P);

	camera->setParameters(params.flatten());
	camera->setPreviewTarget(gbp);
	params = camera->getParameters();

	camera->startPreview();

	//printf("camera param: %s\n", params.flatten().string());

	android_video_encoder enc;
	enc.init(640, 480, 2500000);
	uint8_t *m = new uint8_t[1920*1080*3/2];
	memset(m, 0, 1920*1080*3/2);

	FILE * f = fopen("/data/enc.h264", "wb");

	while(1)
	{
		uint8_t *ooo = NULL;
		int encoded_size = enc.get_encoded_frame(&ooo);
		if (encoded_size > 0 && ooo)
		{
			printf("hahaha %d\n", encoded_size);
			fwrite(ooo, 1, encoded_size, f);
			fflush(f);
		}
		
		void *p = enc.get_next_input_frame_pointer();
		if (!p)
		{
			usleep(10000);
			continue;
		}

		memcpy(p, m, 640*480*3/2);
		enc.encode_next_frame();
	}

	codec_init();

    while(0)
    {
		CpuConsumer::LockedBuffer lb;
		status_t ccs = cc->lockNextBuffer(&lb);

		if (OK == ccs)
		{
			printf("new data :%dx%d, format:%d, ts=%d!\n", lb.stride, lb.height, lb.format, int(lb.timestamp));

			ANativeWindow_Buffer ab;
			s->lock(&ab, NULL);
			
			printf("ab: stride=%d, height=%d, format=%d\n", ab.stride, ab.height, ab.format);
			memset(ab.bits, 0xff, ab.stride * 1080 * 4);
			memcpy(ab.bits, lb.data, 1920*1080);

			/*
			for(int y=0; y<480; y++)
				for(int x=0; x<800; x++)
			{
				uint8_t * dst = (uint8_t*)ab.bits + (y*ab.stride+x)*4;
				uint8_t * src = (uint8_t*)lb.data + (y*lb.stride+x);

				dst[0] = src[0];
				dst[1] = src[0];
				dst[2] = src[0];
				dst[3] = 255;
			}
			*/

			s->unlockAndPost();
			cc->unlockBuffer(lb);
		}
		else
		{
			printf("s=%d\n", ccs);
		}

		usleep(10000);
    }

    IPCThreadState::self()->joinThreadPool();

    IPCThreadState::self()->stopProcess();

    return 0;
}

