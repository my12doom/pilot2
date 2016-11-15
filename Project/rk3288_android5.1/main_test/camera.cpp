#include "camera.h"

#include <system/camera.h>
#include <camera/Camera.h>
#include <camera/ICamera.h>
#include <camera/CameraParameters.h>
#include <camera/ICameraService.h>

#include <ui/GraphicBufferAllocator.h>
#include <gui/Surface.h>
#include <gui/CpuConsumer.h>

#include <binder/IPCThreadState.h>


using namespace android;
using namespace devices;
using namespace sensors;

sp<Camera> camera;
CameraParameters params;
sp<IGraphicBufferProducer> gbp;
sp<IGraphicBufferConsumer> gbc;
sp<CpuConsumer> cc;
sp<ProcessState> proc(ProcessState::self());

static int64_t getus()
{    
   struct timespec tv;  
   clock_gettime(CLOCK_MONOTONIC, &tv);    
   return (int64_t)tv.tv_sec * 1000000 + tv.tv_nsec/1000;    
}

int camera_init()
{
	// set up the thread-pool
    ProcessState::self()->startThreadPool();


	RK3288Camera51 c;

	uint8_t *p = NULL;
	int i;

	printf("camera start\n", i++);

	while(1)
	{
		int s = c.get_frame(&p);

		if (s == 0)
			printf("frame:%d\n", i++);
	}

	return 0;
}

namespace sensors
{

	RK3288Camera51::RK3288Camera51()
	{
		init(0);
	}

	RK3288Camera51::~RK3288Camera51()
	{
	}

	int RK3288Camera51::init(int camera_id)
	{
		printf("RK3288Camera51::init(%d)\n", camera_id);
		int count = Camera::getNumberOfCameras();
		if (camera_id < 0)
		{
			if (count < 2)
				return -1;
			camera_id = 1;
		}

		camera = Camera::connect(camera_id, String16("YetAnotherPilot"), Camera::USE_CALLING_UID);

		if (!camera.get())
			return -2;

		BufferQueue::createBufferQueue(&gbp, &gbc);
		cc = new CpuConsumer(gbc, 3);

		params = camera->getParameters();
		params.setPreviewSize(1920, 1080);
		//params.setPictureSize(3120, 3120);
		params.setPreviewFormat(CameraParameters::PIXEL_FORMAT_YUV420P);
		camera->setParameters(params.flatten());
		camera->setPreviewTarget(gbp);
		params = camera->getParameters();
		camera->startPreview();

		printf("open camera %d done, gbp=%08x,gbc=%08x,cc=%08x\n", camera_id, gbp.get(), gbc.get(), cc.get());

		/*
		Vector<Size> sizes;
		params.getSupportedPreviewSizes(sizes);
		for(int i=0; i<sizes.size(); i++)
			printf("preview res:%dx%d\n", sizes[i].width, sizes[i].height);

		int minfps, maxfps;
		params.getPreviewFpsRange(&minfps, &maxfps);
		printf("%d-%dfps, format:%s\n", minfps, maxfps, params.getPreviewFormat());

		params.getSupportedVideoSizes(sizes);
		for(int i=0; i<sizes.size(); i++)
			printf("video res:%dx%d\n", sizes[i].width, sizes[i].height);

		params.getSupportedPictureSizes(sizes);
		for(int i=0; i<sizes.size(); i++)
			printf("pic res:%dx%d\n", sizes[i].width, sizes[i].height);
		
		Vector<int> formats;
		params.getSupportedPreviewFormats(formats);
		for(int i=0; i<formats.size(); i++)
			printf("preview format:%d\n", formats[i]);
		*/

		return 0;
	}

	// get one frame from the camere's internal queue.
	// pp: out pointer
	// timestamp: out pointer
	// only_latest: if true, discard all frame except the latest one.
	// return: 0 if new frame retrived, 1 if no new data, negative values for error.
	int RK3288Camera51::get_frame(uint8_t **pp, devices::timestamp *timestamp/*=NULL*/, bool only_latest/* = false*/)
	{
		CpuConsumer::LockedBuffer lb;
		status_t err1 = cc->lockNextBuffer(&lb);
		if (OK == err1)
		{
			if (!only_latest)
			{
				*pp = lb.data;
				return 0;
			}

			CpuConsumer::LockedBuffer lb2;
			status_t err2 = cc->lockNextBuffer(&lb2);

			if (OK == err2)
			{
				cc->unlockBuffer(lb);
				*pp = lb2.data;
				return 0;
			}
			else
			{
				*pp = lb.data;
				return 0;
			}
		}
		else
		{
			return 1;
		}
	}

	// get current frame format
	int RK3288Camera51::get_frame_format(devices::frame_format *format)
	{
		return 0;
	}

	// get available frame format.
	int RK3288Camera51::get_available_frame_format(int index, devices::frame_format *format)
	{
		if (index < 0 || index >= 1 || !format)
			return -1;

		return 0;
	}
	int RK3288Camera51::get_available_frame_format_count()
	{
		return 1;
	}

	// set frame format.
	// return 0 if successed, -1 if failed.
	int RK3288Camera51::set_frame_format(const devices::frame_format format)
	{
		return -1;
	}

	// set a callback
	int RK3288Camera51::set_callback(devices::ICameraCallback *cb)
	{
		return -1;
	}

}
