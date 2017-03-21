#include "camera.h"

#include <binder/IPCThreadState.h>


#include "camera.h"

using namespace android;
using namespace devices;
using namespace sensors;


sp<ProcessState> proc(ProcessState::self());

bool thread_pool_run = false;

namespace sensors
{

	RK3288Camera51::RK3288Camera51()
	{
		tbl_count = 0;

		// set up the thread-pool
		if (!thread_pool_run)
			ProcessState::self()->startThreadPool();

		thread_pool_run = true;
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

		printf("Camera::getNumberOfCameras() = %d\n", count);

		camera = Camera::connect(camera_id, String16("YetAnotherPilot"), Camera::USE_CALLING_UID);

		if (!camera.get())
			return -2;

		BufferQueue::createBufferQueue(&gbp, &gbc);
		cc = new CpuConsumer(gbc, 1);

		params = camera->getParameters();
		params.setPreviewSize(640, 480);
		//params.setPreviewSize(3120, 3120);
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
				add_table(*pp, lb);
				return 0;
			}

			CpuConsumer::LockedBuffer lb2;
			status_t err2 = cc->lockNextBuffer(&lb2);

			if (OK == err2)
			{
				cc->unlockBuffer(lb);
				*pp = lb2.data;
				add_table(*pp, lb);
				return 0;
			}
			else
			{
				*pp = lb.data;
				add_table(*pp, lb);
				return 0;
			}
		}
		else
		{
			return 1;
		}
	}

	// release one frame and add it back to camera's internal queue
	int RK3288Camera51::release_frame(uint8_t *p)
	{
		CpuConsumer::LockedBuffer *lb = find_table(p);

		if (!lb)
			return -1;

		for(int i=0; i<tbl_count; i++)
		{
			if (p_tbl[i] == p)
			{
				p_tbl[i] = NULL;
				memmove(p_tbl+i, p_tbl+i+1, (tbl_count-i-1) * sizeof(uint8_t*));
				memmove(lb_tbl+i, lb_tbl+i+1, (tbl_count-i-1) * sizeof(CpuConsumer::LockedBuffer *));

				tbl_count --;
			}
		}
		cc->unlockBuffer(*lb);

		return 0;
	}

	int RK3288Camera51::add_table(uint8_t *p, android::CpuConsumer::LockedBuffer lb)
	{
		if (tbl_count >= 16)
			return -1;

		if (find_table(p))
			return 0;
		
		lb_tbl[tbl_count] = lb;
		p_tbl[tbl_count++] = p;
		
		return 0;
	}

	android::CpuConsumer::LockedBuffer * RK3288Camera51::find_table(uint8_t *p)
	{
		for(int i=0; i<tbl_count; i++)
			if (p_tbl[i] == p)
				return &lb_tbl[i];
		return NULL;
	}

	// get current frame format
	int RK3288Camera51::get_frame_format(devices::frame_format *format)
	{
		params.getPreviewSize(&format->width, &format->height);

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
