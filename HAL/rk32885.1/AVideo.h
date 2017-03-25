#pragma once
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include <HAL/Interface/ICamera.h>
#include <gui/CpuConsumer.h>
#include <HAL/rk32885.1/ALog.h>
#include <HAL/rk32885.1/OV7740Control.h>

#include <linux/videodev2.h>
#include <linux/fb.h>
#include <linux/version.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <pthread.h>
#include <hardware/rga.h>

#include <rockchip_ion.h>

//v4l2 buffer number
#define BUFCOUNT 2

#define PAGE_ALIGN(x)   (((x) + 0xFFF) & (~0xFFF)) // Set as multiple of 4K

#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 480

#define RK30_PLAT 1
#define RK29_PLAT 0

#define VIDEODEV 			"/dev/video0"
#define VIDEO_DEV_NAME   	"/dev/video0"
#define PMEM_DEV_NAME    	"/dev/pmem_cam"
#define DISP_DEV_NAME    	"/dev/graphics/fb1"
#define ION_DEVICE       	"/dev/ion"
#define CAMSYS_DEVNAME   	"/dev/camsys_marvin"

#define FBIOSET_ENABLE		0x5019	


#define CAM_OVERLAY_BUF_NEW  1
#define RK29_CAM_VERSION_CODE_1 KERNEL_VERSION(0, 0, 1)
#define RK29_CAM_VERSION_CODE_2 KERNEL_VERSION(0, 0, 2)
#define  FB_NONSTAND ((is_rk30_plat == RK29_PLAT)?0x2:0x20)

namespace sensors
{
	class RK3288Video : public devices::ICamera
	{
	public:
		RK3288Video();
		~RK3288Video();
		//return value: 0 init success; 1 already init ; negative value error occur
		int init(const char *videoPath,devices::frame_format *format=NULL);

		// get one frame from the camere's internal queue.
		// pp: out pointer
		// timestamp: out pointer
		// only_latest: if true, discard all frame except the latest one.
		// return: 0 if new frame retrived, 1 if no new data, negative values for error.
		virtual int get_frame(uint8_t **pp, devices::timestamp *timestamp=NULL, bool only_latest = false);
		int cameraRun(unsigned long phy_addr[4], int buffer_count, int w, int h);
		int selectPreferedDrvSize(int *width,int * height,int driver_support_fmt_num);
		// release one frame and add it back to camera's internal queue
		virtual int release_frame(uint8_t *p);

		// get current frame format
		virtual int get_frame_format(devices::frame_format *format);

		// get available frame format.
		virtual int get_available_frame_format(int index, devices::frame_format *format);
		virtual int get_available_frame_format_count();

		// set frame format.
		// return 0 if successed, -1 if failed.
		virtual int set_frame_format(const devices::frame_format format);

		// set a callback
		virtual int set_callback(devices::ICameraCallback *cb);

		// return false if any error/waning
		virtual bool healthy() { return _healthy;}
	protected:
		bool _healthy;
	private:
		//v4l2
		int iCamFd;
		int iIonFd;
		int preview_w;
		int preview_h;
		uint32_t pix_format;
		void *m_v4l2Buffer[BUFCOUNT];
		uint8_t imageBuf[1024*1024*3];
		unsigned long v4l2Buffer_phy_addr[BUFCOUNT];
		struct v4l2_capability mCamDriverCapability;
		struct v4l2_buffer frameV4l2_g;
		//ion
		struct ion_allocation_data ionAllocData;
		struct ion_fd_data fd_data;
		struct ion_handle_data handle_data;

		struct ion_phys_data  phys_data;
		struct ion_custom_data data;
		//platform
		int is_rk30_plat;
	};
}
