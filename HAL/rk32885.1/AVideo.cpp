#include "AVideo.h"
#include <camera/CameraParameters.h>

using namespace devices;
static int arm_camera_yuv420_scale_arm(int v4l2_fmt_src, int v4l2_fmt_dst,char *srcbuf, char *dstbuf,int src_w, int src_h,int dst_w, int dst_h,int mirror,int zoom_val);
static int mFrameSizesEnumTable[][2] = {
				{176,144},
				{320,240},
				{352,288},
				{640,480},
				{720,480},
				{800,600},
				{1024,768},
				{1280,720},
				{1280,960},
				{1600,1200},
				{2048,1536},
				{2592,1944},
				{0,0}
};
static int driver_support_size[12][2] = {0x0};
static int64_t gettime()
{
	struct timespec tv;
	clock_gettime(CLOCK_REALTIME, &tv);
	return (int64_t)((tv.tv_sec) * 1000000 + (tv.tv_nsec)/1000);
}
namespace sensors
{

	RK3288Video::RK3288Video()
	{
		iCamFd = -1;
		iIonFd = -1;
		for(int i=0;i<BUFCOUNT;i++)
		{
			m_v4l2Buffer[i] = NULL;
			v4l2Buffer_phy_addr[i] = 0x00;
		}
		is_rk30_plat = RK30_PLAT;
	}

	RK3288Video::~RK3288Video()
	{
		struct v4l2_requestbuffers creqbuf;
		creqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (iCamFd > 0)
		{
			if(ioctl(iCamFd, VIDIOC_STREAMOFF, &creqbuf.type) == -1)
			{
				LOG2("%s VIDIOC_STREAMOFF Failed\n", __FUNCTION__);
			}
			close(iCamFd);
		}
	}

	//return value: 0 init success; 1 already init ; negative value error occur
	int RK3288Video::init(const char *videoPath,frame_format *format)
	{
		int err,size;
		if(!videoPath)
			return -1;
		if(iCamFd > 0)
		{
			return 1;
		}
		iCamFd = open(videoPath,O_RDWR|O_CLOEXEC);
		if(iCamFd < 0)
		{
			LOG2("androidUAV:open camera device %s failed\n",videoPath);	
			return -1;		
		}
		memset(&mCamDriverCapability, 0, sizeof(struct v4l2_capability));
		err = ioctl(iCamFd, VIDIOC_QUERYCAP, &mCamDriverCapability);
		if (err < 0) 
		{
			LOG2("Error opening device unable to query device.\n");
			goto exit;
		} 
		if(strstr((char*)&mCamDriverCapability.card[0], "front") != NULL)
		{
			LOG2("it is a front camera \n!");
		}
		else if(strstr((char*)&mCamDriverCapability.card[0], "back") != NULL)
		{
			LOG2("it is a back camera \n!"); 
		}
		else
		{
			LOG2("it is a usb camera \n!");
		}
		if (mCamDriverCapability.version == RK29_CAM_VERSION_CODE_1 ) 
		{
		    pix_format = V4L2_PIX_FMT_YUV420;
		    LOG2("Current camera driver version: 0.0.1 \n");    
		} 
		else 
		{ 
		    pix_format = V4L2_PIX_FMT_NV12;
		    LOG2("Current camera driver version: %d.%d.%d \n",(mCamDriverCapability.version>>16) & 0xff,(mCamDriverCapability.version>>8) & 0xff,mCamDriverCapability.version & 0xff); 
		}
		if(access("/sys/module/rk29_camera_oneframe", O_RDWR) >=0 )
		{
			is_rk30_plat =  RK29_PLAT;
		    LOG2("it is rk29 platform!\n");
		}
		else if(access("/sys/module/rk30_camera_oneframe", O_RDWR) >=0)
		{
			LOG2("it is rk30 platform!\n");
		}
		else
		{
			LOG2("default as rk30 platform\n");
		}
		for(int i=0;i<BUFCOUNT;i++)
		{
			if(v4l2Buffer_phy_addr[i] !=0)
				goto suc_alloc;
		}

		if(access(PMEM_DEV_NAME, O_RDWR) < 0) 
		{
		    iIonFd = open(ION_DEVICE, O_RDONLY|O_CLOEXEC);
		    if(iIonFd < 0 ) 
		    {
		        LOG2("%s: Failed to open ion device - %s",__FUNCTION__, strerror(errno));
		        iIonFd = -1;
				err = -1;
		        goto exit1;
		    }
		    for(int i=0;i<BUFCOUNT;i++)
			{
			    ionAllocData.len = 0x100000;
			    ionAllocData.align = 4*1024;
			    ionAllocData.heap_id_mask = ION_HEAP(ION_CMA_HEAP_ID);
				ionAllocData.flags = 0;

				err = ioctl(iIonFd, ION_IOC_ALLOC, &ionAllocData);
			    if(err) 
			    {
			        LOG2("%s: ION_IOC_ALLOC failed to alloc 0x%x bytes with error - %s\n", 
						__FUNCTION__, ionAllocData.len, strerror(errno));
					err = -errno;
			        goto exit2;
			    }

			    fd_data.handle = ionAllocData.handle;
			    handle_data.handle = ionAllocData.handle;

			    err = ioctl(iIonFd, ION_IOC_MAP, &fd_data);
			    if(err) 
			    {
			        LOG2("%s: ION_IOC_MAP failed with error - %s",
			                __FUNCTION__, strerror(errno));
			        ioctl(iIonFd, ION_IOC_FREE, &handle_data);
					err = -errno;
			       goto exit2;
			    }
				
			    m_v4l2Buffer[i] = mmap(0,ionAllocData.len,PROT_READ|PROT_WRITE,MAP_SHARED,fd_data.fd, 0);
			    if(m_v4l2Buffer[i] == MAP_FAILED) 
			    {
			        LOG2("%s: Failed to map the allocated memory: %s",
			                __FUNCTION__, strerror(errno));
			        err = -errno;
			        ioctl(iIonFd, ION_IOC_FREE, &handle_data);
			        goto exit2;
			    }
				memset(m_v4l2Buffer[i], 0x00, ionAllocData.len);

				//err = ioctl(fd_data.fd, PMEM_GET_PHYS, &sub);yzm
				phys_data.handle = ionAllocData.handle;
				phys_data.phys = 0;
				data.cmd = ION_IOC_GET_PHYS;
				data.arg = (unsigned long)&phys_data;
				err = ioctl(iIonFd, ION_IOC_CUSTOM, &data);
				if (err < 0) 
				{
			    	LOG2(" ion get phys_data fail !!!!\n");
			        ioctl(iIonFd, ION_IOC_FREE, &handle_data);
			        goto exit2;
				}
				err = ioctl(iIonFd, ION_IOC_FREE, &handle_data);
				if(err)
				{
					LOG2("%s: ION_IOC_FREE failed with error - %s",
			                __FUNCTION__, strerror(errno));
					err = -errno;
				}
				else
			    	LOG2("%s: Successfully allocated 0x%x bytes, mIonFd=%d, SharedFd=%d\n",
			    			__FUNCTION__,ionAllocData.len, iIonFd, fd_data.fd);

				v4l2Buffer_phy_addr[i] = phys_data.phys;
			}
			//run camera
			if(format)
			{
				
			}
			else
			{
				if(cameraRun(v4l2Buffer_phy_addr,BUFCOUNT,VIDEO_WIDTH,VIDEO_HEIGHT) < 0)
				{
					LOG2("androidUAV: Run camera failed\n");
					return -1;
				}
				LOG2("androidUAV:set camera registers...\n");
				sleep(2);
				initOV7740();
				return 0;
			}
		}

	suc_alloc:   
		err = ioctl(iCamFd, VIDIOC_QUERYCAP, &mCamDriverCapability);
	    if (err < 0) 
	    {
	    	LOG2("Error opening device unable to query device.\n");
		    goto exit;
	    }  
		return 0;
	exit3:
		for(int i=0;i<BUFCOUNT;i++)
		{
			munmap(m_v4l2Buffer[i], ionAllocData.len);
		}
	exit2:

		if(iIonFd > 0)
		{
			close(iIonFd);
			iIonFd = -1;
		}
	exit1:
	exit:
		return err;
	}

	// get one frame from the camere's internal queue.
	// pp: out pointer
	// timestamp: out pointer
	// only_latest: if true, discard all frame except the latest one.
	// return: 0 if new frame retrived, 1 if no new data, negative values for error.
	int RK3288Video::get_frame(uint8_t **pp, devices::timestamp *timestamp/*=NULL*/, bool only_latest/* = false*/)
	{
		int err;
		memset(&frameV4l2_g,0,sizeof(struct v4l2_buffer));
		frameV4l2_g.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		frameV4l2_g.memory = V4L2_MEMORY_OVERLAY;
		frameV4l2_g.reserved = 0;
		if(iCamFd > 0)
		{
			if (ioctl(iCamFd, VIDIOC_DQBUF, &frameV4l2_g) < 0)
			{
				LOG2("%s VIDIOC_DQBUF Failed!!! \n",__FUNCTION__);
				return -1;
			}


			int64_t v4l2_time = ((int64_t)(frameV4l2_g.timestamp.tv_sec) * 1000000 + (frameV4l2_g.timestamp.tv_usec));

			printf("time diff:%d,%d, flag=%08x\n", int(gettime()), int(v4l2_time), frameV4l2_g.flags);

			if( (uint8_t*)m_v4l2Buffer[frameV4l2_g.index] )
			{
				*pp = (uint8_t *)(m_v4l2Buffer[frameV4l2_g.index]);
				/*struct timeval tv;
				unsigned long ftime =frameV4l2_g.timestamp.tv_sec*1000000+frameV4l2_g.timestamp.tv_usec;
				unsigned long ftimeNow = 0;
				gettimeofday(&tv,NULL);
				
				ftimeNow = tv.tv_sec*1000000+tv.tv_usec;*/
				//LOG2("time frame %u now %u %u\n",ftime,ftimeNow,ftimeNow-ftime);
				//LOG2("%u %u \n",frameV4l2_g.timestamp.tv_sec,frameV4l2_g.timestamp.tv_usec);
				/*err = arm_camera_yuv420_scale_arm(V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV12,(char*)(m_v4l2Buffer[frameV4l2_g.index]),(char*)imageBuf,
					640,480,
					320,240,
					false,0);
				if(err == 0)
				{
					*pp = imageBuf;
					return 0;
				}
				*/
				return 0;
			}
			else
			{
				return -1;
			}
		}
		else
		{
			return -1;
		}
	}

	// release one frame and add it back to camera's internal queue
	int RK3288Video::release_frame(uint8_t *p)
	{
		if (ioctl(iCamFd,VIDIOC_QBUF,&frameV4l2_g) < 0)
		{
			return -1;
		}
		return 0;
	}


	// get current frame format
	int RK3288Video::get_frame_format(devices::frame_format *format)
	{
		if(format)
		{
			format->width = preview_w;
			format->height = preview_h;
			return 0;
		}
		return -1;
	}
	// get available frame format.
	int RK3288Video::get_available_frame_format(int index, devices::frame_format *format)
	{
		if (index < 0 || index >= 1 || !format)
			return -1;

		return 0;
	}
	int RK3288Video::get_available_frame_format_count()
	{
		return 1;
	}

	// set frame format.
	// return 0 if successed, -1 if failed.
	int RK3288Video::set_frame_format(const devices::frame_format format)
	{
		return -1;
	}

	// set a callback
	int RK3288Video::set_callback(devices::ICameraCallback *cb)
	{
		return -1;
	}
	int RK3288Video::cameraRun(unsigned long phy_addr[4], int buffer_count, int w, int h)
	{
		int err,i = 0;
		int nSizeBytes;
		struct v4l2_format format;
		enum v4l2_buf_type type;
		struct v4l2_requestbuffers creqbuf;

		struct v4l2_format fmt;
		unsigned int mCamDriverFrmWidthMax = 0,mCamDriverFrmHeightMax = 0;
		int driver_support_fmt_num = 0;
		
		//buffer_count = 2;
		if( phy_addr == 0 || buffer_count == 0  ) 
		{
			LOG2(" Video Buf is NULL\n");
			goto  fail_bufalloc;
		}

		/* Try preview format */		
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fmt.fmt.pix.pixelformat= pix_format;
		fmt.fmt.pix.field = V4L2_FIELD_NONE;


		/*picture size setting*/
		fmt.fmt.pix.width = 10000;
		fmt.fmt.pix.height = 10000;
		err = ioctl(iCamFd, VIDIOC_TRY_FMT, &fmt);

		mCamDriverFrmWidthMax = fmt.fmt.pix.width;
		mCamDriverFrmHeightMax = fmt.fmt.pix.height;		

		if (mCamDriverFrmWidthMax > 3264) 
		{
			LOG2("Camera driver support maximum resolution(%dx%d) is overflow 8Mega!",mCamDriverFrmWidthMax,mCamDriverFrmHeightMax);
			mCamDriverFrmWidthMax = 3264;
			mCamDriverFrmHeightMax = 2448;
		}

		/*preview size setting*/
		while(mFrameSizesEnumTable[i][0])
		{
		    if (mCamDriverFrmWidthMax >= mFrameSizesEnumTable[i][0]) 
		    {
		        fmt.fmt.pix.width = mFrameSizesEnumTable[i][0];
		        fmt.fmt.pix.height = mFrameSizesEnumTable[i][1];
		        if (ioctl(iCamFd, VIDIOC_TRY_FMT, &fmt) == 0) 
		        {
		            if ((fmt.fmt.pix.width == mFrameSizesEnumTable[i][0]) && (fmt.fmt.pix.height == mFrameSizesEnumTable[i][1])) 
		            {
		                driver_support_size[driver_support_fmt_num][0] = mFrameSizesEnumTable[i][0];
		                driver_support_size[driver_support_fmt_num][1] = mFrameSizesEnumTable[i][1];
						LOG2("Video size = %dX%d\n",driver_support_size[driver_support_fmt_num][0],driver_support_size[driver_support_fmt_num][1]);
						driver_support_fmt_num++;
		            }
		        }
		    }
		    i++;
		}
		err = selectPreferedDrvSize(&w,&h,driver_support_fmt_num);
	

		/* Set preview format */
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		format.fmt.pix.width = w;
		format.fmt.pix.height = h;
		format.fmt.pix.pixelformat = pix_format;
		format.fmt.pix.field = V4L2_FIELD_NONE;	
		err = ioctl(iCamFd, VIDIOC_S_FMT, &format);
		if ( err < 0 )
		{
			LOG2(" Failed to set VIDIOC_S_FMT\n");
			goto exit1;
		}

		preview_w = w;
		preview_h = h;	
		creqbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		creqbuf.memory = V4L2_MEMORY_OVERLAY;
		creqbuf.count  =  buffer_count /*- 1*/ ; //We will use the last buffer for snapshots.
		if (ioctl(iCamFd, VIDIOC_REQBUFS, &creqbuf) < 0) 
		{
		    LOG2("%s VIDIOC_REQBUFS Failed\n",__FUNCTION__);
		    goto fail_reqbufs;
		}
		LOG2("creqbuf.count = %d\n",creqbuf.count);
		for (i=0;i<(int)creqbuf.count;i++) 
		{
		    struct v4l2_buffer buffer;
		    buffer.type = creqbuf.type;
		    buffer.memory = creqbuf.memory;
		    buffer.index = i;

		    if (ioctl(iCamFd, VIDIOC_QUERYBUF, &buffer) < 0) 
		    {
		        LOG2("%s VIDIOC_QUERYBUF Failed\n",__FUNCTION__);
		        goto fail_loop;
		    }

		    #if CAM_OVERLAY_BUF_NEW
		    //buffer.m.offset = phy_addr + i*buffer.length;
			buffer.m.offset = phy_addr[i];
		    #else
		    buffer.m.offset = phy_addr;
		    #endif

		    err = ioctl(iCamFd, VIDIOC_QBUF, &buffer);
		    if (err < 0) 
		    {
		        LOG2("%s CameraStart VIDIOC_QBUF Failed\n",__FUNCTION__);
		        goto fail_loop;
		    }
		}
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		err = ioctl(iCamFd, VIDIOC_STREAMON, &type);
		if ( err < 0) 
		{
		    LOG2("%s VIDIOC_STREAMON Failed\n",__FUNCTION__);
		    goto fail_loop;
		}

		return 0;

	fail_bufalloc:
	fail_loop:
	fail_reqbufs:

	exit1:
		close(iCamFd);
		iCamFd = -1;
	exit:
		return -1;
	}
	
	int RK3288Video::selectPreferedDrvSize(int *width,int * height,int driver_support_fmt_num)
	{
		int num = driver_support_fmt_num;
		int ori_w = *width,ori_h = *height,pref_w=0,pref_h=0;
		int demand_ratio = (*width * 100) / (*height);
		int32_t total_pix = (*width) * (*height);

		int cur_ratio ,cur_ratio_diff,pref_ratio_diff = 10000;
		int32_t cur_pix ,cur_pix_diff,pref_pix_diff = 8*1000*1000;
		int i = 0;
		//serch for the preferred res
		for(i =0;i<num;i++)
		{
		    cur_ratio = driver_support_size[i][0] * 100 / driver_support_size[i][1];
		    cur_pix   = driver_support_size[i][0] * driver_support_size[i][1];
		    cur_pix_diff = ((total_pix - cur_pix)>0)?(total_pix - cur_pix):(cur_pix - total_pix);
		    cur_ratio_diff = ((demand_ratio - cur_ratio)>0)?(demand_ratio - cur_ratio):(cur_ratio - demand_ratio);
		    //
		    if((cur_pix_diff < pref_pix_diff) && (cur_ratio_diff <=  pref_ratio_diff))
		    {
		        pref_pix_diff = cur_pix_diff;
		        cur_ratio_diff = pref_ratio_diff;
		        pref_w = driver_support_size[i][0];
		        pref_h = driver_support_size[i][1];
		    }
		}
		if(pref_w != 0)
		{
		    *width = pref_w;
		    *height = pref_h;
		    LOG2("%s:prefer res (%dx%d)",__FUNCTION__,pref_w,pref_h);
		}
		else
		{
		    LOG2("WARINING:have not select preferred res!!");
		}
		return 0;
	}
}

static int arm_camera_yuv420_scale_arm(int v4l2_fmt_src, int v4l2_fmt_dst, 
									char *srcbuf, char *dstbuf,int src_w, int src_h,int dst_w, int dst_h,int mirror,int zoom_val)
{
	unsigned char *psY,*pdY,*psUV,*pdUV; 
	unsigned char *src,*dst;
	int srcW,srcH,cropW,cropH,dstW,dstH;
	long zoomindstxIntInv,zoomindstyIntInv;
	long x,y;
	long yCoeff00,yCoeff01,xCoeff00,xCoeff01;
	long sX,sY;
	long r0,r1,a,b,c,d;
	int ret = 0;
	//bool nv21DstFmt = false;
	int nv21DstFmt = false;
	int ratio = 0;
	int top_offset=0,left_offset=0;
	if((v4l2_fmt_src != V4L2_PIX_FMT_NV12) ||
		((v4l2_fmt_dst != V4L2_PIX_FMT_NV12) && (v4l2_fmt_dst != V4L2_PIX_FMT_NV21) )){
		printf("%s:%d,not suppport this format ",__FUNCTION__,__LINE__);
		return -1;
	}

    //just copy ?
    if((v4l2_fmt_src == v4l2_fmt_dst) && (mirror == false)
        &&(src_w == dst_w) && (src_h == dst_h) && (zoom_val == 100)){
        memcpy(dstbuf,srcbuf,src_w*src_h*3/2);
        return 0;
    }/*else if((v4l2_fmt_dst == V4L2_PIX_FMT_NV21) 
            && (src_w == dst_w) && (src_h == dst_h) 
            && (mirror == false) && (zoom_val == 100)){
    //just convert fmt

        cameraFormatConvert(V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, NULL, 
    					    srcbuf, dstbuf,0,0,src_w*src_h*3/2,
    					    src_w, src_h,src_w,
    					    dst_w, dst_h,dst_w,
    						mirror);
        return 0;

    }*/

	if ((v4l2_fmt_dst == V4L2_PIX_FMT_NV21)){
		nv21DstFmt = true;
		
	}

	//need crop ?
	if((src_w*100/src_h) != (dst_w*100/dst_h)){
		ratio = ((src_w*100/dst_w) >= (src_h*100/dst_h))?(src_h*100/dst_h):(src_w*100/dst_w);
		cropW = ratio*dst_w/100;
		cropH = ratio*dst_h/100;
		
		left_offset=((src_w-cropW)>>1) & (~0x01);
		top_offset=((src_h-cropH)>>1) & (~0x01);
	}else{
		cropW = src_w;
		cropH = src_h;
		top_offset=0;
		left_offset=0;
	}

    //zoom ?
    if(zoom_val > 100){
        cropW = cropW*100/zoom_val;
        cropH = cropH*100/zoom_val;
		left_offset=((src_w-cropW)>>1) & (~0x01);
		top_offset=((src_h-cropH)>>1) & (~0x01);
    }

	src = psY = (unsigned char*)(srcbuf)+top_offset*src_w+left_offset;
	//psUV = psY +src_w*src_h+top_offset*src_w/2+left_offset;
	psUV = (unsigned char*)(srcbuf) +src_w*src_h+top_offset*src_w/2+left_offset;

	
	srcW =src_w;
	srcH = src_h;
//	cropW = src_w;
//	cropH = src_h;

	
	dst = pdY = (unsigned char*)dstbuf; 
	pdUV = pdY + dst_w*dst_h;
	dstW = dst_w;
	dstH = dst_h;

	zoomindstxIntInv = ((unsigned long)(cropW)<<16)/dstW + 1;
	zoomindstyIntInv = ((unsigned long)(cropH)<<16)/dstH + 1;
	//y
	//for(y = 0; y<dstH - 1 ; y++ ) {	
	for(y = 0; y<dstH; y++ ) {	 
		yCoeff00 = (y*zoomindstyIntInv)&0xffff;
		yCoeff01 = 0xffff - yCoeff00; 
		sY = (y*zoomindstyIntInv >> 16);
		sY = (sY >= srcH - 1)? (srcH - 2) : sY; 	 
		for(x = 0; x<dstW; x++ ) {
			xCoeff00 = (x*zoomindstxIntInv)&0xffff;
			xCoeff01 = 0xffff - xCoeff00;	
			sX = (x*zoomindstxIntInv >> 16);
			sX = (sX >= srcW -1)?(srcW- 2) : sX;
			a = psY[sY*srcW + sX];
			b = psY[sY*srcW + sX + 1];
			c = psY[(sY+1)*srcW + sX];
			d = psY[(sY+1)*srcW + sX + 1];

			r0 = (a * xCoeff01 + b * xCoeff00)>>16 ;
			r1 = (c * xCoeff01 + d * xCoeff00)>>16 ;
			r0 = (r0 * yCoeff01 + r1 * yCoeff00)>>16;
			
			if(mirror)
				pdY[dstW -1 - x] = r0;
			else
				pdY[x] = r0;
		}
		pdY += dstW;
	}

	dstW /= 2;
	dstH /= 2;
	srcW /= 2;
	srcH /= 2;

	//UV
	//for(y = 0; y<dstH - 1 ; y++ ) {
	for(y = 0; y<dstH; y++ ) {
		yCoeff00 = (y*zoomindstyIntInv)&0xffff;
		yCoeff01 = 0xffff - yCoeff00; 
		sY = (y*zoomindstyIntInv >> 16);
		sY = (sY >= srcH -1)? (srcH - 2) : sY;		
		for(x = 0; x<dstW; x++ ) {
			xCoeff00 = (x*zoomindstxIntInv)&0xffff;
			xCoeff01 = 0xffff - xCoeff00;	
			sX = (x*zoomindstxIntInv >> 16);
			sX = (sX >= srcW -1)?(srcW- 2) : sX;
			//U
			a = psUV[(sY*srcW + sX)*2];
			b = psUV[(sY*srcW + sX + 1)*2];
			c = psUV[((sY+1)*srcW + sX)*2];
			d = psUV[((sY+1)*srcW + sX + 1)*2];

			r0 = (a * xCoeff01 + b * xCoeff00)>>16 ;
			r1 = (c * xCoeff01 + d * xCoeff00)>>16 ;
			r0 = (r0 * yCoeff01 + r1 * yCoeff00)>>16;
		
			if(mirror && nv21DstFmt)
				pdUV[dstW*2-1- (x*2)] = r0;
			else if(mirror)
				pdUV[dstW*2-1-(x*2+1)] = r0;
			else if(nv21DstFmt)
				pdUV[x*2 + 1] = r0;
			else
				pdUV[x*2] = r0;
			//V
			a = psUV[(sY*srcW + sX)*2 + 1];
			b = psUV[(sY*srcW + sX + 1)*2 + 1];
			c = psUV[((sY+1)*srcW + sX)*2 + 1];
			d = psUV[((sY+1)*srcW + sX + 1)*2 + 1];

			r0 = (a * xCoeff01 + b * xCoeff00)>>16 ;
			r1 = (c * xCoeff01 + d * xCoeff00)>>16 ;
			r0 = (r0 * yCoeff01 + r1 * yCoeff00)>>16;

			if(mirror && nv21DstFmt)
				pdUV[dstW*2-1- (x*2+1) ] = r0;
			else if(mirror)
				pdUV[dstW*2-1-(x*2)] = r0;
			else if(nv21DstFmt)
				pdUV[x*2] = r0;
			else
				pdUV[x*2 + 1] = r0;
		}
		pdUV += dstW*2;
	}
	return 0;
}
