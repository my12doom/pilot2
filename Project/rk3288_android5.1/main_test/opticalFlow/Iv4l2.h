#ifndef _IV4L2_H
#define _IV4L2_H
#include <stdint.h>
#include <stdio.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <unistd.h>

#define UAVDEBUG

#ifdef UAVDEBUG
#define LOGE(format,...) fprintf(stdout,format,##__VA_ARGS__)
#else
#define LOGE(format,...)
#endif

#define VIDEODEV "/dev/video0"
#define BUFNUM 4
#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 480

//#define RECORDUAV
typedef struct Ibuffer
{
	void *start;
	int length;
}IBuf;

typedef struct camera_ionbuf_s
{
    void* ion_hdl;
    int   map_fd;
    unsigned long vir_addr;
    unsigned long phy_addr;
    size_t size;
    int share_id;
}camera_ionbuf_t;


int processImage(void *frame);
#endif
