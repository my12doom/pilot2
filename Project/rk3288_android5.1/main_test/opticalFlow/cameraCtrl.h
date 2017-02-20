#ifndef _CAMERACTRL_H
#define _CAMERACTRL_H

#include "Iv4l2.h"
#include <main_test/encoder.h>
#include <libyuv.h>
#include <stdint.h>
#include <stdio.h>
#include <linux/videodev2.h>

#include <unistd.h>
#include <fcntl.h>
//#include <v4l2-ctl.h>
#include<sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/fb.h>

#include <pthread.h>
#include <hardware/rga.h>

#include <rockchip_ion.h>
#include <linux/version.h>
#define BUFPROVIDERCOUNT 4


#define V4L2_BUFFER_MAX             32

#define PAGE_ALIGN(x)   (((x) + 0xFFF) & (~0xFFF)) // Set as multiple of 4K


struct testcase_base_info
{
    char name[32];
    char display_name[68];
    int activated;
    char binary[20];
    int id;
    int category; /* 0: auto, 1: manual */
    int run_type;
};


#define INIT_CMD_PIPE()                                         \
    FILE *cmd_pipe;                                             \
    int test_case_id;                                           \
    if (argc < 4) {                                             \
        db_error("%s: invalid parameter, #%d\n", argv[0], argc);\
        return -1;                                              \
    }                                                           \
    cmd_pipe = fopen(CMD_PIPE_NAME, "w");                       \
    setlinebuf(cmd_pipe);                                       \
    test_case_id = atoi(argv[3])

#define SEND_CMD_PIPE_OK()                                      \
    fprintf(cmd_pipe, "%d 0\n", test_case_id)

#define SEND_CMD_PIPE_OK_EX(exdata)                             \
    fprintf(cmd_pipe, "%d 0 %s\n", test_case_id, exdata)

#define SEND_CMD_PIPE_FAIL()                                    \
    fprintf(cmd_pipe, "%d 1\n", test_case_id)

#define SEND_CMD_PIPE_FAIL_EX(exdata)                           \
    fprintf(cmd_pipe, "%d 1 %s\n", test_case_id, exdata)

#define EXIT_CMD_PIPE()                                         \
    fclose(cmd_pipe)
 

int cameraCreate(int iCamFd);
int cameraStart(int w, int h);
int cameraRun(int phy_addr[4], int buffer_count, int w, int h);
int selectPreferedDrvSize(int *width,int * height,int driver_support_fmt_num);
int capture(uint8_t **pdata);
int dropFrame();

#endif
