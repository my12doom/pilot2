#ifndef _CHECKCAP_H
#define _CHECKCAP_H
#include <stdio.h>  
#include <string.h>  
#include <errno.h>  
#include <stdlib.h>  
#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h>  
#include <time.h>  
#include <sys/mman.h>  
#include <assert.h>  
#include <linux/videodev2.h>  
#include <linux/fb.h>
void enum_video_ctrls_and_menus(int fd);
int capablityGet(char* devName);

#endif
