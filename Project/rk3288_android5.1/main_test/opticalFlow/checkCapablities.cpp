#include "checkCapablities.h"

  
#define CLEAR(x)    memset(&(x), 0, sizeof(x))  
typedef unsigned char BYTE;  
typedef unsigned short WORD;  
typedef unsigned int DWORD;  
typedef long LONG;  
  
#define CHECKNUM 8  
  
struct {  
unsigned int type;  
char *name;  
} enum_fmt[]={  
{V4L2_CAP_VIDEO_CAPTURE, "V4L2_CAP_VIDEO_CAPTURE"},  
{V4L2_CAP_VIDEO_OUTPUT, "V4L2_CAP_VIDEO_OUTPUT"},  
{V4L2_CAP_VIDEO_OVERLAY, "V4L2_CAP_VIDEO_OVERLAY"},  
{V4L2_CAP_VIDEO_CAPTURE_MPLANE, "V4L2_CAP_VIDEO_CAPTURE_MPLANE"},  
{V4L2_CAP_VIDEO_OUTPUT_MPLANE, "V4L2_CAP_VIDEO_OUTPUT_MPLANE"},  
{V4L2_CAP_VIDEO_M2M_MPLANE, "V4L2_CAP_VIDEO_M2M_MPLANE"},  
{V4L2_CAP_VIDEO_M2M, "V4L2_CAP_VIDEO_M2M"},  
{V4L2_CAP_STREAMING, "V4L2_CAP_STREAMING"},  
};  
  
  
  
int open_camer_device(char *path)  
{  
    int fd;  
  
    if((fd = open(path,O_RDWR | O_NONBLOCK)) < 0)  
    {  
        perror("Fail to open");  
        exit(EXIT_FAILURE);  
    }   
  
    printf("open cam:%s success %d\n",path, fd);  
    return fd;  
}  
  
void enum_video_ctrls_and_menus(int fd){  
 /* 
  \* To query the attributes of a control applications set the id field of a struct  
  \* v4l2_queryctrl and call the VIDIOC_QUERYCTRL ioctl with a pointer to this 
  \* structure. The driver fills the rest of the structure or returns an EINVAL  
  \* error code when the id is invalid. 
  \* 
  \* To enumerate controls call VIDIOC_QUERYCTRL with successive 
  \* id values starting from V4L2_CID_BASE up to and exclusive V4L2_CID_LASTP1,  
  \* or starting from V4L2_CID_PRIVATE_BASE until the driver returns EINVAL.  
  */   
  struct v4l2_queryctrl queryctrl;    /* Query Control structure as defined in <sys/videodev2.h> */  
  struct v4l2_querymenu querymenu;    /* Query Menu Structure as defined in <sys/videodev2.h> */  
  
  fprintf(stdout,"Discovering controls:\n");  
   
  
  memset (&queryctrl, 0, sizeof (queryctrl));    
  for (queryctrl.id = V4L2_CID_BASE; queryctrl.id < V4L2_CID_LASTP1; queryctrl.id++) 
  {  
      if ( ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl) == 0 ) {  
          /* Check to see if this control is permanently disabled and should be ignored by the application */  
          if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)  
              continue;  
          /* We got a video control back */  
          fprintf(stdout,"\nVIDIOC_QUERYCTRL(V4L2_CID_BASE+%d)\n", queryctrl.id-V4L2_CID_BASE);  
          fprintf(stdout,"   id: %d\n", queryctrl.id);  
          switch (queryctrl.type)
          {  
              case V4L2_CTRL_TYPE_INTEGER:  
                  fprintf(stdout, "   type: INTEGER\n");  
                  break;  
              case V4L2_CTRL_TYPE_BOOLEAN:  
                  fprintf(stdout, "   type: BOOLEAN\n");  
                  break;  
              case V4L2_CTRL_TYPE_MENU:  
                  fprintf(stdout, "   type: MENU\n");  
                  /* Additional information is required for menu controls, the name of menu items.  
                   \* To query them applications set the id and index fields of struct v4l2_querymenu  
                   \*   and call the VIDIOC_QUERYMENU ioctl with a pointer to this structure. The driver 
                   \*   fills the rest of the structure or returns an EINVAL error code when the id or  
                   \*   index is invalid. Menu items are enumerated by calling VIDIOC_QUERYMENU with  
                   \*   successive index values from struct v4l2_queryctrl minimum (0) to maximum, inclusive. 
                   */  
                  querymenu.id = queryctrl.id;  
                  for (querymenu.index = queryctrl.minimum; querymenu.index < queryctrl.maximum; querymenu.index++){  
                      fprintf(stdout, "      menu id:%d\n", querymenu.index);  
                      fprintf(stdout, "      menu name:%s\n", querymenu.name);  
                  }  
                  break;  
              case V4L2_CTRL_TYPE_BUTTON:  
                  fprintf(stdout, "   type: BUTTON\n");  
                  break;  
          }  
          fprintf(stdout,"   name: %s\n", queryctrl.name);  
          fprintf(stdout,"   minimum: %d\n", queryctrl.minimum);  
          fprintf(stdout,"   maximum: %d\n", queryctrl.maximum);  
          fprintf(stdout,"   step: %d\n", queryctrl.step);  
          fprintf(stdout,"   default_value: %d\n", queryctrl.default_value);  
          fprintf(stdout,"   flags: %d\n", queryctrl.flags);  
      } 
      else 
      {  
  
             if (errno == EINVAL)  
                 continue;  
  
             perror("VIDIOC_QUERYCTRL");  
             break;  
      }  
  
   }        
  
} /* End of enum_video_ctrls_and_menus() */  
  
  
int capablityGet(char* devName)  
{  
    int fd;  
    struct v4l2_fmtdesc fmt;  
    struct v4l2_capability cap;  
    struct v4l2_format stream_fmt;  
    struct v4l2_input input;  
    struct v4l2_control ctrl;  
    struct v4l2_streamparm stream;  
    int err;  
    int ret;  
    int i;  
     
    fd = open_camer_device(devName);  
    for(i = 0; i < CHECKNUM; i++)  
    {  
        memset(&fmt,0,sizeof(fmt));  
        fmt.index = 0;  
        fmt.type = enum_fmt[i].type;  
        //printf("enum_fmt:%s\n", enum_fmt[i].name);  
        while((ret = ioctl(fd,VIDIOC_ENUM_FMT,&fmt)) == 0)  
        {  
            fmt.index ++ ;  
            printf("%s:{pixelformat = %c%c%c%c},description = '%s'\n",  
                    enum_fmt[i].name,  
                    fmt.pixelformat & 0xff,(fmt.pixelformat >> 8)&0xff,  
                    (fmt.pixelformat >> 16) & 0xff,(fmt.pixelformat >> 24)&0xff,  
                    fmt.description);  
        }  
    }  
    printf("\n\n");  
    enum_video_ctrls_and_menus(fd);  
    //查询视频设备支持的功能  
    ret = ioctl(fd,VIDIOC_QUERYCAP,&cap);  
    if(ret < 0){  
        perror("FAIL to ioctl VIDIOC_QUERYCAP");  
        //exit(EXIT_FAILURE);  
    }  
    //printf("capabilities:%08x\n", cap.capabilities);  
    printf("\ncheck the support capabilities\n");  
    for(i = 0; i < CHECKNUM; i++)  
    {  
        if(cap.capabilities & enum_fmt[i].type)  
        printf("%s\n",enum_fmt[i].name);  
    }  
    printf("\n");  
    if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))  
    {  
        printf("The Current device is not a video capture device\n");  
        //exit(EXIT_FAILURE);  
      
    }  
    else  
    printf("The Current device is a video capture device\n");  
    if(!(cap.capabilities & V4L2_CAP_STREAMING))  
    {  
        printf("The Current device does not support streaming i/o\n");  
        //exit(EXIT_FAILURE);  
    }  
    else  
    printf("The Current device support streaming i/o\n");  
  
    return 0;  
}  
