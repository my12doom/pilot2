#include "opticalFlow.h"
#include "OV7740Control.h"

using namespace HAL;
using namespace sensors;
using namespace androidUAV;
AFlow flow;
RK3288Camera51 ACamera;
RK3288Video camera0;
int camfd;
devices::frame_format cameraOpticalParams = {
	.width  = IMAGE_WIDTH,
	.height =IMAGE_HEIGHT,
};

int init_all()
{
	int ret;
#ifdef USERKDRV
	if(ACamera.init(0,&cameraOpticalParams) == 0)
#else
	if(camera0.init(VIDEODEV,NULL)==0)
#endif
	{
		
		if(flow.initPIPE(FLOWPIPE,PIP_CREATE) < 0)
		{
			LOG2("androidUAV:init optical-flow pipe error\n");
			return -1;
		}
#ifdef USERKDRV
		if(flow.init(&ACamera) == 0)
#else
		if(flow.init(&camera0) == 0)
#endif
		{
			LOG2("androidUAV:init optical flow base cuccess\n");
		}
		else
		{
			LOG2("androidUAV:init optical flow base error\n");
			return -1;
		}
	}
	else
	{
		LOG2("androidUAV:init optical-flow camera error\n");
		return -1;
	}
	
	return 0;
}
int initOV7740()
{
	int ret = 0;
	uint8_t dataT;
	camfd = open(CAMSYS_DEVNAME,O_RDWR);
	if(camfd < 0)
		return -1;
	//set OV7740 registers here
	ret = write_I2C_cam(camfd,OV7740_SLAVE_ADDR,AEC_CTL,0x86,1,1);//0x82 AE/AG close 0x86
	
	
	write_I2C_cam(camfd,OV7740_SLAVE_ADDR,AEC_TIME_H,0x00,1,1);
	write_I2C_cam(camfd,OV7740_SLAVE_ADDR,AEC_TIME_L,0xe0,1,1);//set 

	/*write_I2C_cam(camfd,OV7740_SLAVE_ADDR,AGC_CTL_H,0x00,1,1);
	write_I2C_cam(camfd,OV7740_SLAVE_ADDR,AGC_CTL_L,0x02,1,1);*/
	
	write_I2C_cam(camfd,OV7740_SLAVE_ADDR,WPT,0x00,1,1);
	write_I2C_cam(camfd,OV7740_SLAVE_ADDR,BPT,0x80,1,1);
	LOG2("init OV7740 successfully\n");
	
	return 0;
}
int main(int argc,char **argv)
{
	uint8_t dataH,dataL;
	uint8_t data;
	LOG2("androidUAV:init optical-flow camera process...\n");
	if(init_all() < 0)
	{
		return -1;
	}
	//wait camera I2C stable
	sleep(2);
	//if(initOV7740() < 0)
	{
		LOG2("androidUAV:waring init OV7740 error,use default parameters\n");
	}
	while(1)
	{
		
		//read_I2C_cam(camfd,OV7740_SLAVE_ADDR,0x31,&dataH,1,1);
		//read_I2C_cam(camfd,OV7740_SLAVE_ADDR,0x32,&dataL,1,1);
		//LOG2("read register %x %x\n",dataH,dataL);
		//usleep(300000);
		/*flow.writePIPE((void*)writedata,sizeof(writedata));
		sleep(1);
		LOG2("ret %d\n",flow.readPIPE(&flowdata,sizeof(flowdata)));
		LOG2("data %.2f,%.2f %lld\n",flowdata.array[0],flowdata.array[1],flowdata.time_stamp);*/
	}
	return 0;
}
