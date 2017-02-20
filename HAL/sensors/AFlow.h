#pragma once
#include <stdint.h>
 #include <unistd.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/Interface/ICamera.h>
#include <HAL/Interface/IFlow.h>
#include <HAL/rk32885.1/ALog.h>
#include <HAL/rk32885.1/AIMUFIFO.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <vector>
#include <math.h>
#include <Protocol/RFData.h>
#include <utils/log.h>

#define FLOWPIPE "/data/flow.pip"

/*
* Attention: SQUARESIZE^2 = AREANUM
*/
#define AREANUM 64
#define SQUARESIZE 8

#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240

#define SUBWIN_HEIGHT IMAGE_HEIGHT/SQUARESIZE
#define SUBWIN_WIDTH IMAGE_WIDTH/SQUARESIZE

#define MAX_COUNT 20

#define PI 3.1415926f

//#define RECORDUAV

//#define RECTOFILE
//use rk3288 v4l2 camera dirver ,cpu consumer buffer will be used
//#define USERKDRV

#ifdef USERKDRV

#else
#define UAVVGA
#endif
namespace sensors
{
	enum pipops
    {
        PIP_OPEN = 0,
        PIP_CREATE = 1,
        PIP_CLOSE = 2,
    };
    typedef struct corner_Info
	{
		bool flag;
		int areaID;
		int quality;
		cv::Point2f corner;
	}cornerInfo;
	class AFlow : public IFlow,devices::IRangeFinder
	{
		private:
			devices::ICamera *ACamera;
			bool initFeaturesFlag;
			uint8_t *p_data;
			
			std::vector<cv::Point2f> points[2];
			cv::Mat gray;
			cv::Mat prevGray;
			cv::Point2f viewOffset;		//offset x-(clos offset),y-(rows offset)
			float quality;
			cv::Point2f viewOffsetPrev;
			cv::Point2f radians;
			devices::frame_format format;
			pthread_t tidp;
			int calcOpticalThreadId;
			bool calcProcessExit;
			int policy;
			pthread_mutex_t mutex;//Non-recursive lock
			pthread_attr_t attr;
			struct sched_param schparam;
			androidUAV::AFIFO flowPipe;
			imu_data_t flowdata;
			
			//image process
			cornerInfo cornerInfoArray[AREANUM];

			std::vector<cv::Point2f> cornerOld;
			std::vector<cv::Point2f> cornerNew;
			std::vector<uchar> status;
			cv::Ptr<cv::FastFeatureDetector> featuredetector;
		private:
			static void *caculateOpticalThread(void *p);
			static void *readOpticalThread(void *p);
			void caculateProcess();
			void readProcess();
			int getOffset(std::vector<cv::Point2f> &pointBefore,std::vector<cv::Point2f> &pointCurrent,std::vector<uchar> &status,cv::Point2f &offset,std::vector<float> &err,float *quality);
			int offset2radians();
			int coor2radians(cv::Point2f last,cv::Point2f current,cv::Point2f &radians);
			int points2radians(std::vector<cv::Point2f> &pointBefore,std::vector<cv::Point2f> &pointCurrent,std::vector<uchar> &status,cv::Point2f &offset,float *flow_quality);
			int set_priority(int32_t priority);
			
			
		public:
			AFlow();
			~AFlow();
			//image processing
			int updateCorners(cv::Mat &gray,std::vector<cv::Point2f> &cornerNew,std::vector<uchar> &status,cornerInfo *cornerinfos,int infosize);
			int fastCorner(cv::Mat &gray,int id,int grayThreshold,bool NonmaxSuppression,int type,cv::Point2f &outCorner);
			int addPointManual(cv::Mat &gray,int id,cv::Point2f *point);
			int sharpImage(cv::Mat &gray,cv::Mat &sharpImg);
			// flow data pipe
			int initPIPE(const char *path,int mode);
			int writePIPE(void *data,int size);
			int readPIPE(void *data,int size);
			//camera:NULL for read optical flow parameter
			virtual int init(devices::ICamera *camera);
			
			// return false if any error/waning
			virtual bool healthy();
			//IFlow
			// return 0 if new data available, 1 if old data, negative for error.
			// assume camera face down and top of camera image aligned to head of airframe.
			// x and y returns "visual angular rate caused by combination of linear(relative to the ground) and angular velocity of body, in radian
			// quality returns estimated image detail level, 0 ~ 1.
			// x+ points to right side of camera image, y+ points to top side of camera image.
			virtual int read(sensors::flow_data *out);
			
			virtual int read(float *out, int64_t *timestamp = NULL){return 0;}
			// IRangeFinder
			virtual int trigger(){return 0;}		// ignore trigger() since we don't have that I2C command
	};
}
static int get_thread_policy (pthread_attr_t *attr);
static int get_thread_priority (pthread_attr_t *attr,struct sched_param *param);
static void set_thread_policy (pthread_attr_t *attr,int policy);
static void set_thread_priority(pthread_attr_t *attr,struct sched_param *param,int32_t priority);
static uint16_t crc16(const uint8_t* data,uint32_t size);
static uint16_t UpdateCRC16(uint16_t crcIn,uint8_t byte);
static int64_t gettime();
