#include "AFlow.h"
#include <main_test/encoder.h>
#include <libyuv.h>
using namespace cv;
using namespace std;
using namespace android;
using namespace devices;
// test area
android_video_encoder enc;

vector<Point2f> pointsOriginal;

float camearMatrix[3][3] = {
		{502.54,0,161.265},
		{0,499.86,111.62},
		{0,0,1},
};

Point2f point_posi_ori;
namespace sensors
{
	AFlow::AFlow():cornerOld(AREANUM),cornerNew(AREANUM),status(AREANUM)
	{
		this->ACamera = NULL;
		initFeaturesFlag = 1;
		p_data = NULL;
		calcOpticalThreadId = 0;
		calcProcessExit = 1;
		quality = 0.0;
		//image process init
		memset(&cornerInfoArray,0,sizeof(cornerInfoArray));
		for(int i=0;i<AREANUM;i++)
		{
			cornerInfoArray[i].flag = 0;
			cornerInfoArray[i].areaID = i;
			status[i] = 0;
		}
		featuredetector = FastFeatureDetector::create();
	}
	AFlow::~AFlow()
	{
		
	}
	int AFlow::init(devices::ICamera *ICamera)
	{
		LOG2("\nin flow init function\n");
		if(ICamera)
		{
			this->ACamera = ICamera;
			ACamera->get_frame_format(&format);
			pthread_attr_init (&attr);

			policy = get_thread_policy (&attr);
			//Only before pthread_create excuted ,can we set thread parameters
			set_thread_policy(&attr,SCHED_FIFO);
			//update thread pilicy after set thread policy
			policy = get_thread_policy (&attr);
			set_priority(0);
			pthread_mutex_init(&mutex,NULL);
			calcOpticalThreadId = pthread_create(&tidp,&attr,caculateOpticalThread,(void*)this);
			if(calcOpticalThreadId < 0)
			{
				//perror
			}
			LOG2("androidUAV: frame info %d %d",format.width,format.height);
			return 0;
		}
		else //read flow data from another process
		{
			this->ACamera = NULL;
			pthread_attr_init (&attr);

			policy = get_thread_policy (&attr);
			//Only before pthread_create excuted ,can we set thread parameters
			set_thread_policy(&attr,SCHED_FIFO);
			//update thread pilicy after set thread policy
			policy = get_thread_policy (&attr);
			set_priority(0);
			pthread_mutex_init(&mutex,NULL);
			//calcOpticalThreadId = pthread_create(&tidp,&attr,readOpticalThread,(void*)this);
			if(calcOpticalThreadId < 0)
			{
				//perror
			}
		}
		return 0;
	}
	// return 0 if new data available, 1 if old data, negative for error.
	int AFlow::read(sensors::flow_data *out)
	{
#ifdef USERKDRV
		if(out && flowPipe.healthy() == 0)
		{
			imu_data_t flowdataTemp;
			int ret = readPIPE(&flowdataTemp,sizeof(imu_data_t));

			if(ret == 0)
			{
				out->x = flowdataTemp.array[0];
				out->y = flowdataTemp.array[1];
				out->quality = flowdataTemp.array[2];
				out->timestamp = flowdataTemp.time_stamp;
				return 0;
			}
			else if(ret == 0)
			{
				return 1;
			}
		}
#else
		if(out)
		{
			out->x = viewOffset.x;
			out->y = viewOffset.y;
			out->quality = quality;
			//LOG2("Flow %.2f %.2f %.2f\n",out->x,out->y,out->quality);
			return 0;
		}
#endif
		return -1;
	}
	
	bool AFlow::healthy()
	{
		return (this->ACamera)?0:-1;
	}
	void *AFlow::caculateOpticalThread(void *p)
	{
		AFlow *_this = (AFlow *)p;
		_this->caculateProcess();
		LOG2("caculateProcess exited,warning\n");
		pthread_exit(0);
		return NULL;
	}
	void *AFlow::readOpticalThread(void *p)
	{
		AFlow *_this = (AFlow *)p;
		_this->readProcess();
		pthread_exit(0);
		return NULL;
	}
	int AFlow::set_priority(int32_t priority)
	{
		//checkout policy ,only SCHED_FIFO SCHED_RR policies have a sched_priority value (1~99)
		if(policy == SCHED_OTHER)
			return -1;
		int32_t priority_min = 0;
		int32_t priority_max = 0;
		priority_min = sched_get_priority_min(policy);
		priority_max = sched_get_priority_max(policy);
		if(priority<=priority_min || priority>=priority_max)
			return -1;
		set_thread_priority(&attr,&schparam,priority);
		return 0;
	}
	//read flow data from another process by named pipe
	void AFlow::readProcess()
	{
		while(calcProcessExit)
		{
			if(readPIPE(&flowdata,sizeof(imu_data_t)) == sizeof(flowdata))
			{
				printf("read info %.2f %.2f \t%.2f\n",flowdata.array[0],flowdata.array[0],flowdata.array[1]);
				//log2((void*)&flowdata,TAG_FLOW_LOG,sizeof(flowdata));
			}
		}
	}
	void AFlow::caculateProcess()
	{
		int ret = 0;
		int cornerCurrent = 0;
		TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
		Mat gray80x60,prevGray80x60,uflow;
    	Size subPixWinSize(10,10), winSize(31,31);
    	FILE *file;
    	file = fopen("/data/video.dat","w+");
#ifdef RECORDUAV
		FILE *ff = fopen("/data/enc.h264", "wb");
		ret = enc.init(IMAGE_WIDTH, IMAGE_HEIGHT, 2500000);
#endif		
		while(calcProcessExit && ACamera)
		{
#ifdef RECORDUAV
			uint8_t *ooo = NULL;
			int encoded_size = enc.get_encoded_frame(&ooo);
			if (encoded_size > 0 && ooo)
			{
				int nal_type = ooo[4] & 0x1f;
				fwrite(ooo, 1, encoded_size, ff);
				fflush(ff);
			}
#endif
			int ret = ACamera->get_frame(&p_data);
			if(ret == 0)
			{
#ifdef UAVVGA
				Mat frame(480,640,CV_8UC1,p_data);
				Mat sharpTmp;
				/*cv::Rect rect(0,0,320,240);
				Mat gray_roi = frame(rect);
				gray_roi.copyTo(gray);*/
				//pyrDown(frame,gray,Size(320,240));
				resize(frame,gray,Size(320,240));
				//sharpImage(gray,sharpTmp);
				//sharpTmp.copyTo(gray);
#else
				Mat frame(240,320,CV_8UC1,p_data);
				frame.copyTo(gray);
#endif				
				ACamera->release_frame(p_data);

				if(initFeaturesFlag)
				{
				    int res = updateCorners(gray,cornerNew,status,cornerInfoArray,AREANUM);
				    cornerOld = cornerNew;
				    LOG2("features init num-->%d\n",res);
				    initFeaturesFlag = 0;
				}
				if(!cornerOld.empty())
				{
					vector<float> err;
					vector<uchar> statusT;
					int countt = 0;
					int ret = 0;
					if(prevGray.empty())
					{
						gray.copyTo(prevGray);
					}
					calcOpticalFlowPyrLK(prevGray, gray, cornerOld, cornerNew, statusT, err, winSize,3, termcrit,OPTFLOW_LK_GET_MIN_EIGENVALS, 0.001);
					
					ret = getOffset(cornerOld, cornerNew,statusT,viewOffset,err,&quality);
					
					if(ret > 0)
					{
						if(ret > 5)
						{
							viewOffset.x = viewOffset.x * 59.43 * PI / 1800.0;
							viewOffset.y = viewOffset.y * 59.43 * PI / 1800.0;
							viewOffsetPrev = viewOffset;
						}
						else
						{
							/*int64_t start,end;
							start = gettime();
							resize(frame,gray80x60,Size(40,30));
							resize(frame,prevGray80x60,Size(40,30));
							calcOpticalFlowFarneback(prevGray80x60,gray80x60,uflow,0.5,3,15,3,5, 1.2,0);
							end = gettime();
							printf("cost %lld\n",end-start);*/
							viewOffset.x = 0.0;
							viewOffset.y = 0.0;
							quality = 0.0;	
						}
					}
					else
					{
						viewOffset.x = 0.0;
						viewOffset.y = 0.0;
						quality = 0.0;
					}
					//write data to pilot via pipe if use rk camera driver
#ifdef USERKDRV
					float writedata[3] = {viewOffset.x,viewOffset.y,quality};
					writePIPE(writedata,sizeof(writedata));
#endif
					
					cornerCurrent = updateCorners(gray,cornerNew,statusT,cornerInfoArray,AREANUM);
					//printf("ret %d cornerCurrent %d\n",ret,cornerCurrent);
#ifdef RECORDUAV					
					for(int i=0;i<statusT.size();i++)
				    {
				        if(statusT[i] == 1)
				       	{
				            circle( gray, cornerNew[i], 3, Scalar(0,255,0), -1, 8);
				        }
				        
				    }
#endif

#ifdef RECTOFILE
					for(int i=0;i<statusT.size();i++)
				    {
				        if(statusT[i] == 1)
				       	{
				            circle(gray,cornerNew[i], 3, Scalar(0,255,0), -1, 8);
				        }
				        
				    }
				    char buf[15];
                	string label;
                	sprintf(buf,"corners %d",cornerCurrent);
                	label.append(buf);
                    putText(gray,label,Point2f(100,100),CV_FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0));
    				fwrite((uint8_t *)gray.ptr<uchar>(0),320*240,1,file);
#endif
				}
				
				//printf("%.2f %.2f\n",viewOffset.x,viewOffset.y);
				// feed live streaming encoder
#ifdef RECORDUAV
				void *live = enc.get_next_input_frame_pointer();
				if (live)
				{
					memcpy(live,(uint8_t *)gray.ptr<uchar>(0),320*240*3/2);
					enc.encode_next_frame();
				}
#endif
				std::swap(cornerNew, cornerOld);
				cv::swap(prevGray, gray);
				//usleep(10000);		
			}
			else
			{
				//usleep(5000);
			}
		}
	}
	int AFlow::updateCorners(Mat &gray,vector<Point2f> &cornerNew,vector<uchar> &status,cornerInfo *cornerinfos,int infosize)
	{
		if(!cornerinfos)
		{
			return -1;
		}
		int cornerFoundNum = 0;
		int currentCorner = 0;
		int width = gray.cols;
		int height = gray.rows;
		int areaRow = 0 / SQUARESIZE;
		int areaCol = 0 % SQUARESIZE;
		int startRow = 0;
		int startCol = 0;
	
		for(int i=0;i<status.size();i++)
		{
			if(status[i] == 1)
			{
				cornerFoundNum++;
			}
		}
		
		for(int i=0;i<infosize;i++)
		{
			if(status[i] == 0)
		    {
		        Point2f cornerT;
		        if(fastCorner(gray,i,40,true,FastFeatureDetector::TYPE_9_16,cornerT) == 0)
		        {
		            cornerNew[i] = cornerT;
		            cornerinfos[i].flag = 1;
		            cornerinfos[i].corner = cornerT;
		            currentCorner++;
		        }
		        else
		        {

		        }
		    }
		    else
		    {
		        //seperate corners
				Point2f cornerNow = cornerNew[i];
				areaRow = i / SQUARESIZE;
				areaCol = i % SQUARESIZE;
	
				startRow = areaRow*SUBWIN_HEIGHT;//
				startCol = areaCol*SUBWIN_WIDTH; //x
				if(cornerNow.x >= startCol && cornerNow.x <= startCol+SUBWIN_WIDTH && cornerNow.y >= startRow && cornerNow.y <= startRow+SUBWIN_HEIGHT)
				{
					currentCorner++;
				}
				else
				{
					Point2f cornerT;
				    if(fastCorner(gray,i,40,true,FastFeatureDetector::TYPE_9_16,cornerT) == 0)
				    {
				        cornerNew[i] = cornerT;
				        currentCorner++;
				        //printf("init success\n");
				    }
				}
				//currentCorner++;
		    }
		    
		}
		return currentCorner; 
	}
	int AFlow::fastCorner(cv::Mat &gray,int id,int grayThreshold,bool NonmaxSuppression,int type,cv::Point2f &outCorner)
	{
		int width = gray.cols;
		int height = gray.rows;
		int areaRow = id / SQUARESIZE;
		int areaCol = id % SQUARESIZE;
		int startRow = 0;
		int startCol = 0;
		std::vector<KeyPoint> keypoints;
		//
		startRow = areaRow*SUBWIN_HEIGHT;
		startCol = areaCol*SUBWIN_WIDTH;
		
		cv::Rect rect(startCol,startRow,SUBWIN_WIDTH,SUBWIN_HEIGHT);
		Mat gray_roi = gray(rect);
		//
		featuredetector->setThreshold(grayThreshold);
		featuredetector->setNonmaxSuppression(NonmaxSuppression);
		featuredetector->setType(type);
		featuredetector->detect(gray_roi,keypoints);

		if(keypoints.size() == 0)
		{
		    return -1;
		}
		else
		{
		    //pick out corners
		    outCorner.x = keypoints[keypoints.size()/2].pt.x+startCol;
		    outCorner.y = keypoints[keypoints.size()/2].pt.y+startRow;
		    return 0;
		}
	}
	int AFlow::sharpImage(cv::Mat &gray,cv::Mat &sharpImg)
	{
		Mat kernel(3,3,CV_32F,Scalar(0));
		kernel.at<float>(1,1) = 5.0;
		kernel.at<float>(0,1) = -1.0;
		kernel.at<float>(1,0) = -1.0;
		kernel.at<float>(1,2) = -1.0;
		kernel.at<float>(2,1) = -1.0;
	
		filter2D(gray,sharpImg,gray.depth(),kernel);
		return 0;
	}
	int AFlow::addPointManual(Mat &gray,int id,Point2f *point)
	{
		if(!point)
			return -1;
		int width = gray.cols;
		int height = gray.rows;
		int areaRow = id / SQUARESIZE;
		int areaCol = id % SQUARESIZE;
		int startRow = areaRow*SUBWIN_HEIGHT;
		int startCol = areaCol*SUBWIN_WIDTH;
	
		point->x = (startCol+(startCol+SUBWIN_WIDTH))/2;
		point->y = (startRow+(startRow+SUBWIN_HEIGHT))/2;
	
		return 0;
	}
	int AFlow::getOffset(vector<Point2f> &pointBefore,vector<Point2f> &pointCurrent,vector<uchar> &status,Point2f &offset,vector<float> &err,float *quality)
	{
		int meancount = 0;
		double pixel_flow_x_mean = 0.0;
		double pixel_flow_y_mean = 0.0;
		double pixel_flow_x_integral = 0.0;
		double pixel_flow_y_integral = 0.0;
		double pixel_flow_x_stddev = 0.0;
		double pixel_flow_y_stddev = 0.0;
		//should undistort features points ...
		float pixel_flow_x_wight = 0.0;
		float pixel_flow_y_wight = 0.0;
		vector<Point2f> cornerIndexErr;
		
		for(int i=0;i<status.size();i++)
		{
			if(status[i] == 1)
			{
				pixel_flow_x_mean += pointCurrent[i].x-pointBefore[i].x;
				pixel_flow_y_mean += pointCurrent[i].y-pointBefore[i].y;
				meancount++;
			}
		}
		if(meancount)
		{
			pixel_flow_x_mean /= meancount;
			pixel_flow_y_mean /= meancount;
			//compute the flow variance
			for(int i=0;i<status.size();i++)
			{
				if(status[i] == 1)
				{
					pixel_flow_x_stddev += (pointCurrent[i].x-pointBefore[i].x-pixel_flow_x_mean)*(pointCurrent[i].x-pointBefore[i].x-pixel_flow_x_mean);
					pixel_flow_y_stddev += (pointCurrent[i].y-pointBefore[i].y-pixel_flow_y_mean)*(pointCurrent[i].y-pointBefore[i].y-pixel_flow_y_mean);
				}
			}
			pixel_flow_x_stddev /= meancount;
			pixel_flow_y_stddev /= meancount;
			//convert to standard deviation
			pixel_flow_x_stddev = sqrt(pixel_flow_x_stddev);
			pixel_flow_y_stddev = sqrt(pixel_flow_y_stddev);
			//re-compute the mean flow with only the 95% consenting features
			meancount = 0;
			for(int i=0;i<status.size();i++)
			{
				if(status[i] == 1)
				{
					double this_flow_x = pointCurrent[i].x-pointBefore[i].x;
					double this_flow_y = pointCurrent[i].y-pointBefore[i].y;
					if(abs(this_flow_x-pixel_flow_x_mean) <= 2*pixel_flow_x_stddev && abs(this_flow_y-pixel_flow_y_mean) <= 2*pixel_flow_y_stddev)
					{
						pixel_flow_x_integral += this_flow_x;
						pixel_flow_y_integral += this_flow_y;
						meancount++;
						//store valid corner to corner vector 
						cornerIndexErr.push_back(Point2f(i,err[i]));
					}
					else
					{
						//delete corner now
						//status[i] = 0;
					}
				}
			}
			if(cornerIndexErr.size() > 0)
			{
				float sumT = 0.0;
				for(int i=0;i<cornerIndexErr.size();i++)
				{
					sumT+=cornerIndexErr[i].y;
				}
				for(int i=0;i<cornerIndexErr.size();i++)
				{
					cornerIndexErr[i].y = cornerIndexErr[i].y / sumT;
					pixel_flow_x_wight += (pointCurrent[(int)cornerIndexErr[i].x].x-pointBefore[(int)cornerIndexErr[i].x].x)*cornerIndexErr[i].y;
					pixel_flow_y_wight += (pointCurrent[(int)cornerIndexErr[i].x].y-pointBefore[(int)cornerIndexErr[i].x].y)*cornerIndexErr[i].y;
				}
			
			}
			else
			{
				*quality = 0.0;
				return -1;
			}
			if(meancount)
			{
				pixel_flow_x_integral /= meancount;
				pixel_flow_y_integral /= meancount;
				/*offset.x = pixel_flow_x_integral;
				offset.y = pixel_flow_y_integral;
				*quality = (float)meancount/(float)status.size();
				return meancount;*/
				
				offset.x = pixel_flow_x_wight;
				offset.y = pixel_flow_y_wight;
				*quality = (float)cornerIndexErr.size()/(float)status.size();
				return cornerIndexErr.size();
			}
			else
			{
				*quality = 0.0;
				return -1;
			}
		}
		else// no data avaliable
		{
			*quality = 0.0;
			return -1;
		}
	}
	int AFlow::offset2radians()
	{
		float offset_x = viewOffset.x;
		float offset_y = viewOffset.y;
		float x_radians;
		float y_radians;
		
		if(offset_x < 0)
			offset_x = -offset_x;
		if(offset_y < 0)
			offset_y = -offset_y;
		if(offset_x < 1.0)
		{
			x_radians = 0.0;
		}
		else
		{
			x_radians = atan(offset_x/camearMatrix[0][0]);
		}
		if(offset_y < 1.0)
		{
			y_radians = 0.0;
		}
		else
		{
			y_radians = atan(offset_y/camearMatrix[1][1]);
		}

		
		printf("%.4f %.4f\n",x_radians*180.0/PI,y_radians*180.0/PI);
		return 0;
	}
	/*
	* Description: caculate radians of optical flow
	* @param1: points before
	* @param2: pointe current
	* @param3: points flag
	* @param4: offset (rx,ry)-Radians
	*/
	int AFlow::points2radians(vector<Point2f> &pointBefore,vector<Point2f> &pointCurrent,vector<uchar> &status,Point2f &offset,float *flow_quality)
	{
		int meancount = 0;
		float pixel_flow_radians_x_mean = 0.0;
		float pixel_flow_radians_y_mean = 0.0;
		float pixel_flow_radians_x_integral = 0.0;
		float pixel_flow_radians_y_integral = 0.0;
		float pixel_flow_radians_x_stddev = 0.0;
		float pixel_flow_radians_y_stddev = 0.0;
	
		vector<Point2f> radians_x_y;
		for(int i=0;i<status.size();i++)
		{
			Point2f tempRadians;
			coor2radians(pointBefore[i],pointCurrent[i],tempRadians);
			radians_x_y.push_back(tempRadians);
		}
		for(int i=0;i<status.size();i++)
		{	
			if(status[i] == 1)
			{
				pixel_flow_radians_x_mean += radians_x_y[i].x;
				pixel_flow_radians_y_mean += radians_x_y[i].y;
				meancount++;
			}
		}
		if(meancount)
		{
			pixel_flow_radians_x_mean /= meancount;
			pixel_flow_radians_y_mean /= meancount;
			//compute the flow variance
			for(int i=0;i<status.size();i++)
			{
				if(status[i] == 1)
				{
					pixel_flow_radians_x_stddev += (radians_x_y[i].x-pixel_flow_radians_x_mean)*(radians_x_y[i].x-pixel_flow_radians_x_mean);
					pixel_flow_radians_y_stddev += (radians_x_y[i].y-pixel_flow_radians_y_mean)*(radians_x_y[i].y-pixel_flow_radians_y_mean);
				}
			}
			pixel_flow_radians_x_stddev /= meancount;
			pixel_flow_radians_y_stddev /= meancount;
			//convert to standard deviation
			pixel_flow_radians_x_stddev = sqrt(pixel_flow_radians_x_stddev);
			pixel_flow_radians_y_stddev = sqrt(pixel_flow_radians_y_stddev);
			//re-compute the mean flow with only the 95% consenting features
			meancount = 0;
			for(int i=0;i<status.size();i++)
			{
				if(status[i] == 1)
				{
					if(abs(radians_x_y[i].x-pixel_flow_radians_x_mean) <= 2*pixel_flow_radians_x_stddev && abs(radians_x_y[i].y-pixel_flow_radians_y_mean) <= 2*pixel_flow_radians_y_stddev)
					{
						pixel_flow_radians_x_integral += radians_x_y[i].x;
						pixel_flow_radians_y_integral += radians_x_y[i].y;
						meancount++;
					}
					else
					{
						status[i] = 0;
					}
				}
			}
			if(meancount)
			{
				pixel_flow_radians_x_integral /= meancount;
				pixel_flow_radians_y_integral /= meancount;
				offset.x = pixel_flow_radians_x_integral;
				offset.y = pixel_flow_radians_y_integral;
				*flow_quality = (float)meancount/(float)status.size();
				//printf("quality %.2f",flow_quality);
			}
		}
		return 0;
	}
	int AFlow::coor2radians(Point2f last,Point2f current,Point2f &radians)
	{
		float x_last = last.x;
		float y_last = last.y;
		float x_curr = current.x;
		float y_curr = current.y;
		float alpha_x = 0.0;
		float beta_x = 0.0;
		float alpha_y = 0.0;
		float beta_y = 0.0;
		float offset_x = x_curr-x_last;
		float offset_y = y_curr-y_last;
		float offsetR_x = 0.0;
		float offsetR_y = 0.0;
	
		alpha_x = atan((abs(x_last-camearMatrix[0][2]))/camearMatrix[0][0]);
		beta_x = atan((abs(x_curr-camearMatrix[0][2]))/camearMatrix[0][0]);
	
		alpha_y = atan(abs((y_last-camearMatrix[1][2]))/camearMatrix[1][1]);
		beta_y = atan(abs((y_curr-camearMatrix[1][2]))/camearMatrix[1][1]);
	
		if(x_last < camearMatrix[0][2] && x_curr > camearMatrix[0][2] || x_last > camearMatrix[0][2] && x_curr < camearMatrix[0][2])
		{
			offsetR_x = (alpha_x)+(beta_x);
		}
		else
		{
			if(beta_x < alpha_x)
			{
				offsetR_x = alpha_x - beta_x;
			}
			else
			{
				offsetR_x = beta_x - alpha_x;
			}
		}
	
		if(y_last < camearMatrix[1][2] && y_curr > camearMatrix[1][2] || y_last > camearMatrix[1][2] && y_curr < camearMatrix[1][2])
		{
			offsetR_y = (alpha_y)+(beta_y);
		}
		else
		{
			if(beta_y < alpha_y)
			{
				offsetR_y = alpha_y - beta_y;
			}
			else
			{
				offsetR_y = beta_y - alpha_y;
			}
		}
		if(offset_x < 0)
		{
			offsetR_x = -offsetR_x;
		}
		if(offset_y < 0)
		{
			offsetR_y = -offsetR_y;
		}
		radians.x  = offsetR_x*2;
		radians.y = offsetR_y*2;
		//LOG2("radians(%.3f,%.3f)\n",offsetR_x*180.0/PI*2.0,offsetR_y*180.0/PI*2.0);
		return 0;
	}
	int AFlow::initPIPE(const char *path,int mode)
	{
		if(!path)
			return -1;
		switch(mode)
		{
			case PIP_OPEN:
				return flowPipe.open(path,O_RDWR|O_NONBLOCK);
				break;
			case PIP_CREATE:
				return flowPipe.create(path,0777);
				break;
        	case PIP_CLOSE :
        		
        		break;
		}
		return 0;
	}
	int AFlow::writePIPE(void *data,int size)
	{
		return flowPipe.write(data,size);
	}
	int AFlow::readPIPE(void *data,int size)
	{
		return flowPipe.read(data,size);
	}
}
static int get_thread_policy(pthread_attr_t *attr)
{
	int policy;
	int rs = pthread_attr_getschedpolicy(attr, &policy);
	return policy;
}
static int get_thread_priority(pthread_attr_t *attr,struct sched_param *param)
{
	int rs = pthread_attr_getschedparam (attr, param);
	return param->__sched_priority;
}
static void set_thread_policy(pthread_attr_t *attr,int policy)
{
	int rs = pthread_attr_setschedpolicy (attr, policy);
	get_thread_policy (attr);
}
static void set_thread_priority(pthread_attr_t *attr,struct sched_param *param,int32_t priority)
{
	param->sched_priority = priority;
	pthread_attr_setschedparam(attr,param);
}
static uint16_t UpdateCRC16(uint16_t crcIn,uint8_t byte)
{
	uint32_t crc = crcIn;
	uint32_t in = byte|0x100;
	do
	{
		crc <<= 1;
		in <<= 1;
		if(in&0x100)
			++crc;
		if(crc&0x10000)
			crc ^= 0x1021;
	}while(!(in&0x10000));
	return crc&0xffffu;
}
static uint16_t crc16(const uint8_t* data,uint32_t size)
{
	uint32_t crc = 0;
	const uint8_t* dataEnd = data+size;
	while(data<dataEnd)
		crc = UpdateCRC16(crc,*data++);
	crc = UpdateCRC16(crc,0);
	crc = UpdateCRC16(crc,0);
	return crc&0xffffu;
}
static int64_t gettime()
{
	struct timespec tv;
	clock_gettime(CLOCK_MONOTONIC, &tv);
	return (int64_t)((tv.tv_sec) * 1000000 + (tv.tv_nsec)/1000);
}

