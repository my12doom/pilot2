// cvtest.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <stdio.h>
#include <math.h>
#include <Windows.h>

#ifdef DEBUG
#pragma comment(lib, "opencv_highgui243d.lib")
#pragma comment(lib, "opencv_core243d.lib")
#pragma comment(lib, "opencv_video243d.lib")
#pragma comment(lib, "opencv_imgproc243d.lib")
#pragma comment(lib, "opencv_stitching243d.lib")
#else
#pragma comment(lib, "opencv_highgui243.lib")
#pragma comment(lib, "opencv_core243.lib")
#pragma comment(lib, "opencv_video243.lib")
#pragma comment(lib, "opencv_imgproc243.lib")
#pragma comment(lib, "opencv_stitching243.lib")
#endif

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stitching/stitcher.hpp>
using namespace std;
using namespace cv;
bool try_use_gpu = false;
vector<Mat> imgs;
string result_name = "Z:\\test\\result.jpg";
int _tmain(int argc, char * argv[])
{
	int t = GetTickCount();
	string a1 = "Z:\\test\\DJI_0070.jpg";

	Mat img1 = imread(a1);
	Mat img2 = imread("Z:\\test\\DJI_0071.jpg");
	Mat img3 = imread("Z:\\test\\DJI_0072.jpg");    
	Mat img4 = imread("Z:\\test\\DJI_0073.jpg");    
	if (img1.empty() || img2.empty())
	{
		cout << "Can't read image"<< endl;
		return -1;
	}
	imgs.push_back(img1);
	imgs.push_back(img2);
	imgs.push_back(img3);
	imgs.push_back(img4);
	Stitcher stitcher = Stitcher::createDefault(try_use_gpu);
	// 使用stitch函数进行拼接
	Mat pano;
	Stitcher::Status status = stitcher.stitch(imgs, pano);
	imwrite(result_name, pano);
	Mat pano2=pano.clone();

	t = GetTickCount() - t;
	printf("%dms\n", t);
	
	// 显示源图像，和结果图像
	imshow("全景图像", pano);
	if(waitKey()==27)
		return 0;
}