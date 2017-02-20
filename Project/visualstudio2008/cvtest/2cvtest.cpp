// cvtest.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include <Windows.h>
#include <atlbase.h>
#include <conio.h>
#include "feature.h"
#include "flow.h"
#include "avisynth.h"


#pragma comment(lib, "opencv_highgui243.lib")
#pragma comment(lib, "opencv_core243.lib")
#pragma comment(lib, "opencv_video243.lib")
#pragma comment(lib, "opencv_imgproc243.lib")
#pragma comment(lib, "winmm.lib")

static const double pi = 3.14159265358979323846;
static LONGLONG total_flow_time = 0;

inline static double square(int a)
{
	return a * a;
}

void DoEvents()
{
	MSG msg;
	BOOL result;

	while ( ::PeekMessage(&msg, NULL, 0, 0, PM_NOREMOVE ) )
	{
		result = ::GetMessage(&msg, NULL, 0, 0);
		if (result == 0) // WM_QUIT
		{                
			::PostQuitMessage(msg.wParam);
			break;
		}
		else if (result == -1)
		{
			// Handle errors/exit application, etc.
		}
		else 
		{
			::TranslateMessage(&msg);
			:: DispatchMessage(&msg);
		}
	}
}

/* This is just an inline that allocates images.  I did this to reduce clutter in the
 * actual computer vision algorithmic code.  Basically it allocates the requested image
 * unless that image is already non-NULL.  It always leaves a non-NULL image as-is even
 * if that image's size, depth, and/or channels are different than the request.
 */
inline static void allocateOnDemand( IplImage **img, CvSize size, int depth, int channels )
{
	if ( *img != NULL )	return;

	*img = cvCreateImage( size, depth, channels );
	if ( *img == NULL )
	{
		fprintf(stderr, "Error: Couldn't allocate image.  Out of memory?\n");
		exit(-1);
	}
}


bool is_avs(const wchar_t *file)
{
	int len = wcslen(file);
	wchar_t tmp[1024];
	wcscpy(tmp, file + len - 3);
	for(int i=0; i<3; i++)
		tmp[i] = towlower(tmp[i]);

	return wcscmp(tmp, L"avs") == 0;
}

void draw_arrow(IplImage *img, CvPoint from, CvPoint to, CvScalar line_color, int line_thickness)
{
	CvPoint p = from , q = to;

	double angle;		angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
	double hypotenuse;	hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );

	/* Here we lengthen the arrow by a factor of three. */
	q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
	q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

	/* Now we draw the main line of the arrow. */
	/* "frame1" is the frame to draw on.
	 * "p" is the point where the line begins.
	 * "q" is the point where the line stops.
	 * "CV_AA" means antialiased drawing.
	 * "0" means no fractional bits in the center cooridinate or radius.
	 */
	cvLine( img, p, q, line_color, line_thickness, CV_AA, 0 );
	/* Now draw the tips of the arrow.  I do some scaling so that the
	 * tips look proportional to the main line of the arrow.
	 */			
	p.x = (int) (q.x + 9 * cos(angle + pi / 4));
	p.y = (int) (q.y + 9 * sin(angle + pi / 4));
	cvLine( img, p, q, line_color, line_thickness, CV_AA, 0 );
	p.x = (int) (q.x + 9 * cos(angle - pi / 4));
	p.y = (int) (q.y + 9 * sin(angle - pi / 4));
	cvLine( img, p, q, line_color, line_thickness, CV_AA, 0 );
}

int32_t dxdy[1024][1024][3];
int eigen[1024][1024];

int sqrti(float x)
{
	union { float f; int x; } v; 

	// convert to float
	v.f = (float)x;

	// fast aprox sqrt
	//  assumes float is in IEEE 754 single precision format 
	//  assumes int is 32 bits
	//  b = exponent bias
	//  m = number of mantissa bits
	v.x  -= 1 << 23; // subtract 2^m 
	v.x >>= 1;       // divide by 2
	v.x  += 1 << 29; // add ((b + 1) / 2) * 2^m

	// convert to int
	return (int)v.f;
}

void find_feature_eigen(uint8_t *img, uint8_t *out, int width, int height, int stride)
{
	for(int y=1; y<height-1; y++)
	{
		for(int x=1; x<width-1; x++)
		{
			int a0 = img[(y-1)*stride+(x-1)];
			int a1 = img[(y-1)*stride+(x+0)];
			int a2 = img[(y-1)*stride+(x+1)];
			int a3 = img[(y+0)*stride+(x-1)];
			int a4 = img[(y+0)*stride+(x+0)];
			int a5 = img[(y+0)*stride+(x+1)];
			int a6 = img[(y+1)*stride+(x-1)];
			int a7 = img[(y+1)*stride+(x+0)];
			int a8 = img[(y+1)*stride+(x+1)];

			
			int dy			=	(a0*-1 + a1*-2 + a2 *-1 +
								a3* 0 + a4* 0 + a5 * 0 +
								a6*1  + a7* 2 + a8 * 1);

			int dx			=	(a0*-1 + a1*-0 + a2 * 1 +
								a3*-2 + a4* 0 + a5 * 2 +
								a6*-1 + a7* 0 + a8 * 1);

			dxdy[y][x][0] = dx * dx;
			dxdy[y][x][1] = dy * dy;
			dxdy[y][x][2] = dx * dy;
		}
	}

	int global_max = 1;

	for(int y=1; y<height-1; y++)
	{
		for(int x=1; x<width-1; x++)
		{
// 			int dx[9] = {dxdy[y-1][x-1][0], dxdy[y-1][x+0][0], dxdy[y-1][x+1][0], 
// 						 dxdy[y+0][x-1][0], dxdy[y+0][x+0][0], dxdy[y+0][x+1][0], 
// 						 dxdy[y+1][x-1][0], dxdy[y+1][x+0][0], dxdy[y+1][x+1][0], };
// 
// 			int dy[9] = {dxdy[y-1][x-1][1], dxdy[y-1][x+0][1], dxdy[y-1][x+1][1], 
// 						 dxdy[y+0][x-1][1], dxdy[y+0][x+0][1], dxdy[y+0][x+1][1], 
// 						 dxdy[y+1][x-1][1], dxdy[y+1][x+0][1], dxdy[y+1][x+1][1], };

			int dx2 = 0;
			int dy2 = 0;
			int dx_dy = 0;

			dx2 += dxdy[y-1][x-1][0];
			dx2 += dxdy[y-1][x+0][0];
			dx2 += dxdy[y-1][x+1][0];
			dx2 += dxdy[y+0][x-1][0];
			dx2 += dxdy[y+0][x+0][0];
			dx2 += dxdy[y+0][x+1][0];
			dx2 += dxdy[y+1][x-1][0];
			dx2 += dxdy[y+1][x+0][0];
			dx2 += dxdy[y+1][x+1][0];

			dy2 += dxdy[y-1][x-1][1];
			dy2 += dxdy[y-1][x+0][1];
			dy2 += dxdy[y-1][x+1][1];
			dy2 += dxdy[y+0][x-1][1];
			dy2 += dxdy[y+0][x+0][1];
			dy2 += dxdy[y+0][x+1][1];
			dy2 += dxdy[y+1][x-1][1];
			dy2 += dxdy[y+1][x+0][1];
			dy2 += dxdy[y+1][x+1][1];

			dx_dy += dxdy[y-1][x-1][2];
			dx_dy += dxdy[y-1][x+0][2];
			dx_dy += dxdy[y-1][x+1][2];
			dx_dy += dxdy[y+0][x-1][2];
			dx_dy += dxdy[y+0][x+0][2];
			dx_dy += dxdy[y+0][x+1][2];
			dx_dy += dxdy[y+1][x-1][2];
			dx_dy += dxdy[y+1][x+0][2];
			dx_dy += dxdy[y+1][x+1][2];

// 			int a11 = dx2;
// 			int a22 = dy2;
			int a12 = dx_dy;
			int a21 = dx_dy;
			float t2 = sqrt(float(int64_t(4*a12*a21)+int64_t(dx2-dy2)*int64_t(dx2-dy2)));
			eigen[y][x] = (int64_t(dx2+dy2)-t2)/2;

			global_max = max(global_max, eigen[y][x]);
		}
	}

	for(int y=1; y<height-1; y++)
	{
		for(int x=1; x<width-1; x++)
		{
			out[y*stride+x] = (eigen[y][x] << 8) / global_max;
		}
	}
}

int main2()
{
	timeBeginPeriod(1);

	FILE * f = fopen("out.csv", "wb");
	fprintf(f, "n,vx,vy,x,y\n");
	float ix = 0;
	float iy = 0;

	int re_w = 0;
	int re_h = 0;
	wchar_t msg[2048];
	wchar_t video[] = L"optical_flow_input.avi";

	IplImage *frame1 = NULL;
	IplImage *frame1Y = NULL;
	IplImage *frame2 = NULL;
	IplImage *frame2Y = NULL;
	IplImage *pyramid1 = NULL;
	IplImage *pyramid2 = NULL;
	IplImage *eig_image = NULL;
	IplImage *eig_imageY = NULL;
	IplImage *temp_image = NULL;

	try 
	{
		HMODULE avsdll = LoadLibrary(_T("avisynth.dll"));
		if(!avsdll)
		{
			wcscpy(msg, L"failed to load avisynth.dll\n");
			return -1;
		}
		IScriptEnvironment*  (__stdcall* CreateScriptEnvironment)(int version) = (IScriptEnvironment*(__stdcall*)(int)) GetProcAddress(avsdll, "CreateScriptEnvironment");
		if(!CreateScriptEnvironment)
		{
			wcscpy(msg, L"failed to load CreateScriptEnvironment()\n");
			return -2;
		}

		USES_CONVERSION;
		fprintf(stderr, "Loading file %s...", W2A(video));
		IScriptEnvironment* env = CreateScriptEnvironment(AVISYNTH_INTERFACE_VERSION);
		AVSValue arg[2] = {W2A(video), 24.0f};

		AVSValue res;
		if (is_avs(video))
			res = env->Invoke("Import", AVSValue(arg, 1));
		else
			res = env->Invoke("DirectShowSource", AVSValue(arg, 1));

		if(!res.IsClip())
		{swprintf(msg, L"Error: '%s' didn't return a video clip.\n", video); return E_FAIL;}

		PClip clip = res.AsClip();
		VideoInfo inf = clip->GetVideoInfo();
		if(!inf.HasVideo())
		{swprintf(msg, L"Error: '%s' didn't return a video clip.\n", video); return E_FAIL;}

		fprintf(stderr, "OK\n");

		if (inf.pixel_type != VideoInfo::CS_BGR24)
		{
			fprintf(stderr, "Converting to RGB32....");
			res = env->Invoke("ConvertToRGB24", res);
			clip = res.AsClip();
			inf = clip->GetVideoInfo();
		}

		if (inf.pixel_type != VideoInfo::CS_BGR24)
		{
			fprintf(stderr, "FAILED\n");
			{wcscpy(msg, L"FAILED Converting to RGB32"); return E_FAIL;}
		}
		else
		{
			fprintf(stderr, "OK\n");
		}

		// resize
		if (re_w !=0 && re_h != 0)
		{
			AVSValue args[3];
			args[0] = res;
			args[1] = AVSValue(re_w);
			args[2] = AVSValue(re_h);
			res = env->Invoke("LanczosResize", AVSValue(args, 3));
			clip = res.AsClip();
			inf = clip->GetVideoInfo();
		}

		CvSize frame_size;
		frame_size.height = inf.height;
		frame_size.width = inf.width;
		allocateOnDemand(&frame1, frame_size, IPL_DEPTH_8U, 3 );
		allocateOnDemand(&frame2, frame_size, IPL_DEPTH_8U, 3 );
		allocateOnDemand(&frame1Y, frame_size, IPL_DEPTH_8U, 1 );
		allocateOnDemand(&frame2Y, frame_size, IPL_DEPTH_8U, 1 );
		allocateOnDemand( &pyramid1, frame_size, IPL_DEPTH_8U, 1 );
		allocateOnDemand( &pyramid2, frame_size, IPL_DEPTH_8U, 1 );
		allocateOnDemand( &eig_image, frame_size, IPL_DEPTH_32F, 1 );
		allocateOnDemand( &eig_imageY, frame_size, IPL_DEPTH_8U, 1 );
		allocateOnDemand( &temp_image, frame_size, IPL_DEPTH_32F, 1 );


		int width = clip->GetVideoInfo().width;
		int height = clip->GetVideoInfo().height;
		RGBQUAD *data = (RGBQUAD*) malloc(width * (height+32) * sizeof(RGBQUAD));
		int t = GetTickCount();
		int lt = t;
		CvScalar red = CV_RGB(255,0,0);
		CvScalar blue = CV_RGB(0,0,255);
		CvScalar green = CV_RGB(0,255,0);
		CvScalar yellow = CV_RGB(255,255,0);
		for(int i=0; i< clip->GetVideoInfo().num_frames; i= i++/*getch() == 'p' ? i+1 : i-1*/)
		{
			PVideoFrame aframe1 = clip->GetFrame(i, env);
			PVideoFrame aframe2 = clip->GetFrame(i+1, env);
			RGBQUAD* src_data = (RGBQUAD*) aframe1->GetReadPtr();
			int pitch_avisynth = aframe1->GetPitch();
			int pitch_cv = frame1->widthStep;

			printf("\rdecoding %d/%d frames", i, inf.num_frames);

			RGBQUAD * tmp = data;
			for (int j=0; j< inf.height; j++)
			{
				//memcpy(tmp + j*height, src_data + j*inf.height, sizeof(RGBQUAD) * inf.width);

				int width_step = min(pitch_avisynth, pitch_cv);
				memcpy(frame1->imageData + pitch_cv * j, aframe1->GetReadPtr() + pitch_avisynth * j, width_step);
				memcpy(frame2->imageData + pitch_cv * j, aframe2->GetReadPtr() + pitch_avisynth * j, width_step);
			}

			cvConvertImage(frame1, frame1Y);
			cvConvertImage(frame2, frame2Y);

			int number_of_features = 400;
			CvPoint2D32f frame1_features[400];
			CvPoint2D32f frame2_features[400];


			cvGoodFeaturesToTrack(frame1Y, eig_image, temp_image, frame1_features, &number_of_features, .01, 25, NULL);

			CvSize optical_flow_window = cvSize(3,3);
			char optical_flow_found_feature[400];
			float optical_flow_feature_error[400];
			CvTermCriteria optical_flow_termination_criteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );



			float vx = 0;
			float vy = 0;

			for(int j=0; j<number_of_features; j++)
			{
				CvPoint p = {frame1_features[j].x, frame1_features[j].y};
// 				cvCircle(frame1, p, 10, red, 3);

				//frame1_1C->imageData[int(frame1_features[i].x + frame1_features->y * pitch_frame)] ^= 0xff;
				xy diff = fit((uint8_t*)frame1Y->imageData, (uint8_t*)frame2Y->imageData, width, height, frame1Y->widthStep, p.x, p.y);
				CvPoint p2 = {frame1_features[j].x+diff.x, frame1_features[j].y + diff.y};
				draw_arrow(frame1, p, p2, green, 5);

				xy diff2 = fit2((uint8_t*)frame1Y->imageData, (uint8_t*)frame2Y->imageData, width, height, frame1Y->widthStep, p.x, p.y);
				CvPoint p3 = {frame1_features[j].x+diff2.x, frame1_features[j].y + diff2.y};
				draw_arrow(frame1, p, p3, blue, 3);

				vx += diff2.x;
				vy += diff2.y;
			}

			vx /= number_of_features;
			vy /= number_of_features;

			ix += vx / 30;
			iy += vy / 30;

			fprintf(f, "%d,%f,%f,%f,%f\n", i, vx, vy, ix, iy);

			cvCalcOpticalFlowPyrLK(frame1Y, frame2Y, pyramid1, pyramid2, frame1_features, frame2_features, number_of_features, optical_flow_window, 5, optical_flow_found_feature, optical_flow_feature_error, optical_flow_termination_criteria, 0 );
 			for(int j=0; j<number_of_features; j++)
			{
				if ( optical_flow_found_feature[i] == 0 )	continue;

				CvPoint p = {frame1_features[j].x, frame1_features[j].y};
				CvPoint q = {frame2_features[j].x, frame2_features[j].y};

				draw_arrow(frame2, p, q, yellow, 1);
			}

			lt = timeGetTime();
			find_feature_eigen((uint8_t*)frame1Y->imageData, (uint8_t*)frame2Y->imageData, width, height, frame1Y->widthStep);
			printf("%d/%d ms\n", timeGetTime()-t, timeGetTime()-lt);

// 			cvCornerMinEigenVal(frame1Y, eig_image, 3);
// 
// 			float * p = (float*)eig_image->imageData;
// 			uint8_t * p2 = (uint8_t*)eig_imageY->imageData;
// 			int global_max = 0;
// 			for(int i=0; i<eig_image->widthStep/4*eig_image->height; i++)
// 				global_max = max(global_max, p[i]);
// 			for(int i=0; i<eig_image->widthStep/4*eig_image->height; i++)
// 				p2[i] = p[i]*255/global_max;

			cvShowImage("input", frame2Y);
			cvShowImage("input2", frame1Y);
			DoEvents();
		}

		free(data);

		return 0;
	}

	catch(AvisynthError err) 
	{
		fprintf(stderr, err.msg);

		return -3;
	}

	return -4;
}

int main(void)
{

	main2();

	/* Create an object that decodes the input video stream. */
	CvCapture *input_video = cvCaptureFromFile(
		"b.avi"
		);
	if (input_video == NULL)
	{
		/* Either the video didn't exist OR it uses a codec OpenCV
		 * doesn't support.
		 */
		fprintf(stderr, "Error: Can't open video.\n");
		return -1;
	}

	/* Read the video's frame size out of the AVI. */
	CvSize frame_size;
	frame_size.height =
		(int) cvGetCaptureProperty( input_video, CV_CAP_PROP_FRAME_HEIGHT );
	frame_size.width =
		(int) cvGetCaptureProperty( input_video, CV_CAP_PROP_FRAME_WIDTH );

	/* Determine the number of frames in the AVI. */
	long number_of_frames;
	/* Go to the end of the AVI (ie: the fraction is "1") */
	cvSetCaptureProperty( input_video, CV_CAP_PROP_POS_AVI_RATIO, 1. );
	/* Now that we're at the end, read the AVI position in frames */
	number_of_frames = (int) cvGetCaptureProperty( input_video, CV_CAP_PROP_POS_FRAMES );
	/* Return to the beginning */
	cvSetCaptureProperty( input_video, CV_CAP_PROP_POS_FRAMES, 0. );

	/* Create a windows called "Optical Flow" for visualizing the output.
	 * Have the window automatically change its size to match the output.
	 */
	cvNamedWindow("Optical Flow", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("input", CV_WINDOW_AUTOSIZE);

	long current_frame = 0;
	while(true)
	{
		static IplImage *frame = NULL, *frame1 = NULL, *frame1_1C = NULL, *frame2_1C = NULL, *eig_image = NULL, *temp_image = NULL, *pyramid1 = NULL, *pyramid2 = NULL;

		static IplImage *_frame_feature = NULL;

		/* Go to the frame we want.  Important if multiple frames are queried in
		 * the loop which they of course are for optical flow.  Note that the very
		 * first call to this is actually not needed. (Because the correct position
		 * is set outsite the for() loop.)
		 */
		cvSetCaptureProperty( input_video, CV_CAP_PROP_POS_FRAMES, current_frame );

		/* Get the next frame of the video.
		 * IMPORTANT!  cvQueryFrame() always returns a pointer to the _same_
		 * memory location.  So successive calls:
		 * frame1 = cvQueryFrame();
		 * frame2 = cvQueryFrame();
		 * frame3 = cvQueryFrame();
		 * will result in (frame1 == frame2 && frame2 == frame3) being true.
		 * The solution is to make a copy of the cvQueryFrame() output.
		 */
		frame = cvQueryFrame( input_video );
		if (frame == NULL)
		{
			/* Why did we get a NULL frame?  We shouldn't be at the end. */
			fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
			return -1;
		}
		/* Allocate another image if not already allocated.
		 * Image has ONE channel of color (ie: monochrome) with 8-bit "color" depth.
		 * This is the image format OpenCV algorithms actually operate on (mostly).
		 */
		allocateOnDemand( &frame1_1C, frame_size, IPL_DEPTH_8U, 1 );
		allocateOnDemand( &_frame_feature, frame_size, IPL_DEPTH_8U, 1 );
		/* Convert whatever the AVI image format is into OpenCV's preferred format.
		 * AND flip the image vertically.  Flip is a shameless hack.  OpenCV reads
		 * in AVIs upside-down by default.  (No comment :-))
		 */
		cvConvertImage(frame, frame1_1C, CV_CVTIMG_SWAP_RB);

		/* We'll make a full color backup of this frame so that we can draw on it.
		 * (It's not the best idea to draw on the static memory space of cvQueryFrame().)
		 */
		allocateOnDemand( &frame1, frame_size, IPL_DEPTH_8U, 3 );
		cvConvertImage(frame, frame1, CV_CVTIMG_SWAP_RB);

		/* Get the second frame of video.  Same principles as the first. */
		frame = cvQueryFrame( input_video );
		if (frame == NULL)
		{
			fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
			return -1;
		}
		allocateOnDemand( &frame2_1C, frame_size, IPL_DEPTH_8U, 1 );
		cvConvertImage(frame, frame2_1C, CV_CVTIMG_SWAP_RB);

		/* Shi and Tomasi Feature Tracking! */

		/* Preparation: Allocate the necessary storage. */
		allocateOnDemand( &eig_image, frame_size, IPL_DEPTH_32F, 1 );
		allocateOnDemand( &temp_image, frame_size, IPL_DEPTH_32F, 1 );

		/* Preparation: This array will contain the features found in frame 1. */
		CvPoint2D32f frame1_features[400];

		/* Preparation: BEFORE the function call this variable is the array size
		 * (or the maximum number of features to find).  AFTER the function call
		 * this variable is the number of features actually found.
		 */
		int number_of_features;
		
		/* I'm hardcoding this at 400.  But you should make this a #define so that you can
		 * change the number of features you use for an accuracy/speed tradeoff analysis.
		 */
		number_of_features = 400;

		/* Actually run the Shi and Tomasi algorithm!!
		 * "frame1_1C" is the input image.
		 * "eig_image" and "temp_image" are just workspace for the algorithm.
		 * The first ".01" specifies the minimum quality of the features (based on the eigenvalues).
		 * The second ".01" specifies the minimum Euclidean distance between features.
		 * "NULL" means use the entire input image.  You could point to a part of the image.
		 * WHEN THE ALGORITHM RETURNS:
		 * "frame1_features" will contain the feature points.
		 * "number_of_features" will be set to a value <= 400 indicating the number of feature points found.
		 */
		cvGoodFeaturesToTrack(frame1_1C, eig_image, temp_image, frame1_features, &number_of_features, .01, 25, NULL);
		feature_result result = find_feature((uint8_t*)frame1_1C->imageData, frame2_1C->width, frame2_1C->height, frame2_1C->widthStep, (uint8_t*)_frame_feature->imageData);


		/* Pyramidal Lucas Kanade Optical Flow! */

		/* This array will contain the locations of the points from frame 1 in frame 2. */
		CvPoint2D32f frame2_features[400];

		/* The i-th element of this array will be non-zero if and only if the i-th feature of
		 * frame 1 was found in frame 2.
		 */
		char optical_flow_found_feature[400];

		/* The i-th element of this array is the error in the optical flow for the i-th feature
		 * of frame1 as found in frame 2.  If the i-th feature was not found (see the array above)
		 * I think the i-th entry in this array is undefined.
		 */
		float optical_flow_feature_error[400];

		/* This is the window size to use to avoid the aperture problem (see slide "Optical Flow: Overview"). */
		CvSize optical_flow_window = cvSize(3,3);
		
		/* This termination criteria tells the algorithm to stop when it has either done 20 iterations or when
		 * epsilon is better than .3.  You can play with these parameters for speed vs. accuracy but these values
		 * work pretty well in many situations.
		 */
		CvTermCriteria optical_flow_termination_criteria
			= cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );

		/* This is some workspace for the algorithm.
		 * (The algorithm actually carves the image into pyramids of different resolutions.)
		 */
		allocateOnDemand( &pyramid1, frame_size, IPL_DEPTH_8U, 1 );
		allocateOnDemand( &pyramid2, frame_size, IPL_DEPTH_8U, 1 );

		/* Actually run Pyramidal Lucas Kanade Optical Flow!!
		 * "frame1_1C" is the first frame with the known features.
		 * "frame2_1C" is the second frame where we want to find the first frame's features.
		 * "pyramid1" and "pyramid2" are workspace for the algorithm.
		 * "frame1_features" are the features from the first frame.
		 * "frame2_features" is the (outputted) locations of those features in the second frame.
		 * "number_of_features" is the number of features in the frame1_features array.
		 * "optical_flow_window" is the size of the window to use to avoid the aperture problem.
		 * "5" is the maximum number of pyramids to use.  0 would be just one level.
		 * "optical_flow_found_feature" is as described above (non-zero iff feature found by the flow).
		 * "optical_flow_feature_error" is as described above (error in the flow for this feature).
		 * "optical_flow_termination_criteria" is as described above (how long the algorithm should look).
		 * "0" means disable enhancements.  (For example, the second array isn't pre-initialized with guesses.)
		 */
		cvCalcOpticalFlowPyrLK(frame1_1C, frame2_1C, pyramid1, pyramid2, frame1_features, frame2_features, number_of_features, optical_flow_window, 5, optical_flow_found_feature, optical_flow_feature_error, optical_flow_termination_criteria, 0 );
		float dx;
		float dy;
		int quality;

		LARGE_INTEGER tick_start, tick_end;
		QueryPerformanceCounter(&tick_start);
		for(int jjj=0; jjj<10; jjj++)
			quality = compute_flow((uint8_t*)frame1_1C->imageData, (uint8_t*)frame2_1C->imageData, 0, 0, 0, &dx, &dy);
		QueryPerformanceCounter(&tick_end);

		total_flow_time += tick_end.QuadPart - tick_start.QuadPart;


		printf("px4flow:%d, %.1f,%.1f, %dtick\n", quality, dx, dy, int(tick_end.QuadPart - tick_start.QuadPart));
		
		/* For fun (and debugging :)), let's draw the flow field. */
		for(int i = 0; i < number_of_features; i++)
		{
			/* If Pyramidal Lucas Kanade didn't really find the feature, skip it. */
			if ( optical_flow_found_feature[i] == 0 )	continue;

			int line_thickness;				line_thickness = 1;
			/* CV_RGB(red, green, blue) is the red, green, and blue components
			 * of the color you want, each out of 255.
			 */	
			CvScalar line_color;			line_color = CV_RGB(255,0,0);
	
			/* Let's make the flow field look nice with arrows. */

			/* The arrows will be a bit too short for a nice visualization because of the high framerate
			 * (ie: there's not much motion between the frames).  So let's lengthen them by a factor of 3.
			 */
			CvPoint p,q;
			p.x = (int) frame1_features[i].x;
			p.y = (int) frame1_features[i].y;
			q.x = (int) frame2_features[i].x;
			q.y = (int) frame2_features[i].y;

			double angle;		angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
			double hypotenuse;	hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );

			/* Here we lengthen the arrow by a factor of three. */
			q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
			q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

			/* Now we draw the main line of the arrow. */
			/* "frame1" is the frame to draw on.
			 * "p" is the point where the line begins.
			 * "q" is the point where the line stops.
			 * "CV_AA" means antialiased drawing.
			 * "0" means no fractional bits in the center cooridinate or radius.
			 */
			cvLine( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
			/* Now draw the tips of the arrow.  I do some scaling so that the
			 * tips look proportional to the main line of the arrow.
			 */			
			p.x = (int) (q.x + 9 * cos(angle + pi / 4));
			p.y = (int) (q.y + 9 * sin(angle + pi / 4));
			cvLine( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
			p.x = (int) (q.x + 9 * cos(angle - pi / 4));
			p.y = (int) (q.y + 9 * sin(angle - pi / 4));
			cvLine( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
		}

		for(int i=0; i<result.count+1; i++)
		{
			CvPoint center;
			CvPoint target;
			CvScalar line_color;			line_color = CV_RGB(0,0,255);
			int line_thickness;				line_thickness = 1;

			if (i!= result.count)
			{
			center.x = result.x[i];
			center.y = result.y[i];
// 			cvCircle(frame1, center, 3, CV_RGB(0,255,0));

 			xy f = fit((uint8_t*)frame1_1C->imageData, (uint8_t*)frame2_1C->imageData, frame2_1C->width, frame2_1C->height, frame2_1C->widthStep, center.x, center.y);
// 			cvCircle(frame1, center, 3, CV_RGB(0,0,255));
			target.x = center.x + f.x;
			target.y = center.y + f.y;
			}
			else
			{
				center.x = frame1_1C->width/2;
				center.y = frame1_1C->height/2;

				target.x = center.x + dx*10;
				target.y = center.y + dy*10;
				line_color = CV_RGB(0,255,0);

				line_thickness = 5;
			}

			CvPoint p,q;
			p=center;
			q=target;

			/* CV_RGB(red, green, blue) is the red, green, and blue components
			 * of the color you want, each out of 255.
			 */	
			double angle;		angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
			double hypotenuse;	hypotenuse = sqrt( square(p.y - q.y) + square(p.x - q.x) );

			/* Here we lengthen the arrow by a factor of three. */
			q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
			q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

			/* Now we draw the main line of the arrow. */
			/* "frame1" is the frame to draw on.
			 * "p" is the point where the line begins.
			 * "q" is the point where the line stops.
			 * "CV_AA" means antialiased drawing.
			 * "0" means no fractional bits in the center cooridinate or radius.
			 */
			cvLine( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
			/* Now draw the tips of the arrow.  I do some scaling so that the
			 * tips look proportional to the main line of the arrow.
			 */			
			p.x = (int) (q.x + 9 * cos(angle + pi / 4));
			p.y = (int) (q.y + 9 * sin(angle + pi / 4));
			cvLine( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
			p.x = (int) (q.x + 9 * cos(angle - pi / 4));
			p.y = (int) (q.y + 9 * sin(angle - pi / 4));
			cvLine( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
		}

		/* Now display the image we drew on.  Recall that "Optical Flow" is the name of
		 * the window we created above.
		 */


		cvShowImage("input", frame1);
		cvShowImage("Optical Flow", _frame_feature);
		/* And wait for the user to press a key (so the user has time to look at the image).
		 * If the argument is 0 then it waits forever otherwise it waits that number of milliseconds.
		 * The return value is the key the user pressed.
		 */
		int key_pressed = 0;
// 		key_pressed = cvWaitKey(0);
// // 		Sleep(10);
		DoEvents();

		/* If the users pushes "b" or "B" go back one frame.
		 * Otherwise go forward one frame.
		 */
		if (key_pressed == 'b' || key_pressed == 'B')
			current_frame--;
		else
			current_frame++;
		/* Don't run past the front/end of the AVI. */
		if (current_frame < 0)						current_frame = 0;
		if (current_frame >= number_of_frames - 1)	
			break;
	}

	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);

	printf("flow time:%fs", (double)total_flow_time / freq.QuadPart);
}
