#include "feature.h"
#include <stdlib.h>
#include <Windows.h>
#include "SAD.h"

int p[1024*500*4];
int p2[1024*500*4];
int step = 3;
feature_result find_feature(uint8_t *image, int width, int height, int stride, uint8_t *image_out)
{
	feature_result o = {0};


	int global_max = -65535;
	int global_min = 65535;

	for(int y=1; y<height-1; y+=step)
	{
		for(int x=1; x<width-1; x+=step)
		{
// 			int dx1 = abs((int)image[y*stride+x] - image[y*stride+x-1]);
// 			int dx2 = abs((int)image[y*stride+x] - image[y*stride+x+1]);
// 			int dy1 = abs(image[y*stride+x] - image[(y-1)*stride+x]);
// 			int dy2 = abs(image[y*stride+x] - image[(y+1)*stride+x]);
// 			int A11 = dx1*dx1+dx2*dx2;
// 			int A22 = dy1*dy1+dy2*dy2;
// 			int A12 = dx1*dy1 + 
// 			int det =  *dx*dy*dy;
// 			int trace = dx*dx+dy*dy;
// 
// 			p[y*width+x] = det-0.06*trace*trace;
			int a0 = image[(y-1)*stride+(x-1)];
			int a1 = image[(y-1)*stride+(x+0)];
			int a2 = image[(y-1)*stride+(x+1)];
			int a3 = image[(y+0)*stride+(x-1)];
			int a4 = image[(y+0)*stride+(x+0)];
			int a5 = image[(y+0)*stride+(x+1)];
			int a6 = image[(y+1)*stride+(x-1)];
			int a7 = image[(y+1)*stride+(x+0)];
			int a8 = image[(y+1)*stride+(x+1)];

			int dy =		a0*-1 + a1*-2 + a2 *-1 +
							a3* 0 + a4* 0 + a5 * 0 +
							a6*1  + a7* 2 + a8 * 1;

			int dx		 =  a0*-1 + a1*-0 + a2 * 1 +
							a3*-2 + a4* 0 + a5 * 2 +
							a6*-1 + a7* 0 + a8 * 1;

// 			p[y*width+x] =  a0*-3 + a1*-0 + a2 * 3 +
// 							a3*-10+ a4* 0 + a5 *10 +
// 							a6*-3 + a7* 0 + a8 * 3;

			p[y*width+x] = dx*dy - (dx+dy)*(dx*dy)*6/100;
			if (p[y*width+x] > global_max)
				global_max = p[y*width+x];

			if (p[y*width+x] < global_min)
				global_min = p[y*width+x];
		}
	}
// 
// 	int local_window = 1;
// 
// 	for(int y=local_window; y<height-local_window; y++)
// 	{
// 		for(int x=local_window; x<width-local_window; x++)
// 		{
// 			int local_max = -65535;
// 			int c = 0;
// 
// // 			for(int yy = -local_window; yy<= local_window; yy++)
// // 			{
// // 				for(int xx = -local_window; xx<= local_window; xx++)
// // 				{
// // 					c++;
// // 
// // 					if (p[(y+yy)*width+(x+xx)] > local_max )
// // 					{
// // 						local_max = p[(y+yy)*width+(x+xx)];
// // 					}
// // 				}
// // 			}
// 
// // 			if (local_max > 100)
// // 			p2[y*width+x] = local_max;
// 
//  			p2[y*width+x] = p[y*width+x];
// 		}
// 	}

	int threshold = (global_max-global_min) /10;

	for(int y=1; y<height-1; y+=step)
	{
		for(int x=1; x<width-1; x+=step)
		{
			if (p[y*width+x] > threshold && o.count < sizeof(o.v)/sizeof(o.v[0]))
			{
				bool dis = false;

				for(int i=0; i<o.count; i++)
				{
					int dt = abs(x-o.x[i]) + abs(y-o.y[i]);
					if (dt < 5)
					{					
						dis = true;
						break;
					}
				}

				if (!dis)
				{
					o.x[o.count] = x;
					o.y[o.count] = y;
					o.v[o.count] = p[y*width+x];
					o.count++;
				}
			}

// 			image_out[y*width+x] = (p2[y*width+x]-global_min) * 255 / (global_max-global_min);
		}
	}

// 	delete [] p;

	return o;
}

xy fit(uint8_t*image1, uint8_t*image2, int width, int height, int stride, int x, int y)
{
	xy o = {0};

	int window = 25;

	int cx1 = x-8;
	int cy1 = y-8;

	if (cx1<8 || cx1>=width-8 || cy1<8 || cy1>=height-8)
		return o;

	int sad_min = 0x7fffffff;

	for(int dy = -window; dy<=window; dy++)
	{
		for(int dx = -window; dx<=window; dx++)
		{
			int cx2 = cx1+dx;
			int cy2 = cy1+dy;

			if (cx2<8 || cx2>=width-8 || cy2<8 || cy2>=height-8)
				continue;

			int sad = SAD16x16(image1+stride*cy1+cx1, image2+stride*cy2+cx2, stride);

			if (sad<sad_min)
			{
				sad_min = sad;
				o.x = dx;
				o.y = dy;
			}
		}
	}

	return o;
}

xy fit2(uint8_t*image1, uint8_t*image2, int width, int height, int stride, int x, int y)
{
	xy o = {0};
	const int n = 3;
	int range_x = 25;
	int range_y = 25;

	static bool init = false;
	POINT search_table[(n*2+1)*(n*2+1)-1];
	if (!init)
	for(int i=0,y=-n; y<=n; y++)
	{
		for(int x=-n; x<=n; x++)
		{
			if ( x != 0 || y != 0)
			{
				search_table[i].x = x;
				search_table[i++].y = y;
			}
		}
	}

	int cx1 = x-8;
	int cy1 = y-8;

	if (cx1<8 || cx1>=width-8 || cy1<8 || cy1>=height-8)
		return o;


	int last_sad = SAD16x16(image1+stride*(cy1)+cx1, image2+stride*cy1+cx1, stride);
	int dx = 0;
	int dy = 0;
	while(true)
	{
		int min_sad = last_sad;
		int delta[2] = {0};
		for(int i=0; i<sizeof(search_table)/sizeof(search_table[0]); i++)
		{
			int sad = min_sad;
			if (abs(dx+search_table[i].y) < range_x && abs(dy+search_table[i].y) < range_y)
			{
				int nx = dx+search_table[i].x + cx1;
				int ny = dy+search_table[i].y + cy1;

				if (nx>=8 && nx < width-8 && ny>=8 && ny<height-8)
					sad = SAD16x16(image1+stride*(cy1)+cx1, image2+stride*ny+nx, stride);
			}

			if (sad < min_sad)
			{
				min_sad = sad;
				delta[0] = search_table[i].x;
				delta[1] = search_table[i].y;
			}
		}

		if (min_sad >= last_sad)
			break;

		dx += delta[0];
		dy += delta[1];
		last_sad = min_sad;
	}

	o.x = dx;
	o.y = dy;

	return o;
}