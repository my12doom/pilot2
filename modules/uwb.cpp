#include "uwb.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

uwb_test::uwb_test()
{
	reset();
}
uwb_test::~uwb_test()
{

}

int uwb_test::reset()
{
	x = matrix(6,1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
	P = matrix::diag(6, 100.0f, 100.0f, 100.0f, 40.0f, 40.0f, 40.0f);
	Q = matrix::diag(6, 4e-6, 4e-6, 4e-6, 
						5e-4, 5e-4, 5e-4);
	return 0;
}

int uwb_test::set_station_pos(v2d * stations, int count)
{
	station_count = count;
	memcpy(this->stations, stations, sizeof(v2d)*count);

	return 0;
}

int uwb_test::run(float *distances, float dt)
{
	// predictions
	matrix F = matrix(6,6,
	1.0f, 0.0f, 0.0f, dt, 0.0f, 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f, dt, 0.0f,
	0.0f, 0.0f, 1.0f, 0.0f, 0.0f, dt,
	0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);

	matrix Bu = matrix(6,1,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);	// TODO: acc prediction here

	// observations
	float h[MAX_STATIONS] = {0};
	float x22 = x[2]*x[2];
	for(int i=0; i<station_count; i++)
	{
		h[i] += (x[0] - stations[i].x) * (x[0] - stations[i].x);
		h[i] += (x[1] - stations[i].y) * (x[1] - stations[i].y);
		h[i] += x[2] * x[2];
		h[i] = sqrt(h[i]);
	}

	matrix residual(station_count, 1);
	for(int i=0; i<station_count; i++)
		residual[i] = distances[i] - h[i];
	
	// observation matrix H
	matrix H;
	int used_station_count = station_count;
	H.m = used_station_count;
	H.n = 6;
	memset(H.data, 0, sizeof(float)*H.m*H.n);
	for(int i=0; i<used_station_count; i++)
	{
		H(i, 0) = -(stations[i].x - x[0]) / h[i];
		H(i, 1) = -(stations[i].y - x[1]) / h[i];
		H(i, 2) = -(0             - x[2]) / h[i];
	}

	// update
	matrix x1 = F * x + Bu;
	matrix P1 = F * P * F.transpos() + Q;

	float R_diag[MAX_DIMENSION];
	for(int i=0; i<MAX_DIMENSION; i++)
		R_diag[i] = 0.001;
	matrix R = matrix::diag(station_count, R_diag);

	matrix Sk = H * P1 * H.transpos() + R;
	matrix K = P1 * H.transpos() * Sk.inversef();

	x = x1 + K*residual;
	P = (matrix(P1.m) - K*H) * P1;

	return 0;
}

unsigned int rand32(void) 
{
//   random numbers from Mathematica 2.0.
//   SeedRandom = 1;
//   Table[Random[Integer, {0, 2^32 - 1}]
  static const unsigned long x[55] = {
    1410651636UL, 3012776752UL, 3497475623UL, 2892145026UL, 1571949714UL,
    3253082284UL, 3489895018UL, 387949491UL, 2597396737UL, 1981903553UL,
    3160251843UL, 129444464UL, 1851443344UL, 4156445905UL, 224604922UL,
    1455067070UL, 3953493484UL, 1460937157UL, 2528362617UL, 317430674UL, 
    3229354360UL, 117491133UL, 832845075UL, 1961600170UL, 1321557429UL,
    747750121UL, 545747446UL, 810476036UL, 503334515UL, 4088144633UL,
    2824216555UL, 3738252341UL, 3493754131UL, 3672533954UL, 29494241UL,
    1180928407UL, 4213624418UL, 33062851UL, 3221315737UL, 1145213552UL,
    2957984897UL, 4078668503UL, 2262661702UL, 65478801UL, 2527208841UL,
    1960622036UL, 315685891UL, 1196037864UL, 804614524UL, 1421733266UL,
    2017105031UL, 3882325900UL, 810735053UL, 384606609UL, 2393861397UL };
  static int init = 1;
  static unsigned long y[55];
  static int j, k;
  unsigned long ul;
  
  if (init)
  {
    int i;
    
    init = 0;
    for (i = 0; i < 55; i++) y[i] = x[i];
    j = 24 - 1;
    k = 55 - 1;
  }
  
  ul = (y[k] += y[j]);
  if (--j < 0) j = 55 - 1;
  if (--k < 0) k = 55 - 1;
  return((unsigned int)ul);
}

float randf()
{
	int n = (int)rand32();

	return n/2147483647.0f;
}

int uwb_selftest()
{
	FILE * f = fopen("D:\\my12doom\\uwb.csv", "wb");
	fprintf(f, "x,y,z,vx,vy,vz\n");

	// static test
	v2d stations[4] = {{-2,0}, {4,0}, {0,4}, {4,4}};
	float pos[3] = {2,2,1};
	uwb_test uwb;
	uwb.set_station_pos(stations, 4);
	uwb.reset();

	for(int i=0; i<1000; i++)
	{
		float ranges[4] = {0};
		for(int j=0; j<4; j++)
		{
			ranges[j] += (pos[0] - stations[j].x) * (pos[0] - stations[j].x);
			ranges[j] += (pos[1] - stations[j].y) * (pos[1] - stations[j].y);
			ranges[j] += (pos[2] - 0) * (pos[2] - 0);
			ranges[j] = sqrt(ranges[j]);

			ranges[j] += randf()*0.03;
		}

		if (i>500 && i<600)
		{
			pos[0] += 0.01;
		}

		uwb.run(ranges, 0.02f);
		fprintf(f, "%.3f,%.3f,%.3f,", uwb.x[0], uwb.x[1], uwb.x[2]);
		fprintf(f, "%.3f,%.3f,%.3f,", uwb.x[3], uwb.x[4], uwb.x[5]);
		fprintf(f, "%.3f,%.3f,%.3f\n", ranges[0], ranges[1], ranges[2]);
	}

	fclose(f);

	return 0;
}
