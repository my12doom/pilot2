#include <math/matrix.h>

#define MAX_STATIONS 20

typedef struct
{
	float x;
	float y;
} v2d;

class uwb_test
{
public:
	uwb_test();
	~uwb_test();
	int reset();

	int set_station_pos(v2d * stations, int count);
	int run(float *ranges, float dt);

	matrix x;	// [xyz, vxyz]
	matrix P;
	matrix Q;

	v2d stations[MAX_STATIONS];

	int station_count;
};


int uwb_selftest();
