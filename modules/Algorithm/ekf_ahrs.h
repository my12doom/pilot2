#include <math/matrix.h>
#include <Algorithm/motion_detector.h>

class ekf_ahrs
{
public:
	ekf_ahrs();
	~ekf_ahrs();

	int update(float a[3], float gyro[3], float mag[3], float dt, bool use_mag);
	int get_euler(float *euler);

	matrix h(matrix &x);
	matrix f(matrix &x, float gx, float gy, float gz, float dt);

	void remove_mag_ned_z(float *mag_body, float *q);
	int init(float a[3], float gyro[3], float mag[3]);

	matrix x;	// q[0-3], gyro bias[0-2]
	matrix P;

	matrix predicted;
	matrix zk;


	motion_detector motion_acc;
	motion_detector motion_gyro;

	bool still;
	bool inited;
};

int test_ekf_ahrs();
