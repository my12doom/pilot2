#include <math/matrix.h>

class ekf_ahrs
{
public:
	ekf_ahrs();
	~ekf_ahrs();

	int update(float a[3], float gyro[3], float dt);
	matrix h(matrix &x);

	matrix x;	// q[0-3], gyro bias[0-2]
	matrix P;
};

int test_ekf_ahrs();
