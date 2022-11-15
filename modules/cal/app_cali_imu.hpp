#ifndef  ZZ_APP_IMU_CALI_HPP__
#define  ZZ_APP_IMU_CALI_HPP__

#define MAX_ACC_COUNT 10


int imu_cal_init(int acc_count = 2);
int imu_cal_feed(float *acc, float *temperature, const char**acc_name = NULL);

#endif
