#include "ahrs2.h"

NonlinearSO3AHRS::NonlinearSO3AHRS()
{
	detect_acc.set_threshold(1);
	detect_gyro.set_threshold(5 * PI / 180);
	reset();
}

NonlinearSO3AHRS::~NonlinearSO3AHRS()
{

}

void NonlinearSO3AHRS::update(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy, float gz, float twoKp, float twoKi, float twoKpMag, float twoKiMag, float dt)
{

}
void NonlinearSO3AHRS::update_gps()
{

}
void NonlinearSO3AHRS::reset()
{
	initialized = false;
}
