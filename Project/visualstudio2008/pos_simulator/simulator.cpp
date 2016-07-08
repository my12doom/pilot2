#include <Windows.h>
#include <float.h>
#include <time.h>
#include <algorithm/pos_controll.h>
#include <win32/comm.h>
#include "rc.h"
#include <math.h>

static float g = 9.80;
float pos[2] = {0};
float velocity[2] = {0};
float euler[3] = {0};
float target_euler[3] = {0};
pos_controller controller;
static float PI = acos(-1.0);

CRITICAL_SECTION cs;
Comm comm;
bool use_pos_controller = true;

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

DWORD WINAPI update_state(LPVOID p)
{
	int time = GetTickCount();

	while (true)
	{
		float dt = (GetTickCount() - time)/1000.0f;
		if (dt == 0)
		{
			Sleep(1);
			continue;
		}
		else if (dt > 0.5f)
		{
			time = GetTickCount();
			continue;
		}

		EnterCriticalSection(&cs);

		float accel_forward = g * (-sin(euler[1])/cos(euler[1]));		// lean forward = negetive pitch angle
		float accel_right = g * (sin(euler[0])/cos(euler[0]));

		if (!_finite(accel_forward) || !_finite(accel_right))
			printf("\naccel:%f,%f\n", accel_forward, accel_right);

		// rotate accel from forward-right to north-east axis
		float accel_north = cos(euler[2]) * accel_forward - sin(euler[2]) * accel_right;
		float accel_east = sin(euler[2]) * accel_forward + cos(euler[2]) * accel_right;

		float accel_from_attitude[2];
		float wind[2] = {5.5f, 5.5f};
		for(int axis = 0; axis < 2; axis ++)
		{
			// accel noise
			float noise = randf()*2.0f;

			// air resistance
			float dv = velocity[axis] - wind[axis];
			float air_resistance = dv*dv*dv*0.005 + 0.12 * dv*dv + 0.005;		// TODO: factor
			if (air_resistance * dv > 0)
				air_resistance = - air_resistance;


			// update
			velocity[axis] += (noise + air_resistance + (axis == 0 ? accel_north : accel_east)) * dt;
			pos[axis] += velocity[axis] * dt;
		}

		for(int axis=0; axis<2; axis++)
		{
			// 0.5hz LPF to simulate airframe response time
			float alpha = dt / (dt + 1.0f/(2*PI * 5.3f)); 
			euler[axis] = euler[axis] * (1-alpha) + (target_euler[axis]) * alpha;

			// simulate controll noise, 2 degree P-P white noise
			float delta_angle = randf() * (PI/90);
 			euler[axis] += delta_angle;
		}

		float alpha = dt / (dt + 1.0f/(2*PI * 3.3f)); 
		euler[2] = euler[2] * (1-alpha) + (target_euler[2]) * alpha;

		LeaveCriticalSection(&cs);

		time = GetTickCount();
		Sleep(1);
	}

	// 
	
	return 0;
}

DWORD WINAPI update_controller(LPVOID p)
{
	int time = GetTickCount();
	int start_time = GetTickCount();

	EnterCriticalSection(&cs);
	controller.provide_attitue_position(euler, pos, velocity);
// 	controller.set_desired_velocity(rc);
	controller.reset();
	LeaveCriticalSection(&cs);

	while(true)
	{
		use_pos_controller = rc[4] > 0;

		float dt = (GetTickCount() - time)/1000.0f;
		if (dt < 0.01f)
		{
			Sleep(10);
			continue;
		}
		else if (dt > 0.5f)
		{
			time = GetTickCount();
			continue;
		}

		EnterCriticalSection(&cs);

		float desired_velocity[2] = {rc[1] * 5, rc[0] * 5};
		if (abs(desired_velocity[0]) < 0.2 || !use_pos_controller)
			desired_velocity[0] = 0;
		if (abs(desired_velocity[1]) < 0.2 || !use_pos_controller)
			desired_velocity[1] = 0;

		float euler2[3] = {euler[0], euler[1], euler[2]-0*PI/180};

		controller.provide_attitue_position(euler2, pos, velocity);
		controller.set_desired_stick(rc);
		controller.update_controller(dt);
		controller.get_target_angles(target_euler);

// 		if (GetTickCount() - start_time > 5000 && GetTickCount() - start_time < 6000)
// 		{
// 			float sp[2] = {-5, -5};
// 			controller.set_setpoint(sp);
// 		}
// 
// 		if (GetTickCount() - start_time > 15000 && GetTickCount() - start_time < 16000)
// 		{
// 			float sp[2] = {0, 0};
// 			controller.set_setpoint(sp);
// 		}
// // 		if (GetTickCount() - start_time > 18000 && GetTickCount() - start_time < 28000)
// // 		{
// // 			use_pos_controller = false;
// // 		}
// 		if (GetTickCount() - start_time > 28000 && GetTickCount() - start_time < 29000)
// 		{
// 			controller.reset();
// 		}

		if (!_finite(target_euler[0]))
			printf("...");
		if (!use_pos_controller)
		{
			target_euler[0] = rc[0] * PI/4;
			target_euler[1] = -rc[1] * PI/4;
		}
		if (fabs(rc[3]) > 0.05f)
			target_euler[2] += dt * rc[3] * PI;

		LeaveCriticalSection(&cs);

		time = GetTickCount();
		Sleep(1);
	}

	return 0;
}

int remote_OnEvent(int code, void *extra_data)
{
	if (code == WM_CONNECT)
		read_rc_settings();

	return 0;
}

DWORD WINAPI update_stick(LPVOID p)
{
	while(true)
	{
		update_ppm();
		Sleep(20);
	}

	return 0;
}

int init_simulator()
{
	srand(time(NULL));
	for(int i=0; i<rand()&0xff; i++)
		rand32();

	InitializeCriticalSection(&cs);

	comm.add_callback(remote_OnEvent);
	CreateThread(NULL, NULL, update_state, NULL, NULL, NULL);
	CreateThread(NULL, NULL, update_controller, NULL, NULL, NULL);
	CreateThread(NULL, NULL, update_stick, NULL, NULL, NULL);

	return 0;
}

int toggle_position_controller()
{
	use_pos_controller = !use_pos_controller;
	return 0;
}