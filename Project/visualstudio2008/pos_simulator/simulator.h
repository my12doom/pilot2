#include <Windows.h>
#include <algorithm/pos_controll.h>


extern float pos[2];
extern float velocity[2];
extern float euler[3];
extern float target_euler[3];
extern CRITICAL_SECTION cs;
extern pos_controller controller;

int init_simulator();
int toggle_position_controller();