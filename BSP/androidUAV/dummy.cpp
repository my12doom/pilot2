#include <HAL/Resources.h>
#include <HAL/rk32885.1/ATimer.h>
#include <stdio.h>
#include <utils/param.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <unistd.h>
using namespace androidUAV;

void reset_system()
{
    printf("system reset requested!!!\n");
    exit(1);
}

const char bsp_name[] = "androidUAV";

int bsp_init_all()
{
    // Prio in the range -20  to  19
    setpriority(PRIO_PROCESS,getpid(),19);
    static androidUAV::ATimer timer[3];
    manager.register_Timer("mainloop", &timer[0]);
    manager.register_Timer("log", &timer[1]);
    manager.register_Timer("imu", &timer[2]);
    timer[0].set_period(0);
    timer[1].set_period(-15);
    timer[2].set_period(2);

    param("time", 3000)=1000;
    param("ekf", 0)=2;
    init_sensor();

    return 0;
}
