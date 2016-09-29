#include <stdlib.h>
#include "includes.h"
static const char *spidevice = "/dev/oledEuler_dev";

static const char *gpiodevice = "/dev/luobogpio";

int a;
sensors::MPU6000 mpu6000;
int main(int argc,char** arvgv)
{
    int ret;
    androidUAV::AGPIO gpio(gpiodevice,228);
    devices::accelerometer_data acc_data;
    devices::gyro_data gyro_data;
    androidUAV::ASPI spi(spidevice);
	
    mpu6000.init(&spi,&gpio);
    mpu6000.accelerometer_axis_config(0, 1, 2, +1, +1, +1);
    mpu6000.gyro_axis_config(0, 1, 2, +1, +1, +1);
    while(1)
    {
        //mpu6000.read(&acc_data);
        mpu6000.read(&gyro_data);
        printf("gyro = %f %f %f\n",gyro_data.x,gyro_data.y,gyro_data.z);
        //printf("acc = %f %f %f\n",acc_data.x,acc_data.y,acc_data.z);

        usleep(50000);
    }
    return 0;
}
