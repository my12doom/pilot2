#include <stdlib.h>
#include <HAL/rk32885.1/AGpio.h>
#include <HAL/rk32885.1/ASysTimer.h>
#include <HAL/sensors/MPU6000.h>
#include <HAL/rk32885.1/ASPI.h>
static const char *spidevice = "/dev/oledEuler_dev";

static const char *gpiodevice = "/dev/luobogpio";


Android_TIME::ASysTimer ttt;
HAL::ISysTimer * timer2 = (HAL::ISysTimer *)&ttt;
sensors::MPU6000 mpu6000;

int main(int argc,char** arvgv)
{

    //HAL::IGPIO *gpp;
    //gp.toggle();
    int ret;
    HAL::IGPIO *gpio = new HAL::AGPIO(gpiodevice,228);
    devices::accelerometer_data acc_data;
    devices::gyro_data gyro_data;
    androidUAV::ASPI spi(spidevice);
    spi.setSpeed(500000);

    ret = mpu6000.init(&spi,gpio);
    timer2->delayms(100.0);


    mpu6000.accelerometer_axis_config(0, 1, 2, +1, +1, +1);
    mpu6000.gyro_axis_config(0, 1, 2, +1, +1, +1);
    printf(" hello ret = %d\n",ret);
    timer2->delayms(100.0);
    int64_t start,end;
    while(1)
    {
        mpu6000.read(&acc_data);
        mpu6000.read(&gyro_data);
        printf("gyro = %f %f %f\n\n",gyro_data.x,gyro_data.y,gyro_data.z);
        printf("acc = %f %f %f\n",acc_data.x,acc_data.y,acc_data.z);
        timer2->delayms(100.0);
    }
    return 0;
}
