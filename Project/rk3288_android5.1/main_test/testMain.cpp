#include <stdlib.h>
#include <HAL/rk32885.1/AGpio.h>
#include <HAL/rk32885.1/ASysTimer.h>
#include <HAL/rk32885.1/ATimer.h>

#include <HAL/rk32885.1/ASPI.h>
#include <HAL/rk32885.1/AStorage.h>
#include <HAL/sensors/MPU6000.h>
#include <HAL/sensors/MS5611_SPI.h>
static const char *spidevice = "/dev/oledEuler_dev";

static const char *gpiodevice = "/dev/luobogpio";



int main(int argc,char** arvgv)
{
    //storage test
    int ret;
    printf("Tesas ssadt\n");
    //androidUAV::AStorage storage;
    //storage.erase(0);
    Android_TIME::ASysTimer timer;
    sensors::MPU6000 mpu6000;
    sensors::MS5611_SPI ms5611;
    //mpu6000 and gpio test

    HAL::IGPIO *gpio = new HAL::AGPIO(gpiodevice,228);//6000 cs
    HAL::IGPIO *gpio2 = new HAL::AGPIO(gpiodevice,230);//ms5611 cs

    androidUAV::ASPI spi(spidevice);

    devices::accelerometer_data acc_data;
    devices::gyro_data gyro_data;
    devices::baro_data ms5611_data;
    spi.setSpeed(500000);

    ret = mpu6000.init(&spi,gpio);
    ret = ms5611.init(&spi,gpio2);
    if(ret < 0)
        printf("Init ms561 error\n");
    timer.delayms(100.0);


    mpu6000.accelerometer_axis_config(0, 1, 2, +1, +1, +1);
    mpu6000.gyro_axis_config(0, 1, 2, +1, +1, +1);
    printf(" hello asasdret = %d\n",ret);
    timer.delayms(100.0);

    while(0);
    while(1)
    {
        /*mpu6000.read(&acc_data);
        mpu6000.read(&gyro_data);
        printf("gyro = %f %f %f\n\n",gyro_data.x,gyro_data.y,gyro_data.z);
        printf("acc = %f %f %f\n",acc_data.x,acc_data.y,acc_data.z);*/
        ms5611.read(&ms5611_data);
        printf("press = %f %f\n",ms5611_data.pressure,ms5611_data.temperature);
        timer.delayms(100.0);
    }
    return 0;
}
