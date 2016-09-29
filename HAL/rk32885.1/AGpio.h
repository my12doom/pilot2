#pragma once
#include <HAL/Interface/IGPIO.h>
#include <HAL/rk32885.1/ALog.h>

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>

namespace HAL
{
    typedef struct aGpioData
    {
        int gpio;
        int status;
    }AGpioData;
    enum UAVIOCTL
    {
        IO_REQUEST = 0,//request an io device
        IO_SET_STATUS = 1,//set to output mode and set it's status
        IO_SET_VAL = 2,//set value ...(need fix up here 0926)
        IO_SET_IN_MODE = 3,//set as input mode
        IO_READ_STATUS = 4,//read io status
        IO_FREE = 5,// free io device
    };
    class AGPIO : public HAL::IGPIO
    {
        public:
        AGPIO(const char*,int);
        ~AGPIO();
        int setmode(UAVIOCTL mode);
        int set_status(int status);
        int read_status();
        //
        virtual void set_mode(GPIO_MODE mode);
        virtual bool read();
        virtual void write(bool newvalue);
        virtual void toggle();

    private:
        int fdOpen(const char*);
        int fdClose(void);
        void setIoID(int ioID);
    protected:
        char *fdPath;
        int fdGpio;
        int openStatus;
        AGpioData ioData;
    };

}

