#include "AGpio.h"
#include <HAL/rk32885.1/ALog.h>
namespace HAL
{
    AGPIO::AGPIO(const char*fdPath,int ioID)
    {
        openStatus = 0;
        fdGpio = 0;
        if(fdOpen(fdPath) < 0)
        {
            LOG2("androidUAV:Error occur\n");
        }
        setIoID(ioID);
    }

    AGPIO::~AGPIO()
    {
        fdClose();
    }


    int AGPIO::fdOpen(const char *fdPath)
    {
        if(!fdPath)
        {
            LOG2("androidUAV:Invalid fdGpio path\n");
            openStatus = 0;
            return -1;
        }
        fdGpio = open(fdPath,O_RDWR);
        if(fdGpio < 0)
        {
            LOG2("androidUAV:open %s error\n",fdPath);
            openStatus = 0;
            return -1;
        }
        openStatus = 1;
        return 0;
    }
    int AGPIO::fdClose()
    {
        if(openStatus)
        {
            close(fdGpio);
            LOG2("Close success\n");
        }
        return 0;
    }
    void AGPIO::setIoID(int ioID)
    {
        ioData.gpio = ioID;
    }
    /*IO_REQUEST = 0,//request an io device
    IO_SET_OUT_MODE = 1,//set to output mode and set it's status
    IO_SET_SET_VAL = 2,//set value ...(need fix up here 0926)
    IO_SET_IN_MODE = 3,//set as input mode
    IO_READ_STATUS = 4,//read io status
    IO_FREE = 5,// free io device*/
    int AGPIO::setmode(UAVIOCTL mode)
    {
        int ret;
        if(openStatus)//check gpioFd open status
        {
            switch(mode)
            {

                case IO_REQUEST:
                    //ret = ioctl(fdGpio,1,&ioData);
                    break;
                case IO_SET_OUT_MODE:
                    ioData.status = 1;
                    ioData.cmd = IO_SET_OUT_MODE;
                    ret = ioctl(fdGpio,1,&ioData);
                    break;
                case IO_SET_VAL:
                    //ret = ioctl(fdGpio,1,&ioData);
                    break;
                case IO_SET_IN_MODE:
                    ioData.status = 0;
                    ioData.cmd = IO_SET_IN_MODE;
                    ret = ioctl(fdGpio,1,&ioData);
                    break;
                case IO_FREE:
                    break;
                default:
                    LOG2("androidUAV:Unrecognized cmd\n");
            }
            if(ret < 0)
            {
                LOG2("androidUAV:Io control error\n");
                LOG2("androidUAV io number %d\n",ioData.gpio);
                return ret;
            }
        }
        return ret;
    }
    //set output status status = 0 or 1
    int AGPIO::set_status(int status)
    {
        int ret;
        ioData.status = status;
        ioData.cmd = IO_SET_VAL;
        ret = ioctl(fdGpio,1,&ioData);
        return ret;
    }

    int AGPIO::read_status()
    {
    	ioData.cmd = IO_READ_STATUS;
        return ioctl(fdGpio,1,&ioData);
    }
    //MODE_IN = 0,
    //MODE_OUT_PushPull = 1,
    //MODE_OUT_OpenDrain = 2,
    void AGPIO::set_mode(GPIO_MODE mode)
    {
        switch(mode)
        {
            case MODE_IN:
                    setmode(IO_SET_IN_MODE);
                break;
            case MODE_OUT_PushPull:
                    setmode(IO_SET_OUT_MODE);
                break;
            case MODE_OUT_OpenDrain:
                    ;
                break;
        }
    }
    bool AGPIO::read()
    {
        return read_status();
    }
    void AGPIO::write(bool newvalue)
    {
       set_status(newvalue);
    }
    void AGPIO::toggle()
    {

    }
}
