#pragma once
#include <HAL/rk32885.1/ALog.h>
#include <HAL/Interface/ISPI.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define READ 1
#define WRITE 0
#define DEFAULTSPEED 500000
#define DEFAULTBITS 8
#define DEFFAULTDELAY 0

namespace androidUAV
{
    class ASPI : public HAL::ISPI
    {
        public:
            ASPI(const char*);
            ~ASPI();

            int setMode(uint8_t mode);
            int setSpeed(uint32_t speedHz);
            int setDelay(uint16_t delayus);
            int setBits(uint8_t bits);
            uint8_t spi_rxtx(uint8_t data);

            virtual int init();
            virtual int set_speed(int speed);// speed in hz
            virtual int set_mode(int CPOL, int CPHA);// CPOL: 0 = Idle Low, 1 = Idle High; CPHA: 0 = capture at first edge, 1 = capture at second edge
            virtual uint8_t txrx(uint8_t data);
        private:
            int fdOpen(const char*);
            int fdClose();
            int spiFd;
            uint32_t speed;
            uint16_t delay;
            uint8_t bits;
    };
}

