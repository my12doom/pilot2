#include "ASPI.h"
#include <inttypes.h>
namespace androidUAV
{
    ASPI::ASPI(const char* spiPath)
    {
        spiFd = 0;
        speed = DEFAULTSPEED;
        bits = DEFAULTBITS;
        delay = DEFFAULTDELAY;
        uint32_t max_speed;
        max_speed = SPIMAXSPEED;

        if(!spiPath)
        {
            LOG2("androidUAV:input valid\n");
        }
        else
        {
            if(fdOpen(spiPath) < 0)
            {
                LOG2("androidUAV:Create spi dev error\n");
            }
        }
        //set spi max_speed
        ioctl(spiFd, SPI_IOC_WR_MAX_SPEED_HZ,&max_speed);

        setBits(bits);
        setMode(3);
    }
    ASPI::~ASPI()
    {
        fdClose();
    }
    int ASPI::fdOpen(const char*fdPath)
    {
        if(!fdPath)
        {
            LOG2("invalid path\n");
            return -1;
        }
        spiFd = open(fdPath,O_RDWR);
        if(spiFd < 0)
            return -1;
        return 0;
    }
    int ASPI::fdClose()
    {
        close(spiFd);
        LOG2("androidUAV:Close success\n");
        return 0;
    }
    int ASPI::setMode(uint8_t mode)
    {
        int ret = 0;
        ret = ioctl(spiFd, SPI_IOC_WR_MODE, &mode);    //写模式
        if (ret < 0)
        {
            LOG2("androidUAV:set write as mode %d error\n",mode);
            return ret;
        }
        ret = ioctl(spiFd, SPI_IOC_RD_MODE, &mode);    //写模式
        if (ret < 0)
        {
            LOG2("androidUAV:set read as mode %d error\n",mode);
            return ret;
        }
        return ret;
    }
    int ASPI::setSpeed(int speedHz)
    {
        int ret = 0;
        uint32_t read;
        /*ret = ioctl(spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &speedHz); //写模式
        if (ret < 0)
        {
            LOG2("androidUAV:set write speed %d error\n",speedHz);
            return ret;
        }
        ret = ioctl(spiFd, SPI_IOC_RD_MAX_SPEED_HZ, &read);    //写模式
        if (ret < 0)
        {
            LOG2("androidUAV:set read speed %d error\n",speedHz);
            return ret;
        }
        printf("speed = %"PRIu32"\n",read);*/
        this->speed = speedHz;
        return ret;
    }
    int ASPI::setDelay(uint16_t delayus)
    {
        int ret = 0;
        this->delay = delayus;
        return ret;
    }
    int ASPI::setBits(uint8_t bits)
    {
        int ret = 0;
        ret = ioctl(spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits); //写bits
        if (ret < 0)
        {
            LOG2("androidUAV:set write bits %d error\n",bits);
            return ret;
        }
        ret = ioctl(spiFd, SPI_IOC_RD_BITS_PER_WORD, &bits);    //read bits
        if (ret < 0)
        {
            LOG2("androidUAV:set read bots %d error\n",bits);
            return ret;
        }
        this->bits = bits;
        return ret;
    }
    uint8_t ASPI::spi_rxtx(uint8_t data)
    {
        int ret  = 0;
        unsigned char sendData;
        unsigned char recvData = 0;
        uint8_t *recvDataP = NULL;
        uint8_t *sendDataP = NULL;
        sendData = data;

        recvDataP = &recvData;// write
        sendDataP = &sendData;

        struct spi_ioc_transfer tr;
        tr.tx_buf = (unsigned long)sendDataP;
        tr.rx_buf = (unsigned long)recvDataP;
        tr.len = 1;
        tr.delay_usecs = delay;
        tr.speed_hz = speed;
        tr.bits_per_word = bits;
        //.cs_change = 1,

        ret = ioctl(spiFd, SPI_IOC_MESSAGE(1), &tr);
        if(ret < 0)// Need fix here.....
        {
            LOG2("androidUAV:transfer failed\n");
            return ret;
        }
        return recvData;
    }
    int ASPI::init()
    {
        return 0;
    }
    int ASPI::set_speed(int speed)
    {
        return setSpeed(speed);
    }
    int ASPI::set_mode(int CPOL, int CPHA)
    {

        if(!CPOL&&!CPHA)
        {
            setMode(0);
        }
        else if(!CPOL&&CPHA)
        {
            setMode(1);
        }
        else if(CPOL&&!CPHA)
        {
            setMode(2);
        }
        else
        {
            setMode(3);
        }
        return 0;
    }
    uint8_t ASPI::txrx(uint8_t data)
    {
        return spi_rxtx(data);
    }
    uint8_t ASPI::txrx2(const uint8_t *tx, uint8_t *rx, int len)
    {
        int ret  = 0;
        /*unsigned char sendData;
        unsigned char recvData = 0;
        uint8_t *recvDataP = NULL;
        uint8_t *sendDataP = NULL;

        recvDataP = &recvData;// write
        sendDataP = &sendData;*/

        struct spi_ioc_transfer tr;
        tr.tx_buf = (unsigned long)tx;
        tr.rx_buf = (unsigned long)rx;
        tr.len = len;
        tr.delay_usecs = delay;
        tr.speed_hz = speed;
        tr.bits_per_word = bits;
        //.cs_change = 1,
        ret = ioctl(spiFd, SPI_IOC_MESSAGE(1), &tr);
        if(ret < 0)// Need fix here.....
        {
            LOG2("androidUAV:transfer failed\n");
            return ret;
        }
        return len;
    }
}

