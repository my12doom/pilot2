#pragma once
#include <HAL/Interface/IUART.h>

namespace androidUAV
{
    class AUART : public HAL::IUART
    {
        private:

        public:
        virtual int set_baudrate(int baudrate);
        virtual int peak(void *data, int max_count);
        virtual int write(const void *data, int count);
        virtual int flush();
        virtual int read(void *data, int max_count);
        virtual int readline(void *data, int max_count);
        virtual int available();
    };
}
