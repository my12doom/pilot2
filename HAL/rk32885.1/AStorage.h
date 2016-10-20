#pragma once
#include <stdio.h>
#include <stdint.h>
#include <wchar.h>
#include <HAL/Interface/IStorage.h>

//Linux OS wchar_t is 4 bytes add compile option:-fshort-wchar set wchar_t occupy 2 bytes
//Windows OS wchar_t is 2 bytes
#ifndef min
#define min(a,b) (((a)<(b))?(a):(b))
#endif

#ifndef max
#define max(a,b) (((a)>(b))?(a):(b))
#endif
namespace androidUAV
{
    class AStorage: public HAL::IStorage
    {
        public:
            AStorage(const char*filename = NULL);
            ~AStorage(){};
            virtual int init();
            virtual int total_size();
            virtual int page_size();
            virtual int erase(int address);
            virtual int write(int address, const void *data, int size);
            virtual int read(int address, void *data, int maxsize);

            FILE *f;
    };
}
