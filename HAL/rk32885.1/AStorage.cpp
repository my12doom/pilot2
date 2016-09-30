#include "AStorage.h"
#include <linux/limits.h>
namespace androidUAV
{
    AStorage::AStorage(const wchar_t*filename)
    {
        wchar_t _filename[NAME_MAX] = {0};
        if(filename)
            wcscpy(_filename,filename);
        else
        {


        }
    }
    int AStorage::init()
    {
        return 0;
    }
    int AStorage::total_size()
    {
return 0;
    }
    int AStorage::page_size()
    {
return 0;
    }
    int AStorage::erase(int address)
    {
return 0;
    }
    int AStorage::write(int address, const void *data, int size)
    {
return 0;
    }
    int AStorage::read(int address, void *data, int maxsize)
    {
return 0;
    }

}
