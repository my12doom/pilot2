#include "AStorage.h"
#include <linux/limits.h>
#include <string.h>
#include <unistd.h>
namespace androidUAV
{
    AStorage::AStorage(const char*filename)
    {
        char _filename[NAME_MAX] = {0};
        if(filename)
            strcpy(_filename,filename);
        else
        {
            readlink("/proc/self/exe",_filename,NAME_MAX);

            strcpy(strrchr(_filename,'/'),"/defaultstorage");
            //printf("exe path = %s\n",_filename);
        }
        f = fopen(_filename,"rb");
        if(!f)
            fopen(_filename,"wb");
    }
    int AStorage::init()
    {
        return 0;
    }
    int AStorage::total_size()
    {
        return page_size()*3;
    }
    int AStorage::page_size()
    {
        return 128*1024;
    }
    int AStorage::erase(int address)
    {
        if( !f || address >= total_size())
            return -1;
        address = address/page_size()*page_size();
        fseek(f,address,SEEK_SET);

        char *tmp = new char[page_size()];
        memset(tmp,0xff,page_size());
        fwrite(tmp,1,page_size(),f);
        fflush(f);
        delete[] tmp;
        return 0;
    }
    int AStorage::write(int address, const void *data, int size)
    {
        int ret_size;
        if(address < 0 || address+size >=total_size() || size < 0)
            return -1;
        fseek(f,address,SEEK_SET);
        ret_size = fwrite(data,1,size,f);
        fflush(f);
        return ret_size;
    }
    int AStorage::read(int address, void *data, int maxsize)
    {
        int ret_size;
        int size = min(maxsize,total_size() - address);
        if(address<0 || address >= total_size() || size<0)
            return -1;
        fseek(f,address,SEEK_SET);
        fread(data,1,size,f);
        fflush(f);
        return ret_size;
    }

}
HAL::IStorage *get_default_storage()
{
    static androidUAV::AStorage theDefaultStorage;
    return &theDefaultStorage;
}
HAL::IStorage *get_bootloader_storage()
{
    return NULL;
}
