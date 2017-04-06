#include "F1Storage.h"
#include "stm32F10x_flash.h"
#include <stdint.h>
#include "string.h"
using namespace HAL;
namespace STM32F1
{
	F1Storage::F1Storage(uint32_t page__size,uint32_t buffer_size,uint32_t start_address)
	{
		this->page__size=page__size;
		this->buffer_size=buffer_size;
		this->start_address=start_address;
	}
	int F1Storage::init()
	{
		FLASH_Unlock();
		return 0;
	}
	int F1Storage::min(int a, int b)
	{
		if (a>b)
			return b;
		return a;
	}
	int F1Storage::erase(int address)
	{
		FLASH_Status res = FLASH_ErasePage(start_address + address);
		return res;
	}
	int F1Storage::total_size()
	{
		return buffer_size;
	}
	int F1Storage::page_size()
	{
		return page__size;
	}
	int F1Storage::write(int address, const void *data, int size)
	{
		int count = min(size, buffer_size - address);
		char *p = (char*)data;

		for(int i=0; i<count/4*4; i+=4)
		FLASH_ProgramWord(start_address+address+i, *(uint32_t*)(p+i));

		if (count %4)
		{
			uint32_t pending = 0xffffffff;
			memcpy(&pending, p+count/4*4, count%4);
			FLASH_ProgramWord(start_address+address+count/4*4, pending);
		}

		FLASH_WaitForLastOperation(100);
		return count;
	}
	int F1Storage::read(int address, void *data, int maxsize)
	{
		int count = min(maxsize, buffer_size - address);
		memcpy(data, (char*)start_address+address, count);
		return count;
	}	
}

HAL::IStorage *get_default_storage()
{
	static STM32F1::F1Storage theDefaultStorage;
	
	return &theDefaultStorage;
}

HAL::IStorage *get_bootloader_storage()
{
	return NULL;
}
