#pragma once
#include <stdint.h>
#include <HAL/Interface/IStorage.h>

namespace STM32F4
{
	class F4Storage:public HAL::IStorage
	{
	protected:
		int min(int a, int b);
		uint32_t page__size;
		uint32_t buffer_size;
		uint32_t start_address;
	public:
		F4Storage(uint32_t page__size=0x20000,uint32_t buffer_size=0x40000,uint32_t start_address=0x080C0000);
		~F4Storage(){};
		virtual int init();
		virtual int total_size();
		virtual int page_size();
		virtual int erase(int address);
		virtual int write(int address, const void *data, int size);
		virtual int read(int address, void *data, int maxsize);
	};
	
	class BootloaderStorage:public F4Storage
	{
	public:
		BootloaderStorage();
		~BootloaderStorage(){};
		virtual int erase(int address);
	};
}