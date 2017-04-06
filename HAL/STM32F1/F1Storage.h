#pragma once
#include <stdint.h>
#include <HAL/Interface/IStorage.h>

namespace STM32F1
{
	class F1Storage:public HAL::IStorage
	{
	protected:
		int min(int a, int b);
		uint32_t page__size;
		uint32_t buffer_size;
		uint32_t start_address;
	public:
		F1Storage(uint32_t page__size=1024,uint32_t buffer_size=4096,uint32_t start_address=0x0800F000);
		~F1Storage(){};
		virtual int init();
		virtual int total_size();
		virtual int page_size();
		virtual int erase(int address);
		virtual int write(int address, const void *data, int size);
		virtual int read(int address, void *data, int maxsize);
	};
	
	class BootloaderStorage:public F1Storage
	{
	public:
		BootloaderStorage();
		~BootloaderStorage(){};
		virtual int erase(int address);
	};
	
	class RCStorage:public F1Storage
	{
	public:
		RCStorage();
		~RCStorage(){};
		virtual int erase(int address);
	};
}