#pragma once
#include <stdint.h>
#include <HAL/Interface/IStorage.h>

namespace STM32F0
{
	class F0Storage:public HAL::IStorage
	{
	protected:
		int min(int a, int b);
		uint32_t page__size;
		uint32_t buffer_size;
		uint32_t start_address;
	public:
		F0Storage(uint32_t page__size=1024,uint32_t buffer_size=4096,uint32_t start_address=0x0800F000);
		~F0Storage(){};
		virtual int init();
		virtual int total_size();
		virtual int page_size();
		virtual int erase(int address);
		virtual int write(int address, const void *data, int size);
		virtual int read(int address, void *data, int maxsize);
	};
	
	class BootloaderStorage:public F0Storage
	{
	public:
		BootloaderStorage();
		~BootloaderStorage(){};
		virtual int erase(int address);
	};
	
	class RCStorage:public F0Storage
	{
	public:
		RCStorage();
		~RCStorage(){};
		virtual int erase(int address);
	};
}