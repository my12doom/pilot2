#pragma once
#include <stdint.h>
#include "Storage.h"
using namespace HAL;
namespace STM32F4
{
	class F4Storage:public Storage
	{
		/*
		
		
		#define START_ADDRESS 0x080C0000
		define PAGE_SIZE 0x20000
		#define BUFFER_SIZE 0x40000	
		#define FRESH_VALUE 0xFFFFFFFF
		*/
	private:
		int min(int a, int b);
		uint32_t page__size;
		uint32_t buffer_size;
		uint32_t start_address;
	public:
		F4Storage(uint32_t page__size,uint32_t buffer_size,uint32_t start_address);
		~F4Storage(){};
		virtual int init();
		virtual int total_size();
		virtual int page_size();
		virtual int erase(int address);
		virtual int write(int address, const void *data, int size);
		virtual int read(int address, void *data, int maxsize);
	};
}