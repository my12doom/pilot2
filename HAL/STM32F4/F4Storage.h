#pragma once
#include <stdint.h>
#include "Storage.h"
using namespace HAL;
namespace STM32F4
{
	class F4Storage:Storage
	{
		#define IF_ERASE 0
		
		#define START_ADDRESS 0x080C0000
		#define PAGE_SIZE 0x20000
		#define BUFFER_SIZE 0x40000
		#define START_CODE 0x85a3
		#define END_CODE 0xa385
		#define FRESH_VALUE 0xFFFFFFFF
		typedef struct
		{
			unsigned short start_code;
			unsigned char key_size;
			unsigned char data_size;
			unsigned int delete_tag;
		} entry_header;
		typedef struct
		{
			unsigned short all_size;
			unsigned short end_code;
		} entry_footer;
	private:
		int write_pointer;
		int page_count;
		int space_size;
		
		int resorting;
		unsigned char p[BUFFER_SIZE];
		int min(int a, int b);
		int space_raw_erase(int address);
		int space_raw_write(int address, const void *data, int size);
		int space_raw_read(int address, void *data, int size);
		void space_raw_init();
		int find_next_entry(int start_pos);
		int space_virtual_read(int address, void *data, int size);
	public:
		F4Storage();
		~F4Storage(){};
		virtual int init();
		virtual int total_size();
		virtual int page_size();
		virtual int write(int address, const void *data, int size);
		virtual int read(int address, void *data, int maxsize);
	};
}