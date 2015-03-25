#include "F4Storage.h"
#include "stm32f4xx_flash.h"
#include <stdint.h>
#include "string.h"
using namespace HAL;
namespace STM32F4
{
	F4Storage::F4Storage():write_pointer(0),page_count(BUFFER_SIZE/PAGE_SIZE),space_size((page_count-1)*PAGE_SIZE),resorting(0)
	{
		init();
	}
	int F4Storage::min(int a, int b)
	{
		if (a>b)
			return b;
		return a;
	}
	int F4Storage::space_raw_read(int address, void *data, int size)
	{
		int count = min(size, BUFFER_SIZE - address);
		memcpy(data, (char*)START_ADDRESS+address, count);
		return count;
	}
	int F4Storage::space_virtual_read(int address, void *data, int size)
	{
		if (!resorting)
			return space_raw_read(address, data, size);

		int count = 0;
		int res = 0;

		// when resorting, the first page is copied into last page.
		if (address < PAGE_SIZE)
		{
			int size1 = min(size, PAGE_SIZE-address);

			res = space_raw_read((page_count-1)*PAGE_SIZE + address, data, size1) ;
			if (res<0)
				return res;
	
			address = PAGE_SIZE;
			data = (char*)data + size1;
			size -= size1;
			count = size1;
		}

		res = space_raw_read(address, data, size);
		if (res<0)
			return res;

		return count+res;
	}
	int F4Storage::find_next_entry(int start_pos)
	{
		entry_header header;
		entry_footer footer;

		start_pos = start_pos/4*4;

		while(start_pos < space_size - sizeof(entry_header) - sizeof(entry_footer))
		{
			if (space_virtual_read(start_pos, &header, sizeof(header)) < 0)
			return -1;
				
		if (header.start_code == START_CODE)
		{
			int footer_pos = start_pos+sizeof(header)+ (header.key_size+3)/4*4 + (header.data_size+3)/4*4;

			if (footer_pos > 0 && footer_pos < space_size)
				if (space_virtual_read(footer_pos, &footer, sizeof(footer)) < 0)
					return -1;

			if (footer.end_code == END_CODE && footer.all_size == (header.key_size+3)/4*4 + (header.data_size+3)/4*4)
				return start_pos;
		}
		start_pos += 4;
		}

		return -1;
	}
	int F4Storage::space_raw_erase(int address)
	{
		memset(p+ address / PAGE_SIZE * PAGE_SIZE, 0xff, PAGE_SIZE);
		return 0;
	}
	int F4Storage::space_raw_write(int address, const void *data, int size)
	{
		int count = min(size, BUFFER_SIZE - address);
		memcpy(p+address, data, count);
		return count;
	}
	void F4Storage::space_raw_init()
	{
		FLASH_Unlock();
	}
	int F4Storage::init()
	{
		space_raw_init();

		// try all any data entries
		int find_pointer = 0;
		write_pointer = 0;

		while((find_pointer = find_next_entry(find_pointer)) >= 0)
		{
			entry_header header;
			space_virtual_read(find_pointer, &header, sizeof(header));

			find_pointer = find_pointer + sizeof(entry_header) + sizeof(entry_footer) + (header.data_size+3)/4*4 + (header.key_size+3)/4*4;
			write_pointer = find_pointer;
		}

		// if none found, format it and create a NULL entry
		if (IF_ERASE || write_pointer == 0)
		{
			for(int i=0; i<page_count; i++)
				space_raw_erase(i*PAGE_SIZE);

			int meta_data = 0;

			// create a meta entry, meta footer and write it to space
			entry_header meta_header = {START_CODE, 0, sizeof(meta_data)};
			space_raw_write(0, &meta_header, sizeof(meta_header) - sizeof(meta_header.delete_tag));

			space_raw_write(sizeof(meta_header), &meta_data, sizeof(meta_data));

			entry_footer meta_footer = {sizeof(meta_footer),END_CODE};
			space_raw_write(sizeof(meta_header) + sizeof(meta_data), &meta_footer, sizeof(meta_footer));

			// move write pointer
			write_pointer = sizeof(meta_header) + sizeof(meta_footer) + sizeof(meta_data);
		}

		return 0;
	}
	int F4Storage::total_size()
	{
		return  BUFFER_SIZE;
	}
	int F4Storage::page_size()
	{
		return  PAGE_SIZE;
	}
	int F4Storage::write(int address, const void *data, int size)
	{
		int count = min(size, BUFFER_SIZE - address);
		char *p = (char*)data;

		for(int i=0; i<count/4*4; i+=4)
		FLASH_ProgramWord(START_ADDRESS+address+i, *(uint32_t*)(p+i));

		if (count %4)
		{
		uint32_t pending = 0xffffffff;
		memcpy(&pending, p+count/4*4, count%4);
		FLASH_ProgramWord(START_ADDRESS+address+count/4*4, pending);
		}

		FLASH_WaitForLastOperation();
		return count;
	}
	int F4Storage::read(int address, void *data, int maxsize)
	{
		int count = min(maxsize, BUFFER_SIZE - address);
		memcpy(data, (char*)START_ADDRESS+address, count);
		return count;
	}
	
}
