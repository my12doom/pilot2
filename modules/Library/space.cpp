#include "space.h"
#include <string.h>
#include <HAL/Interface/Interfaces.h>

using namespace HAL;
IStorage *param_storage = default_storage;
const int page_size = param_storage->page_size();
const int page_count = default_storage->total_size()/ default_storage->page_size();
const int space_size = (page_count-1)*default_storage->page_size();
const int max_key_size = 8;

const unsigned short start_code = 0x85a3;
const unsigned short end_code = 0xa385;
const unsigned int fresh_value = 0xffffffff;

// unsigned char *p = new unsigned char[buffer_size];
int write_pointer = 0;
int resorting = 0;

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

int find_next_entry(int start_pos);
int space_init();
int find_next_entry(int start_pos);
int space_virtual_read(int address, void *data, int size);


static int min(int a, int b)
{
	if (a>b)
		return b;
	return a;
}

int space_virtual_read(int address, void *data, int size)
{
	if (!resorting)
		return param_storage->read(address, data, size);

	int count = 0;
	int res = 0;

	// when resorting, the first page is copied into last page.
	if (address < page_size)
	{
		int size1 = min(size, page_size-address);

		res = param_storage->read((page_count-1)*page_size + address, data, size1) ;
		if (res<0)
			return res;

		address = page_size;
		data = (char*)data + size1;
		size -= size1;
		count = size1;
	}

	res = param_storage->read(address, data, size);
	if (res<0)
		return res;

	return count+res;
}

int space_init(bool erase/* = false*/)
{
	param_storage->init();

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
	if (erase || write_pointer == 0)
	{
		for(int i=0; i<page_count; i++)
			param_storage->erase(i*page_size);

		int meta_data = 0;

		// create a meta entry, meta footer and write it to space
		entry_header meta_header = {start_code, 0, sizeof(meta_data)};
		param_storage->write(0, &meta_header, sizeof(meta_header) - sizeof(meta_header.delete_tag));

		param_storage->write(sizeof(meta_header), &meta_data, sizeof(meta_data));

		entry_footer meta_footer = {sizeof(meta_footer), end_code};
		param_storage->write(sizeof(meta_header) + sizeof(meta_data), &meta_footer, sizeof(meta_footer));

		// move write pointer
		write_pointer = sizeof(meta_header) + sizeof(meta_footer) + sizeof(meta_data);
	}

	return 0;
}

// find next matching pair of entry header and footer
// return the header position if found, 
// return -1 if not found, -2 on other errors
int find_next_entry(int start_pos)
{
	entry_header header;
	entry_footer footer;

	start_pos = start_pos/4*4;
	
	while(start_pos < space_size - sizeof(entry_header) - sizeof(entry_footer))
	{
		if (space_virtual_read(start_pos, &header, sizeof(header)) < 0)
			return -1;

		if (header.start_code == start_code)
		{
			int footer_pos = start_pos+sizeof(header)+ (header.key_size+3)/4*4 + (header.data_size+3)/4*4;

			if (footer_pos > 0 && footer_pos < space_size)
				if (space_virtual_read(footer_pos, &footer, sizeof(footer)) < 0)
					return -1;

			if (footer.end_code == end_code && footer.all_size == (header.key_size+3)/4*4 + (header.data_size+3)/4*4)
				return start_pos;
		}
		start_pos += 4;
	}

	return -1;
}

// find previous matching pair of entry header and footer
// return the header position if found, 
// return -1 if not found, -2 on other errors
int find_previous_entry(int start_pos)
{
	entry_header header;
	entry_footer footer;

	start_pos = start_pos/4*4;

	while(start_pos > sizeof(entry_header) + sizeof(entry_footer))
	{
		if (space_virtual_read(start_pos, &footer, sizeof(footer)) < 0)
			return -1;

		if (footer.end_code == end_code)
		{
			int header_pos = start_pos - footer.all_size - sizeof(header);
			if (header_pos > 0 && header_pos < space_size)
				if (space_virtual_read(header_pos, &header, sizeof(header))<0)
					return -1;

			if (header.start_code == start_code && footer.all_size == (header.key_size+3)/4*4 + (header.data_size+3)/4*4)
				return header_pos;
		}
		start_pos -= 4;
	}

	return -1;
}

int space_read(const void *key, int keysize, void *data, int num_to_read, int *num_read)
{
	int pos = find_previous_entry(write_pointer);

	if (keysize > max_key_size)
		return -2;
	
	while (pos>0)		// >0 instead of >=0, the meta data is not returned
	{
		entry_header header;
		space_virtual_read(pos, &header, sizeof(header));

		if (header.key_size == keysize && header.delete_tag == fresh_value)
		{
			char key_read[max_key_size];
			space_virtual_read(pos + sizeof(header), &key_read, keysize);

			// key matched
			if (memcmp(key_read, key, keysize) == 0)
			{
				int count = min(num_to_read, header.data_size);
				space_virtual_read(pos + sizeof(header) + (header.key_size+3)/4*4, data, count);

				if (num_read)
					*num_read = count;

				return 0;
			}
		}

		pos = find_previous_entry(pos - sizeof(entry_footer));
	}

	return -1;
}

int space_delete(const void *key, int keysize)
{
	int pos = find_previous_entry(write_pointer);

	if (keysize > max_key_size)
		return -2;

	while (pos>0)		// >0 instead of >=0, the meta data is not returned
	{
		entry_header header;
		space_virtual_read(pos, &header, sizeof(header));

		if (header.key_size == keysize && header.delete_tag == fresh_value)
		{
			char key_read[max_key_size];
			space_virtual_read(pos + sizeof(header), &key_read, keysize);

			// key matched
			if (memcmp(key_read, key, keysize) == 0)
			{
				int delete_tag = ~fresh_value;
				return param_storage->write(pos + sizeof(header) - sizeof(header.delete_tag), &delete_tag, sizeof(header.delete_tag));
			}
		}

		pos = find_previous_entry(pos - sizeof(entry_footer));
	}

	return 0;
}

int space_available()
{
	return space_size - write_pointer - sizeof(entry_header) - sizeof(entry_footer);
}

int space_write(const void *key, int keysize, const void *data, int num_to_write, int *num_written)
{
	int keysize4 = (keysize+3)/4*4;
	int datasize4 = (num_to_write+3)/4*4;

	// delete old entry first
	if (space_delete(key, keysize) < 0)
		return -3;

	// do resort if not enough space
	if (space_available() < keysize4 + datasize4)
		space_resort();

	if (keysize4 > max_key_size || datasize4+keysize4 > space_available())		// we may really out of space
		return -2;

	entry_header header = {start_code, keysize, num_to_write};
	entry_footer footer = {keysize4 + datasize4, end_code};

	
	int result = param_storage->write(write_pointer, &header, sizeof(header) - sizeof(header.delete_tag));		// don't write the delete tag
	write_pointer += sizeof(header);
	if (result < 0)
		return result;

	result = param_storage->write(write_pointer, key, keysize);
	write_pointer += keysize4;
	if (result < 0)
		return result;

	result = param_storage->write(write_pointer, data, num_to_write);
	write_pointer += datasize4;
	if (result < 0)
		return result;

	result = param_storage->write(write_pointer, &footer, sizeof(footer));
	write_pointer += sizeof(footer);
	if (result < 0)
		return result;

	if (num_written)
		*num_written = num_to_write;

	return 0;
}

int space_resort()
{
	printf("--- resorting ---\n");

	char buf[1024];

	// copy first page to last page
	param_storage->erase((page_count-1)*page_size);
	for(int i=0; i<page_size/sizeof(buf); i++)
	{
		if (param_storage->read(i*sizeof(buf), buf, sizeof(buf)) < 0)
			return -1;
		if (param_storage->write((page_count-1)*page_size + i*sizeof(buf), buf, sizeof(buf)) < 0)
			return -2;
	}

	// enable resorting mode, which map last page to first page
	resorting = 1;

	// copy all entries except deleted ones.
	int search_pos = 0;
	int page_erased = -1;
	int free_space_start = (write_pointer-1)/page_size;
	write_pointer = 0;
	while((search_pos = find_next_entry(search_pos)) >= 0)
	{
		// read header
		entry_header header;
		if (space_virtual_read(search_pos, &header, sizeof(header)) < 0)
			return -3;

		// check delete 
		if (header.delete_tag == fresh_value)
		{
			// read entire entry out
			int size = space_virtual_read(search_pos, buf, sizeof(entry_header) + sizeof(entry_footer) + (header.key_size+3)/4*4 + (header.data_size+3)/4*4);
			if (size<0)
				return -4;

			// erase pages
			int end_pos = write_pointer + size;
			int start_page = write_pointer/page_size;
			int end_page = end_pos/page_size;

			if (start_page>page_erased)
			{
				page_erased = start_page;
				param_storage->erase(page_erased*page_size);
			}

			if (end_page>page_erased)
			{
				page_erased = end_page;
				param_storage->erase(page_erased*page_size);
			}

			// write back, except delete tag
			int res = param_storage->write(write_pointer, buf, sizeof(header)-sizeof(header.delete_tag));
			if (res<0)
				return -5;

			write_pointer += sizeof(header);
			size = param_storage->write(write_pointer, buf+sizeof(header), size-sizeof(header));
			if (size<0)
				return -5;

			write_pointer += size;
		}

		search_pos = search_pos + sizeof(entry_header) + sizeof(entry_footer) + (header.data_size+3)/4*4 + (header.key_size+3)/4*4;
	}

	// disable resorting mode
	resorting = 0;

	// erase empty space
	int free_space_start_new = (write_pointer+page_size-1)/page_size;
	for(int i=free_space_start_new; i<=free_space_start; i++)
		param_storage->erase(i*page_size);

	param_storage->erase((page_count-1)*page_size);

	return 0;
}
