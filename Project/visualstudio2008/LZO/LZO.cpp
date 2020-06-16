#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <protocol/crc32.h>
#include <windows.h>
extern "C" 
{
#include <utils/minilzo.h>
}

#define HEAP_ALLOC(var,size) \
	lzo_align_t __LZO_MMODEL var [ ((size) + (sizeof(lzo_align_t) - 1)) / sizeof(lzo_align_t) ]

static HEAP_ALLOC(wrkmem, LZO1X_1_MEM_COMPRESS);

uint32_t dec_crc;
uint8_t *compressed;
lzo_uint compressed_len;
lzo_uint decompressed_len;
uint32_t in_crc = 0;
int blk_cb(lzo_callback_p hcb, lzo_voidp ptr, lzo_uint out_size, lzo_uint read_size)
{
	if (read_size > 0)
	{
		if (read_size > compressed_len)
			read_size = compressed_len;
		memcpy(ptr, compressed, read_size);
		compressed += read_size;
		compressed_len -= read_size;

		in_crc = crc32(in_crc, ptr, read_size);
		
		return read_size;
	}

	dec_crc = crc32(dec_crc, ptr, out_size);
	decompressed_len += out_size;
	return 0;
}

typedef struct 
{
	uint16_t magic;
	uint16_t version;
	uint32_t size;
	uint32_t compressed_size;
	uint32_t crc;
	uint32_t compressed_crc;
	uint8_t padding[512-20];
}LZO_HEADER;


#define ERROR_RTN(x) {rtn=x; goto clearup;}
int check_lzo_file(const char *filename)
{

	FILE * f = fopen(filename, "rb");
	if (!f)
	{
		printf("failed opening file %s\n", filename);
		return -1;
	}

	int rtn = 0;
	uint8_t *data = NULL;
	LZO_HEADER header;
	if (fread(&header, 1, sizeof(header), f) != sizeof(header))
	{
		printf("error reading file header\n");
		ERROR_RTN(-1);
	}

	if (header.magic != 0xA385 || header.version != 0)
	{
		printf("bad header\n");
		ERROR_RTN(-1);
	}

	data = new uint8_t[header.compressed_size];
	if (!data)
	{
		printf("error allocating %d bytes, bad file?\n");
		ERROR_RTN(-1);
	}

	if (fread(data, 1, header.compressed_size, f) != header.compressed_size)
	{
		printf("error reading %d bytes from file, bad file?\n");
		ERROR_RTN(-1);
	}



	if (crc32(0, data, header.compressed_size) != header.compressed_crc)
	{
		printf("bad compressed crc32\n");
		ERROR_RTN(-1);
	}

	dec_crc = 0;
	decompressed_len = 0;
	compressed = data;
	compressed_len = header.compressed_size;
	int blk_size = 512;
	lzo1x_decompress_stream((unsigned char*)wrkmem, sizeof(wrkmem), blk_cb, blk_size);

	if (decompressed_len != header.size)
	{
		printf("bad decompressed size\n");
		ERROR_RTN(-1);
	}

	if (dec_crc != header.crc)
	{
		printf("bad decompressed crc\n");
		ERROR_RTN(-1);
	}

clearup:
	compressed = NULL;
	if (data)
		delete [] data;
	if (f)
		fclose(f);
	return rtn;
}

int main(int argc, char *argv[])
{
	if (argc<3)
	{
		printf("LZO.exe test.bin test.lzo\n");

		return 0;
	}

	FILE * f = fopen(argv[1], "rb");
	if (!f)
	{
		printf("failed opening file %s\n", argv[1]);
		return -1;
	}

	FILE * of = fopen(argv[2], "wb");
	if (!of)
	{
		printf("failed opening file %s\n", argv[2]);
		return -1;
	}

	fseek(f, 0, SEEK_END);
	int data_size = ftell(f);
	fseek(f, 0, SEEK_SET);
	uint8_t *data = new uint8_t[data_size];
	fread(data, 1, data_size, f);
	fclose(f);

	LZO_HEADER header = {0xA385, 0};

	header.size = data_size;
	header.crc = crc32(0, data, data_size);

	compressed = new uint8_t[data_size*2];
	compressed_len = 0;
	lzo_init();
	int r = lzo1x_1_compress(data, data_size, compressed, &compressed_len, wrkmem);
	header.compressed_crc = crc32(0, compressed, compressed_len);
	header.compressed_size = compressed_len;

	fwrite(&header, 1, sizeof(header), of);
	fwrite(compressed, 1, compressed_len, of);
	fclose(of);


	printf("done, %d -> %d, %.2f%%\n", data_size, compressed_len, compressed_len*100.0f/(data_size));

	check_lzo_file(argv[2]);
	
	delete [] data;
	delete [] compressed;

	return 0;
}
