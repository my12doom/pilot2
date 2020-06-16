#include "CompressedFile.h"
#include <HAL/Interface/ISysTimer.h>
#include <FileSystem/ff.h>
#include <stdio.h>
#include <string.h>
#include <protocol/crc32.h>
extern "C" {
#include <utils/minilzo.h>
}


const int decompress_blk_size = 1024;
uint8_t wrkmem[MAX_BS_LEN+decompress_blk_size*4];
uint8_t blk[decompress_blk_size];

FIL fin;
FIL fout;
uint32_t dec_crc = 0;
lzo_uint decompressed_len;

UINT pos = 0;
uint32_t in_crc = 0;
int decompress_blk_cb(lzo_callback_p hcb, lzo_voidp ptr, lzo_uint out_size, lzo_uint read_size)
{
	if (read_size)
	{		
		UINT got = 0;
		
		pos = f_tell(&fin);
		
		// undefined alignment workaround
		f_read(&fin, blk, read_size, &got);
		memcpy(ptr, blk, read_size);
		
		return got;
	}
	
	if (out_size)
	{
		dec_crc = crc32(dec_crc, ptr, out_size);
		decompressed_len += out_size;
		
		// alignment OK
		f_write(&fout, ptr, out_size, NULL);
	}
	
	return 0;
}

#define ERROR_RTN(x) {rtn=x; delete_out=x; goto clearup;}

int decompress_file(const char *src, const char *dst)
{
	int64_t t = systimer->gettime();
	// open file
	int rtn = 0;
	bool delete_out = false;
	if ( f_open(&fin, src, FA_READ | FA_OPEN_EXISTING) != FR_OK)
		return -1;

	if ( f_open(&fout, dst, FA_CREATE_ALWAYS | FA_WRITE | FA_READ) != FR_OK)
	{
		f_close(&fin);
		return -1;
	}
	
	// check header
	LZO_HEADER header;
	UINT got = 0;
	UINT left;// = header.compressed_size;
	uint32_t crc = 0;
	
	if (f_read(&fin, &header, sizeof(header), &got) != FR_OK || got != sizeof(header))
	{
		printf("error reading file header\n");
		ERROR_RTN(-2);
	}

	if (header.magic != 0xA385 || header.version != 0)
	{
		printf("bad header\n");
		ERROR_RTN(-3);
	}
	
	if (header.compressed_size + sizeof(header) > f_size(&fin))
	{
		printf("file has less bytes than header indicated size \n");
		ERROR_RTN(-3);
	}
	
	// calculate & check compressed crc32
	left = header.compressed_size;
	while (left)
	{
		int blk_size = left > sizeof(wrkmem) ? sizeof(wrkmem) : left;
		if (f_read(&fin, wrkmem, blk_size, &got) != FR_OK || got != blk_size)
		{
			printf("error reading file content\n");
			ERROR_RTN(-4);
		}
		crc = crc32(crc, wrkmem, blk_size);
		left -= blk_size;
	}
	f_lseek(&fin, sizeof(header));
	
	if (crc != header.compressed_crc)
	{
		printf("bad compressed crc32\n");
		ERROR_RTN(-5);
	}
	
	// allocate space and enable fast seek
	/*
	f_lseek(&fout, header.size-4);
	if (f_write(&fout, &header.size, 4, &got) != FR_OK || got != 4)
	{
		printf("failed allocating %d bytes for output\n", header.size);
		ERROR_RTN(-6);
	}
	const int SZ_TBL = 4096;
	DWORD lktbl[SZ_TBL];
	fout.cltbl = lktbl;
	lktbl[0] = SZ_TBL;
	f_lseek(&fout, CREATE_LINKMAP);
	*/
	f_lseek(&fout, 0);
	
	
	// decompress
	dec_crc = 0;
	decompressed_len = 0;
	lzo1x_decompress_stream((unsigned char*)wrkmem, sizeof(wrkmem), decompress_blk_cb, decompress_blk_size);
	
	// check result
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
	
	// check generated file crc
	dec_crc = 0;
	f_lseek(&fout, 0);
	left = header.size;
	while (left)
	{
		int blk_size = left > sizeof(wrkmem) ? sizeof(wrkmem) : left;
		if (f_read(&fout, wrkmem, blk_size, &got) != FR_OK || got != blk_size)
		{
			printf("error reading decompressed file content\n");
			ERROR_RTN(-4);
		}
		dec_crc = crc32(dec_crc, wrkmem, blk_size);
		left -= blk_size;
	}
	if (dec_crc != header.crc)
	{
		printf("bad decompressed crc (file writing error)\n");
		ERROR_RTN(-1);
	}

clearup:
	printf("decompress:%lld us\n", systimer->gettime() - t);
	f_close(&fin);
	f_close(&fout);
	if (delete_out)
		f_unlink(dst);
	return rtn;
}
