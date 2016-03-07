#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <protocol/crc32.h>
#include <windows.h>
#include <utils/RIJNDAEL.H>


int gen_bin(const char *filename, const char*filename_out, bool encrypt = false)
{
	AESCryptor aes;
	const unsigned char aes_key[32] = {0x85, 0xA3, 0x6C, 0x69, 0x76, 0x6C, 0x61, 0x76, 0xA3, 0x85};
	aes.set_key(aes_key, 256);

	FILE * f = fopen(filename, "rb");
	if (!f)
	{
		printf("failed opening %s\n", filename);
		return 0;
	}
	fseek(f, 0, SEEK_END);
	int rom_size = ftell(f);

	char *data = new char[rom_size+2048];
	memset(data, 0, rom_size+2048);
	fseek(f, 0, SEEK_SET);
	fread(data, 1, rom_size, f);
	fclose(f);

	f = fopen(filename_out, "wb");
	if (!f)
	{
		printf("failed opening %s\n", filename_out);
		return 0;
	}

	if (!encrypt)
	{
		// write rom directly
		fwrite(data, 1, rom_size, f);


		// write crc and yap tag
		fwrite("YAP ", 1, 4, f);
		uint32_t crc = crc32(0, data, rom_size);
		fwrite(&crc, 1, 4, f);
		fclose(f);
	}
	else
	{
		// do the encrypting and write a encrypted tag
		unsigned char encrypted_tag[4] = {0x85, 0xA3, 0xA3, 0x85};

		rom_size = (rom_size-4+1023)/1024*1024+1024+4;
		int block_count = (rom_size-4)/16;
		uint32_t crc_rom = crc32(0, data+4, rom_size-4-1024);
		memcpy(data+rom_size-1024, &crc_rom, 4);
		memcpy(data+rom_size-1024+4, data, 4);
		memcpy(data, encrypted_tag, 4);

		for(int i=0; i<block_count; i++)
			aes.encrypt((unsigned char*)data+4+i*16, (unsigned char*)data+4+i*16);

		fwrite(data, 1, block_count*16+4, f);

		// write tag and crc
		fwrite("YAP ", 1, 4, f);
		uint32_t crc_file = crc32(0, data, block_count*16+4);
		fwrite(&crc_file, 1, 4, f);
		fclose(f);
	}

	return 0;
}

int main(int argc, char *argv[])
{
// 	merge("Z:\\bootloader.bin", 0x8000, "Z:\\pilot.bin", "Z:\\total.hex");
	if (argc<2)
	{
		printf("BIN2YAP a.bin");

		return 0;
	}

	char yap[1024];
	strcpy(yap, argv[1]);
	if (strrchr(yap, '\\'))
		strcpy((char*)strrchr(yap, '\\'), "\\firmware.yap");
	if (strrchr(yap, '/'))
		strcpy((char*)strrchr(yap, '/'), "\\firmware.yap");

	bool encrypt = false;
	if (argc >=3 && (argv[2][0] == 'E' || argv[2][0] == 'e'))
		encrypt = true;

	gen_bin(argv[1], yap, encrypt);

	return 0;
}
