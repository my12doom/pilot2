#include <stdio.h>
#include <string.h>
#include <stdint.h>

void print_hex(FILE *f, const void *data, int count)
{
	const uint8_t *p = (const uint8_t*)data;
	for(int i=0; i<count; i++)
		fprintf(f, "%02X", p[i]);
}

uint8_t intel_crc(const void *data, int count)
{
	const uint8_t *p = (const uint8_t*)data;
	uint8_t sum = 0;
	for(int i=0; i<count; i++)
		sum += p[i];

	return 0x100-sum;
}

int min(int a, int b)
{
	return a>b?b:a;
}

int write_hex(const void *data, uint32_t base_address, int size, FILE *f)
{
	// 16bytes a line
	uint16_t last_address_hi = 0;
	const uint8_t *p = (const uint8_t*)data;

	for(int i=0; i<size; i+=16)
	{
		uint32_t address = base_address + i;
		uint16_t address_hi = address>>16;
		uint16_t address_lo = address & 0xffff;


		// address hi line
		char tmp[100];
		if (address_hi != last_address_hi)
		{
			last_address_hi = address_hi;

			tmp[0] = 2;
			tmp[1] = 0;
			tmp[2] = 0;
			tmp[3] = 0x4;
			tmp[4] = address_hi >> 8;
			tmp[5] = address_hi & 0xff;
			tmp[6] = intel_crc(tmp, 6);

			fputc(':', f);
			print_hex(f, tmp, 7);
			fputc('\n', f);
		}

		// data 
		int line_size = min(size-i, 16);
		tmp[0] = line_size;
		tmp[1] = address_lo >> 8;
		tmp[2] = address_lo & 0xff;
		tmp[3] = 0x0;
		memcpy(tmp+4, p+i, line_size);
		tmp[4+line_size] = intel_crc(tmp, 4+line_size);
		fputc(':', f);
		print_hex(f, tmp, 5+line_size);
		fputc('\n', f);

	}

	return 0;
}

int filesize(FILE *f)
{
	int p = ftell(f);
	fseek(f, 0, SEEK_END);
	int size = ftell(f);
	fseek(f, p, SEEK_SET);

	return size;
}

int main(int argc, char *argv[])
{
	int count = 0;
	uint32_t address[100];
	char files[100][500];
	FILE *pfiles[100];
	FILE *outfile = NULL;

	for(int i=0; i<argc; i++)
	{
		if (sscanf(argv[i], "%x:%s", &address[count], files[count]) == 2)
		{
			pfiles[count] = fopen(files[count], "rb");
			if (!pfiles[count])
			{
				printf("failed opening %s\n", files[count]);
				return -1;
			}

			printf("%s -->> 0x%08x\n", files[count], address[count]);
			count++;
		}

		else if (strstr(argv[i], ".hex") || strstr(argv[i], ".HEX"))
		{
			outfile = fopen(argv[i], "wb");
			if (!outfile)
			{
				printf("failed opening %s\n", files[i]);
				return -1;
			}
		}
	}

	if (!outfile)
	{
		printf("no output file\n");
		return -3;
	}

	for(int i=0; i<count; i++)
	{
		int size = filesize(pfiles[i]);
		char *data = new char[size];
		fread(data, 1, size, pfiles[i]);

		write_hex(data, address[i], size, outfile);
		fclose(pfiles[i]);
	}

	// end string
	fprintf(outfile, ":00000001FF\n");
	fclose(outfile);

	return 0;
}
