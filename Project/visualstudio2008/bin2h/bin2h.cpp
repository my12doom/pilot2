#include <stdio.h>
#include <Windows.h>
#include <assert.h>


int main(int argc, char *argv[])
{
	if (argc <= 3)
		return -1;

	FILE * f = fopen(argv[1], "rb");

	FILE * h = fopen(argv[3], "wb");

	if (!f || !h)
		return -2;

	fseek(f, 0, SEEK_END);
	int size = ftell(f);
	unsigned char *data = new unsigned char[size];
	fseek(f, 0, SEEK_SET);
	fread(data, 1, size, f);
	fclose(f);

	fprintf(h, "#pragma once\r\n");
	fprintf(h, "const uint8_t %s[%d] = {", argv[2], size);

	for(int i=0; i<size; i++)
		fprintf(h, "0x%02X, ", data[i]);

	fprintf(h, "};\r\n");

	fclose(h);

	return 0;
}