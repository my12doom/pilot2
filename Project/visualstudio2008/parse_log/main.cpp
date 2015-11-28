#include <stdio.h>
#include <conio.h>
#include <stdint.h>
#include <protocol/RFData.h>

int main(int argc, char* argv[])
{
	if (argc < 2)		
	{
		printf("usage: parse_log.exe 0001.dat\npress any key to exit...");
		getch();
		return -2;
	}

	char out_name[300];
	sprintf(out_name, "%s.log", argv[1]);

	FILE *in = fopen(argv[1], "rb");
	FILE *out = fopen(out_name, "wb");

	if (!in)
	{
		printf("failed opening %s\n", argv[1]);
		return -1;
	}

	if (!out)
	{
		printf("failed opening %s\n", out_name);
		return -1;
	}

	fprintf(out, "ArduCopter V3.1.5 (3c57e771)\r\n");
	fprintf(out, "Free RAM: 990\r\n");
	fprintf(out, "MoMo\r\n");
	fprintf(out, "FMT, 9, 23, CURR, IhIh, TimeMS,ThrOut,Volt,Curr\r\n");


	int64_t time = 0;
	ppm_data ppm;
	while (fread(&time, 1, 8, in) == 8)
	{
		uint8_t tag = time >> 56;
		uint16_t tag_ex;
		time = time & ~((uint64_t)0xff << 56);
		char data[65536];
		int size = 24;

		if (tag == TAG_EXTENDED_DATA)		// extended variable length data packets
		{
			if (fread(&tag_ex, 1, 2, in) != 2)
				break;
			if (fread(&size, 1, 2, in) != 2)
				break;
			if (fread(data, 1, size, in) != size)
				break;
		}
		else
		{
			size = 24;
			if (fread(data, 1, 24, in) != 24)
				break;
		}

		// handle packet data here..
		if (tag == TAG_PPM_DATA)
		{
			memcpy(&ppm, data, 24);
		}
		else if (tag == TAG_SENSOR_DATA)
		{
			sensor_data &sensor = *(sensor_data*)data;
			int throttle = (ppm.out[0] + ppm.out[1] + ppm.out[2] + ppm.out[3])/4;
			static int power_id = 0;
			if (power_id++ % 10 == 0)
				fprintf(out, "CURR, %d, %d, %f, %f, %d, %d\r\n", int(time/1000), throttle, sensor.voltage/1000.0f, sensor.current/1000.0f, 0, 0);
		}
	}

	fclose(in);
	fclose(out);

	return 0;
}