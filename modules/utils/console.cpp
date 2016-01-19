#include "console.h"
#include "vector.h"
#include <stdlib.h>
#include <string.h>
#include "param.h"
#include "space.h"
#include <math.h>
#include <Algorithm/pos_estimator.h>
#include <Algorithm/mag_calibration.h>
#include <HAL/resources.h>
#include <Protocol/crc32.h>
#include <modules/main/pilot.h>

using namespace HAL;

const char version_name[] = __DATE__ " " __TIME__;
const char bsp_name[] = " unknown";

#define SIGNATURE_ADDRESS 0x0800E800

static int min(int a, int b)
{
	return a>b?b:a;
}

// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

 // Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint)
{
	if (n<0)
	{
		*res = '-';
		res++;
		n = -n;
	}
	
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10.0f, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

// constants and parameters
const uint32_t ApplicationAddress = 0x8004000;
static param rom_size("size", 0);
static param rom_crc("CRC", 0);

bool check_rom_crc()
{
	// the CRC and size are stored "as" float, but are indeed binary uint32_t.
	float sizef = rom_size;
	float crcf = rom_crc;
	uint32_t size = *(uint32_t*)&sizef;
	uint32_t crc = *(uint32_t*)&crcf;	
	
	if (size <= 1024 || size > 1024*768)
		return false;
	
	return crc32(0, (uint8_t*)ApplicationAddress, size) == crc;
}

extern "C" int parse_command_line(const char *line, char *out)
{

	float roll,  pitch, yaw;
	//float p,i,d;
	int log_level;
	int i,j,k;

	if (line[0] == NULL)
		return 0;

	if (*line == '?')
	{
		float *v = param::find_param(line+1);
		if (v)
		{
			if (isnan(*v))
				strcpy(out, "NAN");
			else
				ftoa(*v, out, 6);
			strcat(out, "\n");
		}
		else
			strcpy(out, "null\n");
		return strlen(out);
	}
	if (*line == '!')
	{
		const char *fourcc = param::enum_params(atoi(line+1));
		if (fourcc)
		{
			char strout[5] = {0};
			memcpy(strout, fourcc, min(strlen(fourcc),4));

			param v(strout, 0);
			
			out[0] = strout[0];
			out[1] = strout[1];
			out[2] = strout[2];
			out[3] = strout[3];
			out[4] = 0;
			strcat(out, "=");
			
			if (isnan(float(v)))
				strcpy(out+strlen(out), "NAN");
			else
			{
				ftoa(v, out+strlen(out), 6);
			}
			strcat(out, "\n");
		}
		else
			strcpy(out, "null\n");
		return strlen(out);
	}
	else if (char *p = (char*)strchr(line, '='))
	{
		*p = NULL;
		p++;

		float *pv = param::find_param(line);
		if (pv)
		{
			float v = atof(p);
			if (strstr(p, "NAN"))
				v = NAN;
			*pv = v;
			param(line, 0).save();
			strcpy(out, "ok\n");
			return 3;
		}
		else
		{
			strcpy(out, "fail\n");
			return 5;
		}
	}
	else if (strstr(line, "rcstates") == line)
	{
		int count = 0;
		count += sprintf(out, "rc:");
		
		IRCIN *rcin = manager.get_RCIN();
		const int channel_count = 8;
		
		int16_t static_min[channel_count] = {0};
		int16_t static_max[channel_count] = {0};
		int16_t current[channel_count] = {0};
		
		rcin->get_channel_data(current, 0, channel_count);
		rcin->get_statistics_data(static_min, static_max, 0, channel_count);		
		
		for(i=0; i<channel_count; i++)
		{
			volatile float p0 = static_min[i];
			volatile float p1 = current[i];
			volatile float p2 = static_max[i];
			count += sprintf(out+count, "%d,%d,%d,", (int)p0, (int)p1, (int)p2);
			if (count > 256)
				return count;
		}

		out[count++] = '\n';

		return count;
	}
	else if (strstr(line, "rcreset") == line)
	{
		manager.get_RCIN()->reset_statistics();
		strcpy(out, "ok\n");
		return 3;
	}
	else if (strstr(line, "hello") == line)
	{
		sprintf(out, "yap(%s)(bsp %s)\n", version_name, bsp_name);
		return strlen(out);
	}
	else if (strstr(line, "imureset") == line)
	{
		yap.avg_count = 0;
		for(i=0; i<2; i++)
		{
			for(j=0; j<3; j++)
			{
				yap.imu_statics[i][0].array[j] = 99999;
				yap.imu_statics[i][2].array[j] = -99999;
				yap.imu_statics[i][3].array[j] = 0;
			}
		}
		strcpy(out, "ok\n");
		return 3;
	}
	else if (strstr(line, "usb") == line)
	{
		if (strstr(line, "usb,") == line)
		{
			yap.usb_data_publish = atoi(line+4);
		}
		else
		{
			yap.usb_data_publish = 0xff;
		}
	}
	else if (strstr(line, "gps") == line)
	{
		devices::IGPS *gps = manager.get_GPS(0);
		if (!gps)
		{
			strcpy(out, "no gps\n");
			return strlen(out);
		}
		
		devices::gps_data data;
		gps->read(&data);
		int hdop = data.DOP[1] / 100;
		int hdop_frac = data.DOP[1] % 100;
		int db_max = -999, db_min=999;
		position_meter pos = yap.estimator.get_estimation_meter();
		position_meter pos_raw = yap.estimator.get_raw_meter();
		sprintf(out, "%.2f/%.2fm, raw%.2f/%.2fm, %.2f/%.2fm/s, %d/%d sat, hdop%d.%02d, %d-%ddb(", pos.latitude, pos.longtitude, pos_raw.latitude, pos_raw.longtitude,
				pos.vlatitude, pos.vlongtitude, data.satelite_in_use, data.satelite_in_view, hdop, hdop_frac, db_min, db_max);
		/*
		char tmp[200] = {0};
		for(int i=0; i<NMEA_MAXSAT; i++)
		{
			if (data.satinfo.sat[i].sig > db_max)
				db_max = data.satinfo.sat[i].sig;
			if (data.satinfo.sat[i].sig < db_min)
				db_min = data.satinfo.sat[i].sig;
			sprintf(tmp+strlen(tmp), "%d,", data.satinfo.sat[i].sig);
		}
		strcat(out, tmp);
		*/
		strcat(out, ")\n");
		
		return strlen(out);
	}
	else if (strstr(line, "imustates") == line)
	{
// 		printf("test:");
// 		printf("%.3f,%d,", mpu6050_temperature, avg_count);

		int count = 0;
		count += sprintf(out, "imu:");
		for(i=0; i<2; i++)
		{
			for(j=0; j<4; j++)
			{
				for(k=0; k<3; k++)
				{
					volatile int t = yap.imu_statics[i][j].array[k] * 10000.0f / (j==3?yap.avg_count:1);
					count += sprintf(out+count, "%d.%04d,",  t/10000,  abs(t%10000));
					//if (count > 256)
					//	return count;
				}
			}
		}

		count += sprintf(out+count, "%d.%d,%d,", (int)yap.mpu6050_temperature, ((int)(yap.mpu6050_temperature*1000))%1000, yap.avg_count);
		out[count++] = '\n';

		return count;
	}
	/*
	// TODO: id & signatures
	else if (strstr(line, "id") == line)
	{
		strcpy(out, "id:");
		memcpy(out+3, (void*)(0x1ffff7e8), 12);
		memcpy(out+15, (void*)SIGNATURE_ADDRESS, 128);
		out[15+128] = '\n';

		return 144;
	}
	else if (strstr(line, "sig1:") == line || strstr(line, "sig2:") == line || strstr(line, "sig3:") == line)
	{
		if (line[3] == '1')
			FLASH_ErasePage(SIGNATURE_ADDRESS);

		int p = line[3] - '1';
		for(int i=0; i<48; i+=4)
			FLASH_ProgramWord(SIGNATURE_ADDRESS+i+48*p, *(unsigned int*)(line+5+i));

		strcpy(out, "ok\n");
		return 3;
	}
	*/
	else if (strstr(line, "resetmc") == line)
	{
		space_init(true);
		reset_system();
	}
	
	else if (strstr(line, "reset") == line)
	{
		reset_system();
	}
	else if (strstr(line, "romcrc,") == line)
	{
		uint32_t size = 0;
		uint32_t crc = 0;
		
		if (sscanf(line+7, "%d,%x", &size, &crc) == 2)
		{
			float sizef = *(float*)&size;
			float crcf = *(float*)&crc;
			
			rom_size = sizef;
			rom_crc = crcf;
			
			FLASH_Unlock();
			rom_size.save();
			rom_crc.save();
		}
		
		if (check_rom_crc())
			strcpy(out, "CRC OK\n");
		else
			strcpy(out, "CRC FAILED\n");		
		
		return strlen(out);
	}
	
	else if (strstr(line, "clearcrc") == line)
	{
		rom_size = 0;
		rom_crc = 0;
		rom_crc.save();
		rom_size.save();
	}

	else if (strstr(line, "bootloader") == line)
	{
		RCC->APB1RSTR &= ~RCC_APB1RSTR_PWRRST; 		// clear RCC_APB1RSTR_PWRRST
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
		PWR_BackupAccessCmd(ENABLE);
		RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div4);
		RCC_RTCCLKCmd(ENABLE);

		void * bkp = (void*)0x40024000;
		memcpy(bkp, "hello", 6);

		reset_system();
	}

	else if (strstr(line, "accel_cal_state") == line)
	{
		if (!yap.acc_cal_requested)
		{
			sprintf(out, "-1\n");
		}
		else
		{
			int acc_state = 0;
			for(int i=0; i<6; i++)
			{
				if (yap.acc_avg_count[i] > 400)
					acc_state |= (1<<i);
			}

			sprintf(out, "%d\n", acc_state);
		}
		
		return strlen(out);
	}
	else if (strstr(line, "accel_cal") == line)
	{
		yap.reset_accel_cal();

		strcpy(out, "ok\n");

		return strlen(out);
	}
	else if (strstr(line, "mag_cal_state") == line)
	{		
		mag_calibration_stage stage = yap.mag_calibrator.get_stage();		// 	stage_horizontal = 0,	stage_vertical = 1,

		// returns: state, last_result, 
		//	state: 0 = not running, 1 = horizontal, 2 = vertical.
		//	result: see mag_calibration.h
		sprintf(out, "%d,%d\n", yap.mag_calibration_state ? (stage+1) : 0, yap.last_mag_calibration_result);
		
		return strlen(out);
	}
	else if (strstr(line, "mag_cal") == line)
	{
		yap.reset_mag_cal();

		strcpy(out, "ok\n");

		return strlen(out);
	}
	else if (strstr(line, "reading") == line)
	{		
		sprintf(out, 
			"%d,%d,%d,%d,"	// sonar, flow, flow quality
			"%d,%d,%d,"		// accel 
			"%d,%d,%d,"		// gyro
			"%d,%d,%d,"		// mag
			"%d, %d\n",		// baro, baro temperature

			yap.frame.ground_distance, yap.frame.pixel_flow_x_sum, yap.frame.pixel_flow_y_sum, yap.frame.qual,								// sonar, flow, flow quality
			int(yap.accel.array[0]*1000), int(yap.accel.array[1]*1000), int(yap.accel.array[2]*1000),									// accel 
			int(yap.gyro_reading.array[0]*18000/PI), int(yap.gyro_reading.array[1]*18000/PI), int(yap.gyro_reading.array[2]*18000/PI),	// gyro
			int(yap.mag.array[0]), int(yap.mag.array[1]), int(yap.mag.array[2]),												// mag
			int(yap.a_raw_pressure), int(yap.a_raw_temperature*100.0f)																		// baro
			);

		return strlen(out);
	}
	else if (strstr(line, "selftest") == line)
	{
		int flow_count = manager.get_flow_count();
		int cmos_version_ok = (yap.frame.cmos_version == 0x1324 || yap.frame.cmos_version == 0x76) && yap.frame.frame_count;
		
		sprintf(out, "%d,%d,%d\n", yap.critical_errors, int(yap.voltage*1000), flow_count <= 0 ? -1 : (cmos_version_ok ? 0 : -2));
		
		return strlen(out);
	}
	
	return 0;
}
