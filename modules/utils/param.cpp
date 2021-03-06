#include "param.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "space.h"

#define PARAM_ENDL "\0\0\0\0"
#define PARAM_KEY 0x85a3a385

int all_param_count = -1;
uint32_t param_init_key = 0;
void init_all_param_count()
{
	static bool c = true;
	if (c)
	{
		c = false;
		all_param_count = -1;
	}
}
struct
{
	char fourcc[4];
	float v;
}all_params[MAX_PARAM_COUNT];

static int fourcclen(const char *p)
{
	for(int i=0; i<4; i++)
		if (p[i] == NULL)
			return i;
	return 4;
}

param::param()
{

}

void param::init(const char *fourcc, float default_value)
{
	// TODO : locks
	init_all_param_count();
	init_all();

	for(int i=0; i<all_param_count; i++)
	{
		if (strncmp(fourcc, all_params[i].fourcc, 4) == 0)
		{
			pv = &all_params[i].v;
			pos = i;
			
			return;
		}
	}

	pos = all_param_count;
	pv = &all_params[all_param_count].v;
	strncpy(all_params[all_param_count].fourcc, fourcc, 4);
	all_params[all_param_count].v = default_value;
	space_read(fourcc, fourcclen(fourcc), &all_params[all_param_count].v, 4, NULL);
	all_param_count++;

	
	save();
}

param::param(const char *fourcc, float default_value)
{
	init(fourcc, default_value);
}

param::~param()
{

}

void param::save()						// save to eeprom
{
	int res;
	float v;
	res = space_read(all_params[pos].fourcc, fourcclen(all_params[pos].fourcc), &v, 4, NULL);

	if (v != all_params[pos].v || res < 0)
		res = space_write(all_params[pos].fourcc, fourcclen(all_params[pos].fourcc), &all_params[pos].v, 4, NULL);
}
void param::init_all()
{
	if (param_init_key == PARAM_KEY)
		return;

	space_init();
	all_param_count = 0;

	int handle = -1;
	do
	{
		char key[16] = {0};
		char data[16] = {0};
		int keysize = 0;
		int datasize = 0;

		handle = space_enum(key, &keysize, data, &keysize, handle);

		// check duplicated value
		bool duplicated = false;
		for(int i=0; i<all_param_count; i++)
		{
			if (strncmp(key, all_params[i].fourcc, 4) == 0)
			{
				duplicated = true;
				break;
			}
		}

		if (duplicated)
			continue;

		pos = all_param_count;
		pv = &all_params[all_param_count].v;
		memset(all_params[all_param_count].fourcc, 0, 4);
		strncpy(all_params[all_param_count].fourcc, key, keysize);
		all_params[all_param_count].v = *(float*)data;
		all_param_count++;
	}
	while(handle > 0);

	param_init_key = PARAM_KEY;
}

void param::save_all()
{
	for(int i=0; i<all_param_count; i++)
		param(all_params[i].fourcc, all_params[i].v).save();
}

float *param::find_param(const char *fourcc)
{
	for(int i=0; i<all_param_count; i++)
		if (strncmp(fourcc, all_params[i].fourcc, 4) == 0)
			return &all_params[i].v;
	return NULL;
}
const char *param::enum_params(int pos)
{
	if (pos < 0 || pos >= all_param_count)
		return NULL;
	return all_params[pos].fourcc;
}

const char *param::fourcc()
{
	return all_params[pos].fourcc;
}

extern "C"
{
float * find_param(const char *fourcc)
{
	return param::find_param(fourcc);
}
int save_param(const char *fourcc)
{
	if (!find_param(fourcc))
		return -1;
	
	param p(fourcc, 0);
	p.save();
	return 0;
}
float * create_param(const char *fourcc, float default_value)
{
	return &param(fourcc, default_value);
}
};