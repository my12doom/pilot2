#include "param.h"
#include <stdlib.h>
#include <string.h>
#include "space.h"

#define PARAM_ENDL "\0\0\0\0"

int all_param_count = -1;
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
	if (all_param_count<0)
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
	space_init();
	all_param_count = 0;
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