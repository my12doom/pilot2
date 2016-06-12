#include <windows.h>
#include <win32/comm.h>

int ppm_states[8][3];
int ppm_center[8] = {0};
float rc_setting[8][4];
float rc[8] = {0};
extern Comm comm;
bool pending_read_setting = false;

int read_rc_settings()
{
	pending_read_setting = true;
	return 0;
}

int read_rc_settings_core()
{
	for(int i=0; i<8; i++)
		for(int j=0;j<4; j++)
		{
			char tmp[200];
			sprintf(tmp, "rc%d%d", i,j);
			if (comm.read_float(tmp, &rc_setting[i][j]) < 0)
				return -1;
		}

	pending_read_setting = false;
	return 0;
}

static float ppm2rc(float ppm, float min_rc, float center_rc, float max_rc, bool revert)
{
	float v = (ppm-center_rc) / (ppm>center_rc ? (max_rc-center_rc) : (center_rc-min_rc));

	//v = limit(v, -1, +1);
	if (v<-1)
		v = -1;
	if (v>1)
		v = 1;

	if (revert)
		v = -v;

	return v;
}

int update_ppm()
{
	char cmd[] = "rcstates\n";
	char output[20480] = {0};
	char *p = output;

	int len = comm.command(cmd, strlen(cmd), output, sizeof(output));
	if (len < 0)
		return -1;

	if (pending_read_setting)
		read_rc_settings_core();

	// parse result
	p = strstr(p, "rc:");
	if (p)
	{
		p+=3;
		for(int i=0; i<6; i++)
		{
			if (sscanf(p, "%d,%d,%d,", &ppm_states[i][0], &ppm_states[i][1], &ppm_states[i][2]) != 3)
				return -1;

			// skip comma
			p = strstr(p, ",")+1;
			p = strstr(p, ",")+1;
			p = strstr(p, ",")+1;
		}
	}

	// update scaled rc position
	for(int i=0; i<8; i++)
		rc[i] = ppm2rc(ppm_states[i][1], rc_setting[i][0], rc_setting[i][1], rc_setting[i][2], rc_setting[i][3] > 0);

	rc[2] = (rc[2]+1)/2;

	return 0;
}
