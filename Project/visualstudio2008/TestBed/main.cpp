#include <stdio.h>
#include "resource.h"
#include <Windows.h>
#include <Commctrl.h>
#include <math.h>
#include <float.h>
#include <modules/win32/comm.h>


#pragma comment(lib, "Comctl32.lib")
/*
0、主控mcu连接自检

selftest
critial_error(uint2), voltage(float), flow(uint2)
1、罗盘自检
2、陀螺仪自检
3、加速计自检
4、气压计自检
5、主电压自检
6、光流mcu自检
7、光流cmos自检

accel_cal			// reset accel calibration
accel_cal_state		// get accel calibration
state(uint6)
8、加速计校准---6面静置

mag_cal				// start/reset mag calibration
mag_cal_state
9、罗盘校准---转2圈

reading
10、超声波读数---拿起和放下
11、光流读数---水平移动，看读数趋势和大小
12、加速计读数---分别在不同方向静置看总加速度
13、陀螺读数---3个方向旋转看读数
13、罗盘读数---不同方向看总磁场读数
14、气压读数---跟当天气压比对

*/

#define G_in_ms2 9.8065f			// gravity in m/s^2
#define PI 3.1415926
char version[200];
float acc_bias[3] = {0};
float mag_bias[3] = {1,1,1};
float acc_scale[3] = {0};
float mag_scale[3] = {1,1,1};
float g_force = 0;
float mag_length = 0;
int ppm_states[8][3];
float rc_setting[8][4];
float rc[8] = {0};

static const char * critical_error_desc[] = 
{
	"陀螺",
	"加速计",
	"罗盘",
	"气压计",
	"遥控接收机",
	"GPS",
	"error_MAX",
};


enum critical_error
{
	error_gyro = 1,
	error_accelerometer = 2,
	error_magnet = 4,
	error_baro = 8,
	error_RC = 16,
	error_GPS = 32,
	error_MAX,
} ;

HWND wnd;
Comm comm;

int read_rc_settings();

int disconnect()
{
	comm.disconnect();
	return -1;
}

static float limit(float v, float low, float high)
{
	if (v<low)
		return low;
	if (v>high)
		return high;
	return v;
}
static float ppm2rc(float ppm, float min_rc, float center_rc, float max_rc, bool revert)
{
	float v = (ppm-center_rc) / (ppm>center_rc ? (max_rc-center_rc) : (center_rc-min_rc));

	v = limit(v, -1, +1);

	if (revert)
		v = -v;

	return v;
}

int read_calibration_data()
{
	comm.read_float("abix", &acc_bias[0]);
	comm.read_float("abiy", &acc_bias[1]);
	comm.read_float("abiz", &acc_bias[2]);
	comm.read_float("ascx", &acc_scale[0]);
	comm.read_float("ascy", &acc_scale[1]);
	comm.read_float("ascz", &acc_scale[2]);
	comm.read_float("mbx", &mag_bias[0]);
	comm.read_float("mby", &mag_bias[1]);
	comm.read_float("mbz", &mag_bias[2]);
	comm.read_float("mgx", &mag_scale[0]);
	comm.read_float("mgy", &mag_scale[1]);
	comm.read_float("mgz", &mag_scale[2]);
	
	return 0;
}

int testbed_OnEvent(int code, void *extra_data)
{
	if (code == WM_DISCONNECT)
	{
		SetWindowTextA(GetDlgItem(wnd, IDC_SELFTEST), "插入飞控");
		SetWindowTextA(GetDlgItem(wnd, IDC_READING), "");
		SetWindowTextA(GetDlgItem(wnd, IDC_ACC), "");
		SetWindowTextA(GetDlgItem(wnd, IDC_MAG), "");

		memset(acc_bias, 0, sizeof(acc_bias));
		memset(acc_scale, 0, sizeof(acc_scale));
		memset(mag_bias, 0, sizeof(mag_bias));
		memset(mag_scale, 0, sizeof(mag_scale));
	}

	if (code == WM_CONNECT)
	{
		char output[1024] = {0};
		char cmd[] = "hello\n";
		if (comm.command(cmd, strlen(cmd), output, sizeof(output)) < 0)
		{
			disconnect();
			return -1;
		}

		read_rc_settings();
		
		strcpy(version, output+3);
	}

	return 0;
}

void read_selftest()
{
	// "selftest"
	char output[1024] = {0};
	char cmd[] = "selftest\n";
	if (comm.command(cmd, strlen(cmd), output, sizeof(output)) < 0)
	{
		disconnect();
		return;
	}

	int critical_errors = 0;
	int voltage;
	int flow_state = 0;
	if (sscanf(output, "%d,%d,%d", &critical_errors, &voltage, &flow_state) != 3)
	{
		disconnect();
		return;
	}

	char desc[1024] = {0};
	for(int i=0; (1<<i)<error_MAX; i++)
	{
		if ((1<<i) & critical_errors)
		{
			strcat(desc, critical_error_desc[i]);
			strcat(desc, "--错误\n");
		}
		else
		{
// 				strcat(desc, critical_error_desc[i]);
// 				strcat(desc, "--OK\n");
		}
	}

	if (flow_state == -1)
	{
		strcat(desc, "FLOW--MCU错误\n");
	}
	else if (flow_state == -2)
	{
		strcat(desc, "FLOW--CMOS错误\n");
	}
	else if (flow_state < 0)
	{
		sprintf(desc + strlen(desc), "FLOW--OK\n");
	}


	static int n = 0;
	if (n++ % 20 == 0)
		read_calibration_data();
	bool has_acc_calibration = false;
	bool has_mag_calibration = false;
	for(int i=0; i<3; i++)
	{
		if (acc_bias[i] != 0 || acc_scale[i] != 1)
			has_acc_calibration = true;
		if (mag_bias[i] != 0 || mag_scale[i] != 1)
			has_mag_calibration = true;
	}
	for(int i=0; i<3; i++)
	{
		if (_isnan(acc_bias[i]) || _isnan(acc_scale[i]))
			has_acc_calibration = false;
		if (_isnan(mag_bias[i]) || _isnan(mag_scale[i]))
			has_mag_calibration = false;
	}



	if (!has_acc_calibration)
		strcat(desc, "加速计---无校准数据\n");
	else if (abs(g_force - 1.0) > 0.03)
		strcat(desc, "总加速度---超出允许范围(1.0±0.03)\n");


	if (!has_mag_calibration)
		strcat(desc, "罗盘---无校准数据\n");

	else if (abs(mag_length - 500.0) > 100.0)
		strcat(desc, "总磁场---超出允许范围(500.0±20%)\n");



	strcat(desc, "------------------------\n");
	for(int i=0; (1<<i)<error_MAX; i++)
	{
		if ((1<<i) & critical_errors)
		{
// 			strcat(desc, critical_error_desc[i]);
// 			strcat(desc, "--错误\n");
		}
		else
		{
				strcat(desc, critical_error_desc[i]);
				strcat(desc, "--OK\n");
		}			
	}
	if (flow_state == 0)
	{
		strcat(desc, "FLOW--OK\n");
	}

	strcat(desc, "------------------------\n");

	char tmp[100];
	sprintf(tmp, "电压：%.3fV\n", voltage/1000.0f);
	strcat(desc, tmp);

	sprintf(tmp, "飞控版本：%s\n", version);
	strcat(desc, tmp);

	SetWindowTextA(GetDlgItem(wnd, IDC_SELFTEST), desc);
}

int read_rc_settings()
{
	for(int i=0; i<8; i++)
	for(int j=0;j<4; j++)
	{
		char tmp[200];
		sprintf(tmp, "rc%d%d", i,j);
		if (comm.read_float(tmp, &rc_setting[i][j]) < 0)
			return -1;
	}

	return 0;
}
int update_ppm()
{
	char cmd[] = "rcstates\n";
	char output[20480] = {0};
	char *p = output;

	int len = comm.command(cmd, strlen(cmd), output, sizeof(output));
	if (len < 0)
		return -1;

// 	printf("%s\n", output);

	// parse result
	p = strstr(p, "rc:");
	if (p)
	{
		p+=3;
		for(int i=0; i<8; i++)
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

	// update graph
	int table[8] = {IDC_PROGRESS1, IDC_PROGRESS2, IDC_PROGRESS3, IDC_PROGRESS4, IDC_PROGRESS6, IDC_PROGRESS5, IDC_PROGRESS7, IDC_PROGRESS8, };
	int table2[8] = {IDC_CH1, IDC_CH2, IDC_CH3, IDC_CH4, IDC_CH6, IDC_CH5, IDC_CH7, IDC_CH8, };

	for(int i=0; i<8; i++)
	{
		SendMessage(GetDlgItem(wnd, table[i]), PBM_SETRANGE32, 0, 1000);
		SendMessage(GetDlgItem(wnd, table[i]), PBM_SETPOS, int(rc[i] * 500 + 500), 0);

		char str[200];
		int p = i;
		if (i == 5)
			p = 4;
		else if (i == 4)
			p = 5;

		sprintf(str, "CH%d:%d", p+1, ppm_states[i][1]);
		SetDlgItemTextA(wnd, table2[i], str);
	}


	return 0;
}

void read_acc_cal()
{
	char output[1024] = {0};
	char cmd[] = "accel_cal_state\n";
	if (comm.command(cmd, strlen(cmd), output, sizeof(output)) < 0)
	{
		disconnect();
		return;
	}
	
	int acc_state;
	if (sscanf(output, "%d", &acc_state) != 1)
	{
		disconnect();
		return;
	}
	if (acc_state == -1)
	{
		SetWindowTextA(GetDlgItem(wnd, IDC_ACC), "未开始校准");

	}
	
	else if (acc_state != 0x3f)
	{
		char state_string[200] = "将以下的面朝下静置直到提示消失：\n\n";
		char *state_string6[] = {"后","前","左","右","上","下"};
		for(int i=0; i<6; i++)
		{
			if (!((1<<i) & acc_state))
			{
				strcat(state_string, state_string6[i]);
				strcat(state_string, ", ");
			}
		}

		SetWindowTextA(GetDlgItem(wnd, IDC_ACC), state_string);
	}
	else
	{
		SetWindowTextA(GetDlgItem(wnd, IDC_ACC), "OK");
	}
}

void read_mag_cal()
{
	char output[1024] = {0};
	char cmd[] = "mag_cal_state\n";
	if (comm.command(cmd, strlen(cmd), output, sizeof(output)) < 0)
	{
		disconnect();
		return;
	}

	int mag_state, last_mag_result;
	if (sscanf(output, "%d,%d", &mag_state, &last_mag_result) != 2)
	{
		disconnect();
		return;
	}

	char desc[1024] = {0};
	if (mag_state == 0)
	{
		strcpy(desc, "未开始校准");
	}
	else if (mag_state == 1)
	{
		strcpy(desc, "飞控水平，水平旋转飞控");
	}
	else if (mag_state == 2)
	{
		strcpy(desc, "飞控头部朝下，水平旋转飞控");
	}

	if (last_mag_result != 0xff)
	{
		strcat(desc, "\n\n");
		
		if (last_mag_result == 0)
			strcat(desc, "校准成功");
		else if (last_mag_result == -1)
			strcat(desc, "校准失败，未能收集足够的罗盘数据（罗盘或陀螺仪可能已经损坏）");
		else if (last_mag_result == 1)
			strcat(desc, "校准失败，磁干扰，请远离磁性物体");
		else if (last_mag_result == 2)
			strcat(desc, "校准失败，数据超过规格，可能是次品罗盘");
		else
			strcat(desc, "校准失败， 未知错误");

	}

	SetWindowTextA(GetDlgItem(wnd, IDC_MAG), desc);
}


void read_reading()
{
	char output[1024] = {0};
	char cmd[] = "reading\n";
	if (comm.command(cmd, strlen(cmd), output, sizeof(output)) < 0)
	{
		disconnect();
		return;
	}

	int sonar, flow[2];
	int qual;
	int accel[3];
	int gyro[3];
	int mag[3];
	int baro, temperature;

	int count = sscanf(output, 
		"%d,%d,%d,%d,"	// sonar, flow
		"%d,%d,%d,"		// accel 
		"%d,%d,%d,"		// gyro
		"%d,%d,%d,"		// mag
		"%d, %d\n",		// baro, baro temperature

		&sonar, &flow[0], &flow[1], &qual,
		&accel[0], &accel[1], &accel[2],
		&gyro[0], &gyro[1], &gyro[2],
		&mag[0], &mag[1], &mag[2],
		&baro, &temperature
		);

	if (count != 15)
	{
		disconnect();
		return;
	}

	g_force = sqrt((double)accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2])/G_in_ms2/1000;
	mag_length = sqrt((double)mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
	float angular_rate = sqrt((double)gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]) / 100.0f;
	float roll = atan2((double)accel[1], accel[2]);
	float pitch = atan2((double)-accel[0], accel[2]);
	float magX = -mag[0] * cos(pitch) + (-mag[1]) * sin(roll) * sin(pitch) + (mag[2]) * cos(roll) * sin(pitch);
	float magY = -mag[1] * cos(roll) - (mag[2]) * sin(roll);

	float initialHdg = atan2(-magY, magX);

	char state_string[1024];
	sprintf(state_string, 
		"超声波：%.2f m\n"
		"FLOW : %d, %d\n"
		"FLOW qual: %d\n"
		"加速计：%.2f, %.2f, %.2f m/s2\n"
		"陀螺：%02.2f, %02.2f, %02.2f  °/s\n"
		"罗盘：%02d, %02d, %02d\n"
		"气压：%d Pa\n"
		"温度：%.2f °C\n"
		"总加速度:%.3f g\n"
		"总磁场: %.3f mgauss\n"
		"总旋转速度: %.1f °/s\n"
		"角度:%.2f, %.2f, %.2f"
		
		,

		sonar/1000.0f,
		flow[0], flow[1],
		qual,
		accel[0]/1000.0f, accel[1]/1000.0f, accel[2]/1000.0f,
		gyro[0]/100.0f, gyro[1]/100.0f, gyro[2]/100.0f,
		mag[0], mag[1], mag[2],
		baro,
		temperature/100.0f,
		g_force,
		mag_length,
		angular_rate,
		roll* 180 / PI, pitch * 180 / PI, initialHdg * 180 / PI
		
		);
	SetWindowTextA(GetDlgItem(wnd, IDC_READING), state_string);

}

DWORD CALLBACK worker_thread(LPVOID p)
{
	while(true)
	{
		read_selftest();
		read_acc_cal();
		read_reading();
		read_mag_cal();
		update_ppm();

		Sleep(10);
	}

	return 0;
}


INT_PTR CALLBACK WndProcTestbed(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{

	switch (message)
	{
	case WM_INITDIALOG:
		wnd = hWnd;
		comm.add_callback(testbed_OnEvent);
		CreateThread(NULL, NULL, worker_thread, NULL, NULL, NULL);
		break;
	case WM_CLOSE:
		EndDialog(hWnd, 0);
		break;
	case WM_COMMAND:
		{
			int id = LOWORD(wParam);

			if (id == IDC_START_ACC)
			{
				char cmd[1024] = "accel_cal\n";
				if (comm.command(cmd, strlen(cmd), cmd, sizeof(cmd)) < 0 || strstr(cmd, "ok") == NULL)
				{
					disconnect();
				}
			}
			else if (id == IDC_MAG_START)
			{
				char cmd[1024] = "mag_cal\n";
				if (comm.command(cmd, strlen(cmd), cmd, sizeof(cmd)) < 0 || strstr(cmd, "ok") == NULL)
				{
					disconnect();
				}
			}
		}
		break;

	default:
		return FALSE;
	}
	return TRUE;
}

int main()
{
	InitCommonControls();
	DialogBox(NULL, MAKEINTRESOURCE(IDD_DIALOG1), NULL, WndProcTestbed);

	return 0;
}
