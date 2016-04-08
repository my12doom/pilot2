#pragma once

#include <Windows.h>
#include <vector>

#define WM_CONNECT (WM_USER+1)
#define WM_DISCONNECT (WM_USER+2)

class ICommEvent
{
public:
	virtual int OnEvent(int code, void *extra_data);
};

typedef int (*CommEventProc)(int code, void *extra_data);

class Comm
{
public:
	Comm();
	~Comm();

	int states();
	int command(const char *input, int input_count, char*output, int output_buffer_size);
	int add_callback(ICommEvent * cb);
	int add_callback(CommEventProc cb);
	int add_callback(HWND cb);

	int read_float(const char* id, float *out);
	int write_float(const char* id, float newdata);
	int enum_float(int pos, char *id, float *out);			// return 1 on EOF, 0 on success, -x on error
	int disconnect();	// also on_disconnect()


protected:
	static DWORD CALLBACK find_device_thread(LPVOID p);
	static DWORD CALLBACK heart_beat_thread(LPVOID p);
	int on_connect();
	int find_device();
	int hand_shake();
	CRITICAL_SECTION cs,cs_cb;
	HANDLE port;
	int m_states;

	int last_success_port;

	std::vector<ICommEvent*> callbacks1;
	std::vector<CommEventProc> callbacks2;
	std::vector<HWND> callbacks3;
};