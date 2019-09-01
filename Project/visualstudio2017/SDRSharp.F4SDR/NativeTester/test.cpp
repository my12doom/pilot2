#include <stdio.h>
#include <Windows.h>


typedef int(__stdcall *CSCallback)(const void* ptr, int length_byte);
typedef int(* Func)();
typedef int(* Func2)(CSCallback cb);

int __stdcall cb(const void* ptr, int length_byte)
{
	return 0;
}

int main()
{
	HMODULE dll = LoadLibrary("F4SDRNative.dll");

	Func init = (Func)GetProcAddress(dll, "f4sdr_dll_init");
	Func open = (Func)GetProcAddress(dll, "f4sdr_open");
	Func close = (Func)GetProcAddress(dll, "f4sdr_close");
	Func2 setcb = (Func2)GetProcAddress(dll, "f4sdr_setcb");

	init();
	open();
	setcb(cb);

	Sleep(2000);
	
	printf("stop\n");
	close();

	Sleep(1000);

	printf("start\n");
	open();

	Sleep(50000);
	return 0;
}
