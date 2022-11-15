#include <Windows.h>
#include "fifo.h"
#include "autolock.h"

int show_dso_window(HINSTANCE instance, HWND parent);
int exit_dso();

extern MagicRingBuffer DSO_ringbuf;
extern _critical_section cs_DSO_ringbuf;
