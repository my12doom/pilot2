#pragma once
#include <stdio.h>
#define UAVDEBUG
extern "C"
{

#ifdef UAVDEBUG
#define LOG2(format,...) fprintf(stdout,format,##__VA_ARGS__)
#else
#define LOG2(format,...)
#endif

}

