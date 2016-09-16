#pragma once

#include <Protocol/Common.h>

#ifdef __cplusplus
extern "C" {
#endif
	

int parse_command_line(const char *line, char *out);
extern WEAK const char version_name[];
extern WEAK const char bsp_name[];

#ifdef __cplusplus
}
#endif