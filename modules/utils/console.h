#pragma once

#ifdef __cplusplus
extern "C" {
#endif

int parse_command_line(const char *line, char *out);
extern __attribute__((weak)) const char version_name[];
extern __attribute__((weak)) const char bsp_name[];

#ifdef __cplusplus
}
#endif