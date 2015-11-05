#pragma once

// return 0 if flashed.
// return 1 if same data found.
// reutrn -1 if no valid bootloader file found.
int check_and_flash_bootloader(const char *filename);