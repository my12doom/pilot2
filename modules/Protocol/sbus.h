#pragma once

#include <stdint.h>

typedef struct {
    uint32_t start : 8;
    uint32_t chan1 : 11;
    uint32_t chan2 : 11;
    uint32_t chan3 : 11;
    uint32_t chan4 : 11;
    uint32_t chan5 : 11;
    uint32_t chan6 : 11;
    uint32_t chan7 : 11;
    uint32_t chan8 : 11;
    uint32_t chan9 : 11;
    uint32_t chan10 : 11;
    uint32_t chan11 : 11;
    uint32_t chan12 : 11;
    uint32_t chan13 : 11;
    uint32_t chan14 : 11;
    uint32_t chan15 : 11;
    uint32_t chan16 : 11;
} __attribute__ ((__packed__)) sbus_dat;

typedef union
{
    uint8_t  raw[25];
    sbus_dat dat;
} sbus_u;
