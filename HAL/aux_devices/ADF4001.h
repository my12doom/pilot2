#pragma once

typedef struct
{
    unsigned ADDR:2;
    unsigned R:14;
    unsigned anti_backlash:2;
    unsigned test_mode:2;
    unsigned lock_detect_precision:1;
} ADF4001_REG0;

typedef struct
{
    unsigned ADDR:2;
    unsigned reserved:6;
    unsigned N:13;
    unsigned cp_gain:1;
} ADF4001_REG1;

typedef struct
{
    unsigned ADDR:2;
    unsigned counter_reset:1;
    unsigned power_down1:1;
    unsigned mux_out:3;
    unsigned polarity:1;
    unsigned cp_3state:1;
    unsigned fast_lock_en:1;
    unsigned fast_lock_mode;
    unsigned timer_control:4;
    unsigned cp_current1:3;
    unsigned cp_current2:3;
    unsigned power_down2:1;
} ADF4001_REG2;
