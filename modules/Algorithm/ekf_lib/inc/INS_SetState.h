/*
 * File: INS_SetState.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 26-Nov-2015 12:02:43
 */

#ifndef __INS_SETSTATE_H__
#define __INS_SETSTATE_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "ekf_13state_types.h"


#ifdef __cplusplus
extern "C" {
#endif


/* Function Declarations */
extern void INS_SetState(float p_x, float p_y, float p_z, float v_x, float v_y,
  float v_z, float q0, float q1, float q2, float q3, float gyro_x_bias, float
  gyro_y_bias, float gyro_z_bias, float X[13]);



#ifdef __cplusplus
}
#endif


#endif

/*
 * File trailer for INS_SetState.h
 *
 * [EOF]
 */
