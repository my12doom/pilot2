/*
 * File: init_quaternion_by_euler.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 26-Nov-2015 12:02:43
 */

#ifndef __INIT_QUATERNION_BY_EULER_H__
#define __INIT_QUATERNION_BY_EULER_H__

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
extern void init_quaternion_by_euler(float roll, float pitch, float yaw, float
  *q0, float *q1, float *q2, float *q3);

#ifdef __cplusplus
}
#endif

#endif

/*
 * File trailer for init_quaternion_by_euler.h
 *
 * [EOF]
 */
