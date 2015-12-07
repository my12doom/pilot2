/*
 * File: caculate_delta.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 07-Dec-2015 19:18:57
 */

#ifndef __CACULATE_DELTA_H__
#define __CACULATE_DELTA_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "caculate_delta_types.h"

/* Function Declarations */
extern void caculate_delta(const float q_target[4], const float q_now[4], float
  dt, float delt_angle[3]);

#endif

/*
 * File trailer for caculate_delta.h
 *
 * [EOF]
 */
