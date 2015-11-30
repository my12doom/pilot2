/*
 * File: RungeKutta.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 26-Nov-2015 12:02:43
 */

#ifndef __RUNGEKUTTA_H__
#define __RUNGEKUTTA_H__

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
extern void RungeKutta(float X[13], const float U[6], float dT);

#ifdef __cplusplus
}
#endif

#endif

/*
 * File trailer for RungeKutta.h
 *
 * [EOF]
 */
