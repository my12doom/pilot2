/*
 * File: RungeKutta.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 03-Dec-2015 17:00:31
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

/* Function Declarations */
extern void RungeKutta(float X[13], const float U[6], float dT);

#endif

/*
 * File trailer for RungeKutta.h
 *
 * [EOF]
 */
