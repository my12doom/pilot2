/*
 * File: inv.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 28-Dec-2015 15:54:29
 */

#ifndef __INV_H__
#define __INV_H__

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
extern void invNxN(const float x[64], float y[64]);

#endif

/*
 * File trailer for inv.h
 *
 * [EOF]
 */
