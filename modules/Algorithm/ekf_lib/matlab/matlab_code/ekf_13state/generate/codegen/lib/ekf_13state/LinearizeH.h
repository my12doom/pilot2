/*
 * File: LinearizeH.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 09-Dec-2015 17:37:13
 */

#ifndef __LINEARIZEH_H__
#define __LINEARIZEH_H__

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
extern void LinearizeH(const float X[13], const float Be[3], double H[104]);

#endif

/*
 * File trailer for LinearizeH.h
 *
 * [EOF]
 */
