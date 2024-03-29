/*
 * File: SerialUpdate.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 28-Dec-2015 15:54:29
 */

#ifndef __SERIALUPDATE_H__
#define __SERIALUPDATE_H__

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
extern void SerialUpdate(const float H[104], const float R[64], const float Z[8],
  const float Y[8], float P[169], float X[13]);

#endif

/*
 * File trailer for SerialUpdate.h
 *
 * [EOF]
 */
