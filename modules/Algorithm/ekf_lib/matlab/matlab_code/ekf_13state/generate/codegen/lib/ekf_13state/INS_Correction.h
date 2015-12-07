/*
 * File: INS_Correction.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 03-Dec-2015 17:00:31
 */

#ifndef __INS_CORRECTION_H__
#define __INS_CORRECTION_H__

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
extern void INS_Correction(const float Mag_data[3], const float Pos[3], const
  float Vel[2], float X[13], const float R[64], float P[169], const float Be[3]);

#endif

/*
 * File trailer for INS_Correction.h
 *
 * [EOF]
 */
