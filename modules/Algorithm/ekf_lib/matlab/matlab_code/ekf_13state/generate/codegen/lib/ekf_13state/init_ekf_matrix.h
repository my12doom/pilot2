/*
 * File: init_ekf_matrix.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 03-Dec-2015 17:00:31
 */

#ifndef __INIT_EKF_MATRIX_H__
#define __INIT_EKF_MATRIX_H__

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
extern void init_ekf_matrix(double Be[3], double P[169], double X[13], double Q
  [81], double R[64]);

#endif

/*
 * File trailer for init_ekf_matrix.h
 *
 * [EOF]
 */
