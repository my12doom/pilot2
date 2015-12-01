/*
 * File: init_ekf_matrix.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 26-Nov-2015 12:02:43
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

#ifdef __cplusplus
extern "C" {
#endif



/* Function Declarations */
extern void init_ekf_matrix(float Be[3], float P[169], float X[13], float Q
  [81], float R[64]);

#ifdef __cplusplus
}
#endif

#endif

/*
 * File trailer for init_ekf_matrix.h
 *
 * [EOF]
 */
