/*
 * File: INS_CovariancePrediction.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 28-Dec-2015 15:54:29
 */

#ifndef __INS_COVARIANCEPREDICTION_H__
#define __INS_COVARIANCEPREDICTION_H__

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
extern void INS_CovariancePrediction(const float F[169], const float G[117],
  const float Q[81], float dT, float P[169]);

#endif

/*
 * File trailer for INS_CovariancePrediction.h
 *
 * [EOF]
 */
