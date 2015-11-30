/*
 * File: INS_CovariancePrediction.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 26-Nov-2015 12:02:43
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



#ifdef __cplusplus
extern "C" {
#endif


/* Function Declarations */
extern void INS_CovariancePrediction(const float F[169], const float G[117],
  const float Q[81], float dT, float P[169]);

#ifdef __cplusplus
}
#endif

#endif

/*
 * File trailer for INS_CovariancePrediction.h
 *
 * [EOF]
 */
