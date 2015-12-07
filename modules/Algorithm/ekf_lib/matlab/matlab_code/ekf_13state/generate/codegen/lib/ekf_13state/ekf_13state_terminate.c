/*
 * File: ekf_13state_terminate.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 03-Dec-2015 17:00:31
 */

/* Include files */
#include "rt_nonfinite.h"
#include "INSSetMagNorth.h"
#include "INS_Correction.h"
#include "INS_CovariancePrediction.h"
#include "INS_SetState.h"
#include "LinearFG.h"
#include "LinearizeH.h"
#include "RungeKutta.h"
#include "SerialUpdate.h"
#include "f.h"
#include "h.h"
#include "init_ekf_matrix.h"
#include "init_quaternion_by_euler.h"
#include "normlise_quaternion.h"
#include "quaternion_to_euler.h"
#include "ekf_13state_terminate.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void ekf_13state_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for ekf_13state_terminate.c
 *
 * [EOF]
 */
