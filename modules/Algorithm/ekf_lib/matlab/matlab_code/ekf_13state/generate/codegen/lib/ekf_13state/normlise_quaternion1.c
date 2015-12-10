/*
 * File: normlise_quaternion1.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 09-Dec-2015 17:37:13
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
#include "body2ned.h"
#include "f.h"
#include "h.h"
#include "init_ekf_matrix.h"
#include "init_quaternion_by_euler.h"
#include "ned2body.h"
#include "normlise_quaternion.h"
#include "quaternion_to_euler.h"
#include "normlise_quaternion1.h"

/* Function Definitions */

/*
 * Arguments    : float X[13]
 * Return Type  : void
 */
void b_normlise_quaternion(float X[13])
{
  float norm;
  norm = (real32_T)sqrt(((X[6] * X[6] + X[7] * X[7]) + X[8] * X[8]) + X[9] * X[9]);
  X[6] /= norm;
  X[7] /= norm;
  X[8] /= norm;
  X[9] /= norm;
}

/*
 * File trailer for normlise_quaternion1.c
 *
 * [EOF]
 */
