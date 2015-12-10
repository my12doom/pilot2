/*
 * File: f1.c
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
#include "f1.h"

/* Function Definitions */

/*
 * Arguments    : const float X[13]
 *                const float U[6]
 *                double Xresult[13]
 * Return Type  : void
 */
void b_f(const float X[13], const float U[6], double Xresult[13])
{
  float wx;
  float wy;
  float wz;

  /*  NO BIAS STATES ON ACCELS */
  wx = U[0] - X[10];
  wy = U[1] - X[11];
  wz = U[2] - X[12];

  /*  subtract the biases on gyros */
  Xresult[0] = X[3];
  Xresult[1] = X[4];
  Xresult[2] = X[5];

  /*  Vdot = Reb*a */
  Xresult[3] = ((((X[6] * X[6] + X[7] * X[7]) - X[8] * X[8]) - X[9] * X[9]) * U
                [3] + 2.0F * (X[7] * X[8] - X[6] * X[9]) * U[4]) + 2.0F * (X[7] *
    X[9] + X[6] * X[8]) * U[5];
  Xresult[4] = (2.0F * (X[7] * X[8] + X[6] * X[9]) * U[3] + (((X[6] * X[6] - X[7]
    * X[7]) + X[8] * X[8]) - X[9] * X[9]) * U[4]) + 2.0F * (X[8] * X[9] - X[6] *
    X[7]) * U[5];
  Xresult[5] = ((2.0F * (X[7] * X[9] - X[6] * X[8]) * U[3] + 2.0F * (X[8] * X[9]
    + X[6] * X[7]) * U[4]) + (((X[6] * X[6] - X[7] * X[7]) - X[8] * X[8]) + X[9]
    * X[9]) * U[5]) + 9.89F;

  /* earth gravity */
  /*  qdot = Q*w */
  Xresult[6] = ((-X[7] * wx - X[8] * wy) - X[9] * wz) / 2.0F;
  Xresult[7] = ((X[6] * wx - X[9] * wy) + X[8] * wz) / 2.0F;
  Xresult[8] = ((X[9] * wx + X[6] * wy) - X[7] * wz) / 2.0F;
  Xresult[9] = ((-X[8] * wx + X[7] * wy) + X[6] * wz) / 2.0F;

  /*  best guess is that bias stays constant */
  Xresult[10] = 0.0;
  Xresult[11] = 0.0;
  Xresult[12] = 0.0;
}

/*
 * File trailer for f1.c
 *
 * [EOF]
 */
