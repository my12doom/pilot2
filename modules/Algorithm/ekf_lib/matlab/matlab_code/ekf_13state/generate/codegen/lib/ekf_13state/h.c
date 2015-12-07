/*
 * File: h.c
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

/* Function Definitions */

/*
 * Arguments    : const float X[13]
 *                const float Be[3]
 *                double Y[8]
 * Return Type  : void
 */
void h(const float X[13], const float Be[3], double Y[8])
{
  Y[0] = X[0];
  Y[1] = X[1];
  Y[2] = X[2];
  Y[3] = X[3];
  Y[4] = X[4];

  /* Bb=Rbe*Be */
  Y[5] = ((((X[6] * X[6] + X[7] * X[7]) - X[8] * X[8]) - X[9] * X[9]) * Be[0] +
          2.0F * (X[7] * X[8] + X[6] * X[9]) * Be[1]) + 2.0F * (X[7] * X[9] - X
    [6] * X[8]) * Be[2];
  Y[6] = (2.0F * (X[7] * X[8] - X[6] * X[9]) * Be[0] + (((X[6] * X[6] - X[7] *
             X[7]) + X[8] * X[8]) - X[9] * X[9]) * Be[1]) + 2.0F * (X[8] * X[9]
    + X[6] * X[7]) * Be[2];
  Y[7] = (2.0F * (X[7] * X[9] + X[6] * X[8]) * Be[0] + 2.0F * (X[8] * X[9] - X[6]
           * X[7]) * Be[1]) + (((X[6] * X[6] - X[7] * X[7]) - X[8] * X[8]) + X[9]
    * X[9]) * Be[2];
}

/*
 * File trailer for h.c
 *
 * [EOF]
 */
