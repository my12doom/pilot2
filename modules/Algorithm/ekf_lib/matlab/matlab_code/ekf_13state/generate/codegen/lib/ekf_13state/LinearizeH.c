/*
 * File: LinearizeH.c
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
 *                double H[104]
 * Return Type  : void
 */
void LinearizeH(const float X[13], const float Be[3], double H[104])
{
  signed char I[9];
  int i0;
  int k;
  static const signed char iv0[6] = { 1, 0, 0, 1, 0, 0 };

  memset(&H[0], 0, 104U * sizeof(double));

  /* dP/dP=I; */
  for (i0 = 0; i0 < 9; i0++) {
    I[i0] = 0;
  }

  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1;
    for (i0 = 0; i0 < 3; i0++) {
      H[i0 + (k << 3)] = I[i0 + 3 * k];
    }
  }

  /* dV/dV=I; */
  for (i0 = 0; i0 < 3; i0++) {
    for (k = 0; k < 2; k++) {
      H[(k + ((3 + i0) << 3)) + 3] = iv0[k + (i0 << 1)];
    }
  }

  /* dBb/dq */
  H[53] = 2.0F * ((X[6] * Be[0] + X[9] * Be[1]) - X[8] * Be[2]);
  H[61] = 2.0F * ((X[7] * Be[0] + X[8] * Be[1]) + X[9] * Be[2]);
  H[69] = 2.0F * ((-X[8] * Be[0] + X[7] * Be[1]) - X[6] * Be[2]);
  H[77] = 2.0F * ((-X[9] * Be[0] + X[6] * Be[1]) + X[7] * Be[2]);
  H[54] = 2.0F * ((-X[9] * Be[0] + X[6] * Be[1]) + X[7] * Be[2]);
  H[62] = 2.0F * ((X[8] * Be[0] - X[7] * Be[1]) + X[6] * Be[2]);
  H[70] = 2.0F * ((X[7] * Be[0] + X[8] * Be[1]) + X[9] * Be[2]);
  H[78] = 2.0F * ((-X[6] * Be[0] - X[9] * Be[1]) + X[8] * Be[2]);
  H[55] = 2.0F * ((X[8] * Be[0] - X[7] * Be[1]) + X[6] * Be[2]);
  H[63] = 2.0F * ((X[9] * Be[0] - X[6] * Be[1]) - X[7] * Be[2]);
  H[71] = 2.0F * ((X[6] * Be[0] + X[9] * Be[1]) - X[8] * Be[2]);
  H[79] = 2.0F * ((X[7] * Be[0] + X[8] * Be[1]) + X[9] * Be[2]);
}

/*
 * File trailer for LinearizeH.c
 *
 * [EOF]
 */
