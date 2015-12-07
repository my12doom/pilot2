/*
 * File: LinearFG.c
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
 *                const float U[6]
 *                double F[169]
 *                double G[117]
 * Return Type  : void
 */
void LinearFG(const float X[13], const float U[6], double F[169], double G[117])
{
  signed char I[9];
  int i2;
  int k;
  memset(&F[0], 0, 169U * sizeof(double));
  for (i2 = 0; i2 < 9; i2++) {
    I[i2] = 0;
  }

  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1;
    for (i2 = 0; i2 < 3; i2++) {
      F[i2 + 13 * (3 + k)] = I[i2 + 3 * k];
    }
  }

  F[81] = 2.0F * ((X[6] * U[3] - X[9] * U[4]) + X[8] * U[5]);
  F[94] = 2.0F * ((X[7] * U[3] + X[8] * U[4]) + X[9] * U[5]);
  F[107] = 2.0F * ((-X[8] * U[3] + X[7] * U[4]) + X[6] * U[5]);
  F[120] = 2.0F * ((-X[9] * U[3] - X[6] * U[4]) + X[7] * U[5]);
  F[82] = 2.0F * ((X[9] * U[3] + X[6] * U[4]) - X[7] * U[5]);
  F[95] = 2.0F * ((X[8] * U[3] - X[7] * U[4]) - X[6] * U[5]);
  F[108] = 2.0F * ((X[7] * U[3] + X[8] * U[4]) + X[9] * U[5]);
  F[121] = 2.0F * ((X[6] * U[3] - X[9] * U[4]) + X[8] * U[5]);
  F[83] = 2.0F * ((-X[8] * U[3] + X[7] * U[4]) + X[6] * U[5]);
  F[96] = 2.0F * ((X[9] * U[3] + X[6] * U[4]) - X[7] * U[5]);
  F[109] = 2.0F * ((-X[6] * U[3] + X[9] * U[4]) - X[8] * U[5]);
  F[122] = 2.0F * ((X[7] * U[3] + X[8] * U[4]) + X[9] * U[5]);
  F[84] = 0.0;
  F[97] = -U[0] / 2.0F;
  F[110] = -U[1] / 2.0F;
  F[123] = -U[2] / 2.0F;
  F[85] = U[0] / 2.0F;
  F[98] = 0.0;
  F[111] = U[2] / 2.0F;
  F[124] = -U[1] / 2.0F;
  F[86] = U[1] / 2.0F;
  F[99] = -U[2] / 2.0F;
  F[112] = 0.0;
  F[125] = U[0] / 2.0F;
  F[87] = U[2] / 2.0F;
  F[100] = U[1] / 2.0F;
  F[113] = -U[0] / 2.0F;
  F[126] = 0.0;
  F[136] = X[7] / 2.0F;
  F[149] = X[8] / 2.0F;
  F[162] = X[9] / 2.0F;
  F[137] = -X[6] / 2.0F;
  F[150] = X[9] / 2.0F;
  F[163] = -X[8] / 2.0F;
  F[138] = -X[9] / 2.0F;
  F[151] = -X[6] / 2.0F;
  F[164] = X[7] / 2.0F;
  F[139] = X[8] / 2.0F;
  F[152] = -X[7] / 2.0F;
  F[165] = -X[6] / 2.0F;

  /* caculate G */
  /*  dVdot/dna  - NO BIAS STATES ON ACCELS - S0 REPEAT FOR G HERE */
  /*  dqdot/dnw */
  memset(&G[0], 0, 117U * sizeof(double));
  G[42] = ((-X[6] * X[6] - X[7] * X[7]) + X[8] * X[8]) + X[9] * X[9];
  G[55] = 2.0F * (-X[7] * X[8] + X[6] * X[9]);
  G[68] = -2.0F * (X[7] * X[9] + X[6] * X[8]);
  G[43] = -2.0F * (X[7] * X[8] + X[6] * X[9]);
  G[56] = ((-X[6] * X[6] + X[7] * X[7]) - X[8] * X[8]) + X[9] * X[9];
  G[69] = 2.0F * (-X[8] * X[9] + X[6] * X[7]);
  G[44] = 2.0F * (-X[7] * X[9] + X[6] * X[8]);
  G[57] = -2.0F * (X[8] * X[9] + X[6] * X[7]);
  G[70] = ((-X[6] * X[6] + X[7] * X[7]) + X[8] * X[8]) - X[9] * X[9];
  G[6] = X[7] / 2.0F;
  G[19] = X[8] / 2.0F;
  G[32] = X[9] / 2.0F;
  G[7] = -X[6] / 2.0F;
  G[20] = X[9] / 2.0F;
  G[33] = -X[8] / 2.0F;
  G[8] = -X[9] / 2.0F;
  G[21] = -X[6] / 2.0F;
  G[34] = X[7] / 2.0F;
  G[9] = X[8] / 2.0F;
  G[22] = -X[7] / 2.0F;
  G[35] = -X[6] / 2.0F;
  for (i2 = 0; i2 < 9; i2++) {
    I[i2] = 0;
  }

  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1;
    for (i2 = 0; i2 < 3; i2++) {
      G[(i2 + 13 * (6 + k)) + 10] = I[i2 + 3 * k];
    }
  }
}

/*
 * File trailer for LinearFG.c
 *
 * [EOF]
 */
