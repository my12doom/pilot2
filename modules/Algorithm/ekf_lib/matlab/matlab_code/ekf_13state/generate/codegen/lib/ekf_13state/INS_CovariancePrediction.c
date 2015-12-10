/*
 * File: INS_CovariancePrediction.c
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

/* Function Definitions */

/*
 * Arguments    : const float F[169]
 *                const float G[117]
 *                const float Q[81]
 *                float dT
 *                float P[169]
 * Return Type  : void
 */
void INS_CovariancePrediction(const float F[169], const float G[117], const
  float Q[81], float dT, float P[169])
{
  float c;
  float fv0[169];
  int i9;
  int i10;
  static const signed char iv2[169] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  float fv1[169];
  int i11;
  float b_c[117];
  float fv2[169];
  float c_c[169];

  /*  // *************  CovariancePrediction ************* */
  /*  // Does the prediction step of the Kalman filter for the covariance matrix */
  /*  // Output, Pnew, overwrites P, the input covariance */
  /*  // Pnew = (I+F*T)*P*(I+F*T)' + T^2*G*Q*G' */
  /*  // Q is the discrete time covariance of process noise */
  /*  // Q is vector of the diagonal for a square matrix with */
  /*  // dimensions equal to the number of disturbance noise variables */
  /*  // The General Method is very inefficient,not taking advantage of the sparse F and G */
  /*  // The first Method is very specific to this implementation */
  /*  // ************************************************ */
  c = dT * dT;
  for (i9 = 0; i9 < 13; i9++) {
    for (i10 = 0; i10 < 13; i10++) {
      fv0[i10 + 13 * i9] = (float)iv2[i10 + 13 * i9] + F[i10 + 13 * i9] * dT;
    }
  }

  for (i9 = 0; i9 < 13; i9++) {
    for (i10 = 0; i10 < 13; i10++) {
      fv1[i9 + 13 * i10] = 0.0F;
      for (i11 = 0; i11 < 13; i11++) {
        fv1[i9 + 13 * i10] += fv0[i9 + 13 * i11] * P[i11 + 13 * i10];
      }
    }
  }

  for (i9 = 0; i9 < 13; i9++) {
    for (i10 = 0; i10 < 13; i10++) {
      fv0[i10 + 13 * i9] = (float)iv2[i9 + 13 * i10] + F[i9 + 13 * i10] * dT;
    }

    for (i10 = 0; i10 < 9; i10++) {
      b_c[i9 + 13 * i10] = 0.0F;
      for (i11 = 0; i11 < 9; i11++) {
        b_c[i9 + 13 * i10] += c * G[i9 + 13 * i11] * Q[i11 + 9 * i10];
      }
    }
  }

  for (i9 = 0; i9 < 13; i9++) {
    for (i10 = 0; i10 < 13; i10++) {
      fv2[i9 + 13 * i10] = 0.0F;
      for (i11 = 0; i11 < 13; i11++) {
        fv2[i9 + 13 * i10] += fv1[i9 + 13 * i11] * fv0[i11 + 13 * i10];
      }

      c_c[i9 + 13 * i10] = 0.0F;
      for (i11 = 0; i11 < 9; i11++) {
        c_c[i9 + 13 * i10] += b_c[i9 + 13 * i11] * G[i10 + 13 * i11];
      }
    }
  }

  for (i9 = 0; i9 < 13; i9++) {
    for (i10 = 0; i10 < 13; i10++) {
      P[i10 + 13 * i9] = fv2[i10 + 13 * i9] + c_c[i10 + 13 * i9];
    }
  }
}

/*
 * File trailer for INS_CovariancePrediction.c
 *
 * [EOF]
 */
