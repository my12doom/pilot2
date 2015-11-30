/*
 * File: INS_CovariancePrediction.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 26-Nov-2015 12:02:43
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
  int i5;
  int i6;
  static const signed char iv2[169] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  float fv1[169];
  int i7;
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
  for (i5 = 0; i5 < 13; i5++) {
    for (i6 = 0; i6 < 13; i6++) {
      fv0[i6 + 13 * i5] = (float)iv2[i6 + 13 * i5] + F[i6 + 13 * i5] * dT;
    }
  }

  for (i5 = 0; i5 < 13; i5++) {
    for (i6 = 0; i6 < 13; i6++) {
      fv1[i5 + 13 * i6] = 0.0F;
      for (i7 = 0; i7 < 13; i7++) {
        fv1[i5 + 13 * i6] += fv0[i5 + 13 * i7] * P[i7 + 13 * i6];
      }
    }
  }

  for (i5 = 0; i5 < 13; i5++) {
    for (i6 = 0; i6 < 13; i6++) {
      fv0[i6 + 13 * i5] = (float)iv2[i5 + 13 * i6] + F[i5 + 13 * i6] * dT;
    }

    for (i6 = 0; i6 < 9; i6++) {
      b_c[i5 + 13 * i6] = 0.0F;
      for (i7 = 0; i7 < 9; i7++) {
        b_c[i5 + 13 * i6] += c * G[i5 + 13 * i7] * Q[i7 + 9 * i6];
      }
    }
  }

  for (i5 = 0; i5 < 13; i5++) {
    for (i6 = 0; i6 < 13; i6++) {
      fv2[i5 + 13 * i6] = 0.0F;
      for (i7 = 0; i7 < 13; i7++) {
        fv2[i5 + 13 * i6] += fv1[i5 + 13 * i7] * fv0[i7 + 13 * i6];
      }

      c_c[i5 + 13 * i6] = 0.0F;
      for (i7 = 0; i7 < 9; i7++) {
        c_c[i5 + 13 * i6] += b_c[i5 + 13 * i7] * G[i6 + 13 * i7];
      }
    }
  }

  for (i5 = 0; i5 < 13; i5++) {
    for (i6 = 0; i6 < 13; i6++) {
      P[i6 + 13 * i5] = fv2[i6 + 13 * i5] + c_c[i6 + 13 * i5];
    }
  }
}

/*
 * File trailer for INS_CovariancePrediction.c
 *
 * [EOF]
 */
