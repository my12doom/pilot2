/*
 * File: SerialUpdate.c
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
#include "inv.h"

/* Function Definitions */

/*
 * // *************  SerialUpdate *******************
 *  // Does the update step of the Kalman filter for the covariance and estimate
 *  // Outputs are Xnew & Pnew, and are written over P and X
 *  // Z is actual measurement, Y is predicted measurement
 *  // Xnew = X + K*(Z-Y), Pnew=(I-K*H)*P,
 *  // where K=P*H'*inv[H*P*H'+R]
 *  // NOTE the algorithm assumes R (measurement covariance matrix) is diagonal
 *  // i.e. the measurment noises are uncorrelated.
 *  // It therefore uses a serial update that requires no matrix inversion by
 *  // processing the measurements one at a time.
 *  // Algorithm - see Grewal and Andrews, "Kalman Filtering,2nd Ed" p.121 & p.253
 *  // - or see Simon, "Optimal State Estimation," 1st Ed, p.150
 *  // The SensorsUsed variable is a bitwise mask indicating which sensors
 *  // should be used in the update.
 *  // ************************************************
 * Arguments    : const float H[104]
 *                const float R[64]
 *                const float Z[8]
 *                const float Y[8]
 *                float P[169]
 *                float X[13]
 * Return Type  : void
 */
void SerialUpdate(const float H[104], const float R[64], const float Z[8], const
                  float Y[8], float P[169], float X[13])
{
  float b_H[104];
  int i12;
  int i13;
  int i14;
  float x[64];
  float f0;
  float y[64];
  float K[104];
  float b_Z[8];
  float b_K[169];
  float b_P[169];
  for (i12 = 0; i12 < 8; i12++) {
    for (i13 = 0; i13 < 13; i13++) {
      b_H[i12 + (i13 << 3)] = 0.0F;
      for (i14 = 0; i14 < 13; i14++) {
        b_H[i12 + (i13 << 3)] += H[i12 + (i14 << 3)] * P[i14 + 13 * i13];
      }
    }
  }

  for (i12 = 0; i12 < 8; i12++) {
    for (i13 = 0; i13 < 8; i13++) {
      f0 = 0.0F;
      for (i14 = 0; i14 < 13; i14++) {
        f0 += b_H[i12 + (i14 << 3)] * H[i13 + (i14 << 3)];
      }

      x[i12 + (i13 << 3)] = f0 + R[i12 + (i13 << 3)];
    }
  }

  invNxN(x, y);
  for (i12 = 0; i12 < 13; i12++) {
    for (i13 = 0; i13 < 8; i13++) {
      b_H[i12 + 13 * i13] = 0.0F;
      for (i14 = 0; i14 < 13; i14++) {
        b_H[i12 + 13 * i13] += P[i12 + 13 * i14] * H[i13 + (i14 << 3)];
      }
    }
  }

  for (i12 = 0; i12 < 13; i12++) {
    for (i13 = 0; i13 < 8; i13++) {
      K[i12 + 13 * i13] = 0.0F;
      for (i14 = 0; i14 < 8; i14++) {
        K[i12 + 13 * i13] += b_H[i12 + 13 * i14] * y[i14 + (i13 << 3)];
      }
    }
  }

  for (i12 = 0; i12 < 8; i12++) {
    b_Z[i12] = Z[i12] - Y[i12];
  }

  for (i12 = 0; i12 < 13; i12++) {
    f0 = 0.0F;
    for (i13 = 0; i13 < 8; i13++) {
      f0 += K[i12 + 13 * i13] * b_Z[i13];
    }

    X[i12] += f0;
  }

  for (i12 = 0; i12 < 13; i12++) {
    for (i13 = 0; i13 < 13; i13++) {
      b_K[i12 + 13 * i13] = 0.0F;
      for (i14 = 0; i14 < 8; i14++) {
        b_K[i12 + 13 * i13] += K[i12 + 13 * i14] * H[i14 + (i13 << 3)];
      }
    }
  }

  for (i12 = 0; i12 < 13; i12++) {
    for (i13 = 0; i13 < 13; i13++) {
      f0 = 0.0F;
      for (i14 = 0; i14 < 13; i14++) {
        f0 += b_K[i12 + 13 * i14] * P[i14 + 13 * i13];
      }

      b_P[i12 + 13 * i13] = P[i12 + 13 * i13] - f0;
    }
  }

  for (i12 = 0; i12 < 13; i12++) {
    for (i13 = 0; i13 < 13; i13++) {
      P[i13 + 13 * i12] = b_P[i13 + 13 * i12];
    }
  }
}

/*
 * File trailer for SerialUpdate.c
 *
 * [EOF]
 */
