/*
 * File: SerialUpdate.c
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
  int i8;
  int i9;
  int i10;
  float x[64];
  float f0;
  float y[64];
  float K[104];
  float b_Z[8];
  float b_K[169];
  float b_P[169];
  for (i8 = 0; i8 < 8; i8++) {
    for (i9 = 0; i9 < 13; i9++) {
      b_H[i8 + (i9 << 3)] = 0.0F;
      for (i10 = 0; i10 < 13; i10++) {
        b_H[i8 + (i9 << 3)] += H[i8 + (i10 << 3)] * P[i10 + 13 * i9];
      }
    }
  }

  for (i8 = 0; i8 < 8; i8++) {
    for (i9 = 0; i9 < 8; i9++) {
      f0 = 0.0F;
      for (i10 = 0; i10 < 13; i10++) {
        f0 += b_H[i8 + (i10 << 3)] * H[i9 + (i10 << 3)];
      }

      x[i8 + (i9 << 3)] = f0 + R[i8 + (i9 << 3)];
    }
  }

  invNxN(x, y);
  for (i8 = 0; i8 < 13; i8++) {
    for (i9 = 0; i9 < 8; i9++) {
      b_H[i8 + 13 * i9] = 0.0F;
      for (i10 = 0; i10 < 13; i10++) {
        b_H[i8 + 13 * i9] += P[i8 + 13 * i10] * H[i9 + (i10 << 3)];
      }
    }
  }

  for (i8 = 0; i8 < 13; i8++) {
    for (i9 = 0; i9 < 8; i9++) {
      K[i8 + 13 * i9] = 0.0F;
      for (i10 = 0; i10 < 8; i10++) {
        K[i8 + 13 * i9] += b_H[i8 + 13 * i10] * y[i10 + (i9 << 3)];
      }
    }
  }

  for (i8 = 0; i8 < 8; i8++) {
    b_Z[i8] = Z[i8] - Y[i8];
  }

  for (i8 = 0; i8 < 13; i8++) {
    f0 = 0.0F;
    for (i9 = 0; i9 < 8; i9++) {
      f0 += K[i8 + 13 * i9] * b_Z[i9];
    }

    X[i8] += f0;
  }

  for (i8 = 0; i8 < 13; i8++) {
    for (i9 = 0; i9 < 13; i9++) {
      b_K[i8 + 13 * i9] = 0.0F;
      for (i10 = 0; i10 < 8; i10++) {
        b_K[i8 + 13 * i9] += K[i8 + 13 * i10] * H[i10 + (i9 << 3)];
      }
    }
  }

  for (i8 = 0; i8 < 13; i8++) {
    for (i9 = 0; i9 < 13; i9++) {
      f0 = 0.0F;
      for (i10 = 0; i10 < 13; i10++) {
        f0 += b_K[i8 + 13 * i10] * P[i10 + 13 * i9];
      }

      b_P[i8 + 13 * i9] = P[i8 + 13 * i9] - f0;
    }
  }

  for (i8 = 0; i8 < 13; i8++) {
    for (i9 = 0; i9 < 13; i9++) {
      P[i9 + 13 * i8] = b_P[i9 + 13 * i8];
    }
  }
}

/*
 * File trailer for SerialUpdate.c
 *
 * [EOF]
 */
