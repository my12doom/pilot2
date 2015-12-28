/*
 * File: INS_Correction.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 28-Dec-2015 15:54:29
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
 * Arguments    : const float Mag_data[3]
 *                const float Pos[3]
 *                const float Vel[3]
 *                float X[13]
 *                const float R[64]
 *                float P[169]
 *                const float Be[3]
 * Return Type  : void
 */
void INS_Correction(const float Mag_data[3], const float Pos[3], const float
                    Vel[3], float X[13], const float R[64], float P[169], const
                    float Be[3])
{
  double Z[8];
  float q_now[4];
  float Me[3];
  float Bnorm;
  float b_Me[3];
  double H[104];
  float b_H[104];
  int i;
  int i7;
  int i8;
  float x[64];
  float y[64];
  float K[104];
  double dv0[8];
  float b_Z[8];
  float b_X[13];
  float b_K[169];
  float b_P[169];
  Z[0] = Pos[0];
  Z[1] = Pos[1];
  Z[2] = Pos[2];
  Z[3] = Vel[0];
  Z[4] = Vel[1];

  /*  %% do lots of things to remove megnetic Z value */
  q_now[0] = X[6];
  q_now[1] = X[7];
  q_now[2] = X[8];
  q_now[3] = X[9];
  body2ned(q_now, Mag_data, Me);
  Bnorm = (real32_T)sqrt(Me[0] * Me[0] + Me[1] * Me[1]);
  b_Me[0] = Me[0] / Bnorm;
  b_Me[1] = Me[1] / Bnorm;
  b_Me[2] = 0.0F;
  ned2body(q_now, b_Me, Me);

  /* % */
  Z[5] = Me[0];
  Z[6] = Me[1];
  Z[7] = Me[2];
  LinearizeH(X, Be, H);

  /*  // *************  SerialUpdate ******************* */
  /*  // Does the update step of the Kalman filter for the covariance and estimate */
  /*  // Outputs are Xnew & Pnew, and are written over P and X */
  /*  // Z is actual measurement, Y is predicted measurement */
  /*  // Xnew = X + K*(Z-Y), Pnew=(I-K*H)*P, */
  /*  // where K=P*H'*inv[H*P*H'+R] */
  /*  // NOTE the algorithm assumes R (measurement covariance matrix) is diagonal */
  /*  // i.e. the measurment noises are uncorrelated. */
  /*  // It therefore uses a serial update that requires no matrix inversion by */
  /*  // processing the measurements one at a time. */
  /*  // Algorithm - see Grewal and Andrews, "Kalman Filtering,2nd Ed" p.121 & p.253 */
  /*  // - or see Simon, "Optimal State Estimation," 1st Ed, p.150 */
  /*  // The SensorsUsed variable is a bitwise mask indicating which sensors */
  /*  // should be used in the update. */
  /*  // ************************************************ */
  for (i = 0; i < 8; i++) {
    for (i7 = 0; i7 < 13; i7++) {
      b_H[i + (i7 << 3)] = 0.0F;
      for (i8 = 0; i8 < 13; i8++) {
        b_H[i + (i7 << 3)] += (float)H[i + (i8 << 3)] * P[i8 + 13 * i7];
      }
    }
  }

  for (i = 0; i < 8; i++) {
    for (i7 = 0; i7 < 8; i7++) {
      Bnorm = 0.0F;
      for (i8 = 0; i8 < 13; i8++) {
        Bnorm += b_H[i + (i8 << 3)] * (float)H[i7 + (i8 << 3)];
      }

      x[i + (i7 << 3)] = Bnorm + R[i + (i7 << 3)];
    }
  }

  invNxN(x, y);
  for (i = 0; i < 13; i++) {
    for (i7 = 0; i7 < 8; i7++) {
      b_H[i + 13 * i7] = 0.0F;
      for (i8 = 0; i8 < 13; i8++) {
        b_H[i + 13 * i7] += P[i + 13 * i8] * (float)H[i7 + (i8 << 3)];
      }
    }
  }

  for (i = 0; i < 13; i++) {
    for (i7 = 0; i7 < 8; i7++) {
      K[i + 13 * i7] = 0.0F;
      for (i8 = 0; i8 < 8; i8++) {
        K[i + 13 * i7] += b_H[i + 13 * i8] * y[i8 + (i7 << 3)];
      }
    }
  }

  h(X, Be, dv0);
  for (i = 0; i < 8; i++) {
    b_Z[i] = (float)(Z[i] - dv0[i]);
  }

  for (i = 0; i < 13; i++) {
    Bnorm = 0.0F;
    for (i7 = 0; i7 < 8; i7++) {
      Bnorm += K[i + 13 * i7] * b_Z[i7];
    }

    b_X[i] = X[i] + Bnorm;
  }

  for (i = 0; i < 13; i++) {
    for (i7 = 0; i7 < 13; i7++) {
      b_K[i + 13 * i7] = 0.0F;
      for (i8 = 0; i8 < 8; i8++) {
        b_K[i + 13 * i7] += K[i + 13 * i8] * (float)H[i8 + (i7 << 3)];
      }
    }
  }

  for (i = 0; i < 13; i++) {
    for (i7 = 0; i7 < 13; i7++) {
      Bnorm = 0.0F;
      for (i8 = 0; i8 < 13; i8++) {
        Bnorm += b_K[i + 13 * i8] * P[i8 + 13 * i7];
      }

      b_P[i + 13 * i7] = P[i + 13 * i7] - Bnorm;
    }
  }

  for (i = 0; i < 13; i++) {
    for (i7 = 0; i7 < 13; i7++) {
      P[i7 + 13 * i] = b_P[i7 + 13 * i];
    }
  }

  for (i = 0; i < 13; i++) {
    X[i] = b_X[i];
  }

  Bnorm = (real32_T)sqrt(((b_X[6] * b_X[6] + b_X[7] * b_X[7]) + b_X[8] * b_X[8])
    + b_X[9] * b_X[9]);
  X[6] = b_X[6] / Bnorm;
  X[7] = b_X[7] / Bnorm;
  X[8] = b_X[8] / Bnorm;
  X[9] = b_X[9] / Bnorm;
}

/*
 * File trailer for INS_Correction.c
 *
 * [EOF]
 */
