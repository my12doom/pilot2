/*
 * File: INS_Correction.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 27-Nov-2015 13:49:02
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
 * Arguments    : const float Mag_data[3]
 *                const float Pos[3]
 *                const float Vel[2]
 *                float X[13]
 *                const float R[64]
 *                float P[169]
 *                const float Be[3]
 * Return Type  : void
 */
void INS_Correction(const float Mag_data[3], const float Pos[3], const float
                    Vel[2], float X[13], const float R[64], float P[169], const
                    float Be[3])
{
  double Z[8];
  float Bnorm;
  double H[104];
  float b_H[104];
  int i;
  int i3;
  int i4;
  float x[64];
  float y[64];
  float K[104];
  double dv1[8];
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
  /*  % Bnorm=sqrt(Mag_data(1)^2+Mag_data(2)^2+Mag_data(3)^2); */
  /*  % Mb_x=Mag_data(1)/Bnorm; */
  /*  % Mb_y=Mag_data(2)/Bnorm; */
  /*  % Mb_z=Mag_data(3)/Bnorm; */
  /*  %body frame to earth frame */
  /*  Mbe=[q0^2+q1^2-q2^2-q3^2,2*(q1*q2+q0*q3),2*(q1*q3-q0*q2);2*(q1*q2-q0*q3),q0^2-q1^2+q2^2-q3^2,2*(q2*q3+q0*q1);2*(q1*q3+q0*q2),2*(q2*q3-q0*q1),q0^2-q1^2-q2^2+q3^2]; */
  /*  %earth frame to body frame and remove z value */
  /*  Meb=Mbe'; */
  /*  Mned_x=Meb(1,1)*Mag_data(1) + Meb(1,2)*Mag_data(2) + Meb(1,3)*Mag_data(3); */
  /*  Mned_y=Meb(2,1)*Mag_data(1) + Meb(2,2)*Mag_data(2) + Meb(2,3)*Mag_data(3); */
  /*  %normlize it  */
  /*  Bnorm=sqrt(Mned_x^2+Mned_y^2); */
  /*  Mned_x=Mag_data(1)/Bnorm; */
  /*  Mned_y=Mag_data(2)/Bnorm; */
  /*  %transfer megnetic to body frame */
  /*  Mb_x=Mbe(1,1)*Mned_x + Mbe(1,2)*Mned_y; */
  /*  Mb_y=Mbe(2,1)*Mned_x + Mbe(2,2)*Mned_y; */
  /*  Mb_z=Mbe(3,1)*Mned_x + Mbe(3,2)*Mned_y; */
  /* %This is original openpilot method */
  Bnorm = (real32_T)sqrt((Mag_data[0] * Mag_data[0] + Mag_data[1] * Mag_data[1])
    + Mag_data[2] * Mag_data[2]);
  Z[5] = Mag_data[0] / Bnorm;
  Z[6] = Mag_data[1] / Bnorm;
  Z[7] = Mag_data[2] / Bnorm;
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
    for (i3 = 0; i3 < 13; i3++) {
      b_H[i + (i3 << 3)] = 0.0F;
      for (i4 = 0; i4 < 13; i4++) {
        b_H[i + (i3 << 3)] += (float)H[i + (i4 << 3)] * P[i4 + 13 * i3];
      }
    }
  }

  for (i = 0; i < 8; i++) {
    for (i3 = 0; i3 < 8; i3++) {
      Bnorm = 0.0F;
      for (i4 = 0; i4 < 13; i4++) {
        Bnorm += b_H[i + (i4 << 3)] * (float)H[i3 + (i4 << 3)];
      }

      x[i + (i3 << 3)] = Bnorm + R[i + (i3 << 3)];
    }
  }

  invNxN(x, y);
  for (i = 0; i < 13; i++) {
    for (i3 = 0; i3 < 8; i3++) {
      b_H[i + 13 * i3] = 0.0F;
      for (i4 = 0; i4 < 13; i4++) {
        b_H[i + 13 * i3] += P[i + 13 * i4] * (float)H[i3 + (i4 << 3)];
      }
    }
  }

  for (i = 0; i < 13; i++) {
    for (i3 = 0; i3 < 8; i3++) {
      K[i + 13 * i3] = 0.0F;
      for (i4 = 0; i4 < 8; i4++) {
        K[i + 13 * i3] += b_H[i + 13 * i4] * y[i4 + (i3 << 3)];
      }
    }
  }

  h(X, Be, dv1);
  for (i = 0; i < 8; i++) {
    b_Z[i] = (float)(Z[i] - dv1[i]);
  }

  for (i = 0; i < 13; i++) {
    Bnorm = 0.0F;
    for (i3 = 0; i3 < 8; i3++) {
      Bnorm += K[i + 13 * i3] * b_Z[i3];
    }

    b_X[i] = X[i] + Bnorm;
  }

  for (i = 0; i < 13; i++) {
    for (i3 = 0; i3 < 13; i3++) {
      b_K[i + 13 * i3] = 0.0F;
      for (i4 = 0; i4 < 8; i4++) {
        b_K[i + 13 * i3] += K[i + 13 * i4] * (float)H[i4 + (i3 << 3)];
      }
    }
  }

  for (i = 0; i < 13; i++) {
    for (i3 = 0; i3 < 13; i3++) {
      Bnorm = 0.0F;
      for (i4 = 0; i4 < 13; i4++) {
        Bnorm += b_K[i + 13 * i4] * P[i4 + 13 * i3];
      }

      b_P[i + 13 * i3] = P[i + 13 * i3] - Bnorm;
    }
  }

  for (i = 0; i < 13; i++) {
    for (i3 = 0; i3 < 13; i3++) {
      P[i3 + 13 * i] = b_P[i3 + 13 * i];
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
