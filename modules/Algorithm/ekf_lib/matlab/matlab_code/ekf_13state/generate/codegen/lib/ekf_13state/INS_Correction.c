/*
 * File: INS_Correction.c
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
#include "normlise_quaternion1.h"
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
  signed char I[9];
  int i7;
  int k;
  static const signed char iv1[6] = { 1, 0, 0, 1, 0, 0 };

  double Y[8];
  float b_H[104];
  int i8;
  float x[64];
  float y[64];
  float K[169];
  float b_K[104];
  float b_P[169];
  float b_Z[8];
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
  memset(&H[0], 0, 104U * sizeof(double));

  /* dP/dP=I; */
  for (i7 = 0; i7 < 9; i7++) {
    I[i7] = 0;
  }

  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1;
    for (i7 = 0; i7 < 3; i7++) {
      H[i7 + (k << 3)] = I[i7 + 3 * k];
    }
  }

  /* dV/dV=I; */
  for (i7 = 0; i7 < 3; i7++) {
    for (k = 0; k < 2; k++) {
      H[(k + ((3 + i7) << 3)) + 3] = iv1[k + (i7 << 1)];
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
  for (i7 = 0; i7 < 8; i7++) {
    for (k = 0; k < 13; k++) {
      b_H[i7 + (k << 3)] = 0.0F;
      for (i8 = 0; i8 < 13; i8++) {
        b_H[i7 + (k << 3)] += (float)H[i7 + (i8 << 3)] * P[i8 + 13 * k];
      }
    }
  }

  for (i7 = 0; i7 < 8; i7++) {
    for (k = 0; k < 8; k++) {
      Bnorm = 0.0F;
      for (i8 = 0; i8 < 13; i8++) {
        Bnorm += b_H[i7 + (i8 << 3)] * (float)H[k + (i8 << 3)];
      }

      x[i7 + (k << 3)] = Bnorm + R[i7 + (k << 3)];
    }
  }

  invNxN(x, y);
  for (i7 = 0; i7 < 13; i7++) {
    for (k = 0; k < 8; k++) {
      b_H[i7 + 13 * k] = 0.0F;
      for (i8 = 0; i8 < 13; i8++) {
        b_H[i7 + 13 * k] += P[i7 + 13 * i8] * (float)H[k + (i8 << 3)];
      }
    }
  }

  for (i7 = 0; i7 < 13; i7++) {
    for (k = 0; k < 8; k++) {
      b_K[i7 + 13 * k] = 0.0F;
      for (i8 = 0; i8 < 8; i8++) {
        b_K[i7 + 13 * k] += b_H[i7 + 13 * i8] * y[i8 + (k << 3)];
      }
    }

    for (k = 0; k < 13; k++) {
      K[i7 + 13 * k] = 0.0F;
      for (i8 = 0; i8 < 8; i8++) {
        K[i7 + 13 * k] += b_K[i7 + 13 * i8] * (float)H[i8 + (k << 3)];
      }
    }
  }

  for (i7 = 0; i7 < 13; i7++) {
    for (k = 0; k < 13; k++) {
      Bnorm = 0.0F;
      for (i8 = 0; i8 < 13; i8++) {
        Bnorm += K[i7 + 13 * i8] * P[i8 + 13 * k];
      }

      b_P[i7 + 13 * k] = P[i7 + 13 * k] - Bnorm;
    }
  }

  for (i7 = 0; i7 < 13; i7++) {
    for (k = 0; k < 13; k++) {
      P[k + 13 * i7] = b_P[k + 13 * i7];
    }
  }

  for (i7 = 0; i7 < 8; i7++) {
    b_Z[i7] = (float)(Z[i7] - Y[i7]);
  }

  for (i7 = 0; i7 < 13; i7++) {
    Bnorm = 0.0F;
    for (k = 0; k < 8; k++) {
      Bnorm += b_K[i7 + 13 * k] * b_Z[k];
    }

    X[i7] += Bnorm;
  }

  b_normlise_quaternion(X);
}

/*
 * File trailer for INS_Correction.c
 *
 * [EOF]
 */
