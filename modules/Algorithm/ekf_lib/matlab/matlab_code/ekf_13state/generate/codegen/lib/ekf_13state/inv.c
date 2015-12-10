/*
 * File: inv.c
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
 * Arguments    : const float x[64]
 *                float y[64]
 * Return Type  : void
 */
void invNxN(const float x[64], float y[64])
{
  float A[64];
  int i2;
  signed char ipiv[8];
  int j;
  int c;
  int jBcol;
  int ix;
  float smax;
  int k;
  float s;
  int i;
  int kAcol;
  signed char p[8];
  for (i2 = 0; i2 < 64; i2++) {
    y[i2] = 0.0F;
    A[i2] = x[i2];
  }

  for (i2 = 0; i2 < 8; i2++) {
    ipiv[i2] = (signed char)(1 + i2);
  }

  for (j = 0; j < 7; j++) {
    c = j * 9;
    jBcol = 0;
    ix = c;
    smax = (real32_T)fabs(A[c]);
    for (k = 2; k <= 8 - j; k++) {
      ix++;
      s = (real32_T)fabs(A[ix]);
      if (s > smax) {
        jBcol = k - 1;
        smax = s;
      }
    }

    if (A[c + jBcol] != 0.0F) {
      if (jBcol != 0) {
        ipiv[j] = (signed char)((j + jBcol) + 1);
        ix = j;
        jBcol += j;
        for (k = 0; k < 8; k++) {
          smax = A[ix];
          A[ix] = A[jBcol];
          A[jBcol] = smax;
          ix += 8;
          jBcol += 8;
        }
      }

      i2 = (c - j) + 8;
      for (i = c + 1; i + 1 <= i2; i++) {
        A[i] /= A[c];
      }
    }

    jBcol = c;
    kAcol = c + 8;
    for (i = 1; i <= 7 - j; i++) {
      smax = A[kAcol];
      if (A[kAcol] != 0.0F) {
        ix = c + 1;
        i2 = (jBcol - j) + 16;
        for (k = 9 + jBcol; k + 1 <= i2; k++) {
          A[k] += A[ix] * -smax;
          ix++;
        }
      }

      kAcol += 8;
      jBcol += 8;
    }
  }

  for (i2 = 0; i2 < 8; i2++) {
    p[i2] = (signed char)(1 + i2);
  }

  for (k = 0; k < 7; k++) {
    if (ipiv[k] > 1 + k) {
      jBcol = p[ipiv[k] - 1];
      p[ipiv[k] - 1] = p[k];
      p[k] = (signed char)jBcol;
    }
  }

  for (k = 0; k < 8; k++) {
    y[k + ((p[k] - 1) << 3)] = 1.0F;
    for (j = k; j + 1 < 9; j++) {
      if (y[j + ((p[k] - 1) << 3)] != 0.0F) {
        for (i = j + 1; i + 1 < 9; i++) {
          y[i + ((p[k] - 1) << 3)] -= y[j + ((p[k] - 1) << 3)] * A[i + (j << 3)];
        }
      }
    }
  }

  for (j = 0; j < 8; j++) {
    jBcol = j << 3;
    for (k = 7; k > -1; k += -1) {
      kAcol = k << 3;
      if (y[k + jBcol] != 0.0F) {
        y[k + jBcol] /= A[k + kAcol];
        for (i = 0; i + 1 <= k; i++) {
          y[i + jBcol] -= y[k + jBcol] * A[i + kAcol];
        }
      }
    }
  }
}

/*
 * File trailer for inv.c
 *
 * [EOF]
 */
