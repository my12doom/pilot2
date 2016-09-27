/*
 * File: mrdivide.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 07-Dec-2015 22:29:36
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
#include "mrdivide.h"

/* Function Declarations */
static float eml_matlab_zlarfg(int n, float *alpha1, float x[12], int ix0);
static float eml_xnrm2(int n, const float x[12], int ix0);
static float rt_hypotf_snf(float u0, float u1);

/* Function Definitions */

/*
 * Arguments    : int n
 *                float *alpha1
 *                float x[12]
 *                int ix0
 * Return Type  : float
 */
static float eml_matlab_zlarfg(int n, float *alpha1, float x[12], int ix0)
{
  float tau;
  float xnorm;
  int knt;
  int i6;
  int k;
  tau = 0.0F;
  if (n <= 0) {
  } else {
    xnorm = eml_xnrm2(n - 1, x, ix0);
    if (xnorm != 0.0F) {
      xnorm = rt_hypotf_snf(*alpha1, xnorm);
      if (*alpha1 >= 0.0F) {
        xnorm = -xnorm;
      }

      if ((real32_T)fabs(xnorm) < 9.86076132E-32F) {
        knt = 0;
        do {
          knt++;
          i6 = (ix0 + n) - 2;
          for (k = ix0; k <= i6; k++) {
            x[k - 1] *= 1.01412048E+31F;
          }

          xnorm *= 1.01412048E+31F;
          *alpha1 *= 1.01412048E+31F;
        } while (!((real32_T)fabs(xnorm) >= 9.86076132E-32F));

        xnorm = eml_xnrm2(n - 1, x, ix0);
        xnorm = rt_hypotf_snf(*alpha1, xnorm);
        if (*alpha1 >= 0.0F) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0F / (*alpha1 - xnorm);
        i6 = (ix0 + n) - 2;
        for (k = ix0; k <= i6; k++) {
          x[k - 1] *= *alpha1;
        }

        for (k = 1; k <= knt; k++) {
          xnorm *= 9.86076132E-32F;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0F / (*alpha1 - xnorm);
        i6 = (ix0 + n) - 2;
        for (k = ix0; k <= i6; k++) {
          x[k - 1] *= *alpha1;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

/*
 * Arguments    : int n
 *                const float x[12]
 *                int ix0
 * Return Type  : float
 */
static float eml_xnrm2(int n, const float x[12], int ix0)
{
  float y;
  float scale;
  int kend;
  int k;
  float absxk;
  float t;
  y = 0.0F;
  if (n < 1) {
  } else if (n == 1) {
    y = (real32_T)fabs(x[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = (real32_T)fabs(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * (real32_T)sqrt(y);
  }

  return y;
}

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_hypotf_snf(float u0, float u1)
{
  float y;
  float a;
  float b;
  a = (real32_T)fabs(u0);
  b = (real32_T)fabs(u1);
  if (a < b) {
    a /= b;
    y = b * (real32_T)sqrt(a * a + 1.0F);
  } else if (a > b) {
    b /= a;
    y = a * (real32_T)sqrt(b * b + 1.0F);
  } else if (rtIsNaNF(b)) {
    y = b;
  } else {
    y = a * 1.41421354F;
  }

  return y;
}

/*
 * Arguments    : const double A[9]
 *                const float B[12]
 *                float y[12]
 * Return Type  : void
 */
void mrdivide(const double A[9], const float B[12], float y[12])
{
  float b_A[12];
  int i2;
  int itemp;
  double b_B[9];
  float tau[3];
  signed char jpvt[4];
  float work[4];
  float vn1[4];
  float vn2[4];
  int k;
  int iy;
  float tol;
  float atmp;
  float absxk;
  float t;
  int i;
  int i_i;
  int ix;
  int pvt;
  int i_ip1;
  int lastv;
  int lastc;
  boolean_T exitg2;
  int32_T exitg1;
  double rankR;
  float Y[12];
  double j;
  for (i2 = 0; i2 < 4; i2++) {
    for (itemp = 0; itemp < 3; itemp++) {
      b_A[itemp + 3 * i2] = B[i2 + (itemp << 2)];
    }
  }

  for (i2 = 0; i2 < 3; i2++) {
    for (itemp = 0; itemp < 3; itemp++) {
      b_B[itemp + 3 * i2] = A[i2 + 3 * itemp];
    }
  }

  for (i2 = 0; i2 < 4; i2++) {
    jpvt[i2] = (signed char)(1 + i2);
    work[i2] = 0.0F;
  }

  k = 1;
  for (iy = 0; iy < 4; iy++) {
    tol = 0.0F;
    atmp = 1.17549435E-38F;
    for (itemp = k; itemp <= k + 2; itemp++) {
      absxk = (real32_T)fabs(b_A[itemp - 1]);
      if (absxk > atmp) {
        t = atmp / absxk;
        tol = 1.0F + tol * t * t;
        atmp = absxk;
      } else {
        t = absxk / atmp;
        tol += t * t;
      }
    }

    tol = atmp * (real32_T)sqrt(tol);
    vn1[iy] = tol;
    vn2[iy] = vn1[iy];
    k += 3;
  }

  for (i = 0; i < 3; i++) {
    i_i = i + i * 3;
    itemp = 1;
    ix = i;
    tol = vn1[i];
    for (k = 2; k <= 4 - i; k++) {
      ix++;
      if (vn1[ix] > tol) {
        itemp = k;
        tol = vn1[ix];
      }
    }

    pvt = (i + itemp) - 1;
    if (pvt + 1 != i + 1) {
      ix = 3 * pvt;
      iy = 3 * i;
      for (k = 0; k < 3; k++) {
        tol = b_A[ix];
        b_A[ix] = b_A[iy];
        b_A[iy] = tol;
        ix++;
        iy++;
      }

      itemp = jpvt[pvt];
      jpvt[pvt] = jpvt[i];
      jpvt[i] = (signed char)itemp;
      vn1[pvt] = vn1[i];
      vn2[pvt] = vn2[i];
    }

    if (i + 1 < 3) {
      atmp = b_A[i_i];
      tau[i] = eml_matlab_zlarfg(3 - i, &atmp, b_A, i_i + 2);
    } else {
      atmp = b_A[i_i];
      tau[2] = 0.0F;
    }

    b_A[i_i] = atmp;
    atmp = b_A[i_i];
    b_A[i_i] = 1.0F;
    i_ip1 = (i + (i + 1) * 3) + 1;
    if (tau[i] != 0.0F) {
      lastv = 3 - i;
      itemp = (i_i - i) + 2;
      while ((lastv > 0) && (b_A[itemp] == 0.0F)) {
        lastv--;
        itemp--;
      }

      lastc = 3 - i;
      exitg2 = false;
      while ((!exitg2) && (lastc > 0)) {
        itemp = i_ip1 + (lastc - 1) * 3;
        k = itemp;
        do {
          exitg1 = 0;
          if (k <= (itemp + lastv) - 1) {
            if (b_A[k - 1] != 0.0F) {
              exitg1 = 1;
            } else {
              k++;
            }
          } else {
            lastc--;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    } else {
      lastv = 0;
      lastc = 0;
    }

    if (lastv > 0) {
      if (lastc == 0) {
      } else {
        for (iy = 1; iy <= lastc; iy++) {
          work[iy - 1] = 0.0F;
        }

        iy = 0;
        i2 = i_ip1 + 3 * (lastc - 1);
        for (pvt = i_ip1; pvt <= i2; pvt += 3) {
          ix = i_i;
          tol = 0.0F;
          itemp = (pvt + lastv) - 1;
          for (k = pvt; k <= itemp; k++) {
            tol += b_A[k - 1] * b_A[ix];
            ix++;
          }

          work[iy] += tol;
          iy++;
        }
      }

      if (-tau[i] == 0.0F) {
      } else {
        itemp = i_ip1 - 1;
        pvt = 0;
        for (iy = 1; iy <= lastc; iy++) {
          if (work[pvt] != 0.0F) {
            tol = work[pvt] * -tau[i];
            ix = i_i;
            i2 = lastv + itemp;
            for (k = itemp; k + 1 <= i2; k++) {
              b_A[k] += b_A[ix] * tol;
              ix++;
            }
          }

          pvt++;
          itemp += 3;
        }
      }
    }

    b_A[i_i] = atmp;
    for (iy = i + 1; iy + 1 < 5; iy++) {
      itemp = (i + 3 * iy) + 1;
      if (vn1[iy] != 0.0F) {
        tol = (real32_T)fabs(b_A[i + 3 * iy]) / vn1[iy];
        tol = 1.0F - tol * tol;
        if (tol < 0.0F) {
          tol = 0.0F;
        }

        atmp = vn1[iy] / vn2[iy];
        atmp = tol * (atmp * atmp);
        if (atmp <= 0.000345266977F) {
          if (i + 1 < 3) {
            tol = 0.0F;
            if (2 - i == 1) {
              tol = (real32_T)fabs(b_A[itemp]);
            } else {
              atmp = 1.17549435E-38F;
              pvt = (itemp - i) + 2;
              while (itemp + 1 <= pvt) {
                absxk = (real32_T)fabs(b_A[itemp]);
                if (absxk > atmp) {
                  t = atmp / absxk;
                  tol = 1.0F + tol * t * t;
                  atmp = absxk;
                } else {
                  t = absxk / atmp;
                  tol += t * t;
                }

                itemp++;
              }

              tol = atmp * (real32_T)sqrt(tol);
            }

            vn1[iy] = tol;
            vn2[iy] = vn1[iy];
          } else {
            vn1[iy] = 0.0F;
            vn2[iy] = 0.0F;
          }
        } else {
          vn1[iy] *= (real32_T)sqrt(tol);
        }
      }
    }
  }

  rankR = 0.0;
  tol = 4.0F * (real32_T)fabs(b_A[0]) * 1.1920929E-7F;
  k = 0;
  while ((k < 3) && (!((real32_T)fabs(b_A[k + 3 * k]) <= tol))) {
    rankR++;
    k++;
  }

  for (i2 = 0; i2 < 12; i2++) {
    Y[i2] = 0.0F;
  }

  for (iy = 0; iy < 3; iy++) {
    if (tau[iy] != 0.0F) {
      for (k = 0; k < 3; k++) {
        tol = (float)b_B[iy + 3 * k];
        for (i = 0; i <= 1 - iy; i++) {
          itemp = (iy + i) + 1;
          tol += b_A[itemp + 3 * iy] * (float)b_B[itemp + 3 * k];
        }

        tol *= tau[iy];
        if (tol != 0.0F) {
          b_B[iy + 3 * k] = (float)b_B[iy + 3 * k] - tol;
          for (i = 0; i <= 1 - iy; i++) {
            itemp = (iy + i) + 1;
            b_B[itemp + 3 * k] = (float)b_B[itemp + 3 * k] - b_A[itemp + 3 * iy]
              * tol;
          }
        }
      }
    }
  }

  for (k = 0; k < 3; k++) {
    for (i = 0; i < (int)rankR; i++) {
      Y[(jpvt[i] + (k << 2)) - 1] = (float)b_B[i + 3 * k];
    }

    for (iy = 0; iy < (int)-(1.0 + (-1.0 - rankR)); iy++) {
      j = rankR + -(double)iy;
      Y[(jpvt[(int)j - 1] + (k << 2)) - 1] /= b_A[((int)j + 3 * ((int)j - 1)) -
        1];
      for (i = 0; i <= (int)j - 2; i++) {
        Y[(jpvt[i] + (k << 2)) - 1] -= Y[(jpvt[(int)j - 1] + (k << 2)) - 1] *
          b_A[i + 3 * ((int)j - 1)];
      }
    }
  }

  for (i2 = 0; i2 < 4; i2++) {
    for (itemp = 0; itemp < 3; itemp++) {
      y[itemp + 3 * i2] = Y[i2 + (itemp << 2)];
    }
  }
}

/*
 * File trailer for mrdivide.c
 *
 * [EOF]
 */
