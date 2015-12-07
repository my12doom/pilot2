/*
 * File: caculate_delta.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 07-Dec-2015 19:18:57
 */

/* Include files */
#include "rt_nonfinite.h"
#include "caculate_delta.h"

/* Function Declarations */
static float b_eml_div(double x, float y);
static void b_eml_xaxpy(int n, float a, const float x[12], int ix0, float y[4],
  int iy0);
static float b_eml_xdotc(int n, const float x[9], int ix0, const float y[9], int
  iy0);
static float b_eml_xnrm2(int n, const float x[3], int ix0);
static void b_eml_xrot(float x[12], int ix0, int iy0, float c, float s);
static void b_eml_xscal(int n, float a, float x[3], int ix0);
static void b_eml_xswap(float x[12], int ix0, int iy0);
static float c_eml_div(float x, float y);
static void c_eml_xaxpy(int n, float a, const float x[4], int ix0, float y[12],
  int iy0);
static void c_eml_xscal(float a, float x[12], int ix0);
static void d_eml_xaxpy(int n, float a, int ix0, float y[9], int iy0);
static void d_eml_xscal(float a, float x[9], int ix0);
static float eml_div(float x, double y);
static void eml_xaxpy(int n, float a, int ix0, float y[12], int iy0);
static float eml_xdotc(int n, const float x[12], int ix0, const float y[12], int
  iy0);
static void eml_xgesvd(const float A[12], float U[12], float S[3], float V[9]);
static float eml_xnrm2(int n, const float x[12], int ix0);
static void eml_xrot(float x[9], int ix0, int iy0, float c, float s);
static void eml_xrotg(float *a, float *b, float *c, float *s);
static void eml_xscal(int n, float a, float x[12], int ix0);
static void eml_xswap(float x[9], int ix0, int iy0);

/* Function Definitions */

/*
 * Arguments    : double x
 *                float y
 * Return Type  : float
 */
static float b_eml_div(double x, float y)
{
  return (float)x / y;
}

/*
 * Arguments    : int n
 *                float a
 *                const float x[12]
 *                int ix0
 *                float y[4]
 *                int iy0
 * Return Type  : void
 */
static void b_eml_xaxpy(int n, float a, const float x[12], int ix0, float y[4],
  int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/*
 * Arguments    : int n
 *                const float x[9]
 *                int ix0
 *                const float y[9]
 *                int iy0
 * Return Type  : float
 */
static float b_eml_xdotc(int n, const float x[9], int ix0, const float y[9], int
  iy0)
{
  float d;
  int ix;
  int iy;
  int k;
  d = 0.0F;
  if (n < 1) {
  } else {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

/*
 * Arguments    : int n
 *                const float x[3]
 *                int ix0
 * Return Type  : float
 */
static float b_eml_xnrm2(int n, const float x[3], int ix0)
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
 * Arguments    : float x[12]
 *                int ix0
 *                int iy0
 *                float c
 *                float s
 * Return Type  : void
 */
static void b_eml_xrot(float x[12], int ix0, int iy0, float c, float s)
{
  int ix;
  int iy;
  int k;
  float temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 4; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
  }
}

/*
 * Arguments    : int n
 *                float a
 *                float x[3]
 *                int ix0
 * Return Type  : void
 */
static void b_eml_xscal(int n, float a, float x[3], int ix0)
{
  int i2;
  int k;
  i2 = (ix0 + n) - 1;
  for (k = ix0; k <= i2; k++) {
    x[k - 1] *= a;
  }
}

/*
 * Arguments    : float x[12]
 *                int ix0
 *                int iy0
 * Return Type  : void
 */
static void b_eml_xswap(float x[12], int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  float temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 4; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

/*
 * Arguments    : float x
 *                float y
 * Return Type  : float
 */
static float c_eml_div(float x, float y)
{
  return x / y;
}

/*
 * Arguments    : int n
 *                float a
 *                const float x[4]
 *                int ix0
 *                float y[12]
 *                int iy0
 * Return Type  : void
 */
static void c_eml_xaxpy(int n, float a, const float x[4], int ix0, float y[12],
  int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/*
 * Arguments    : float a
 *                float x[12]
 *                int ix0
 * Return Type  : void
 */
static void c_eml_xscal(float a, float x[12], int ix0)
{
  int k;
  for (k = ix0; k <= ix0 + 3; k++) {
    x[k - 1] *= a;
  }
}

/*
 * Arguments    : int n
 *                float a
 *                int ix0
 *                float y[9]
 *                int iy0
 * Return Type  : void
 */
static void d_eml_xaxpy(int n, float a, int ix0, float y[9], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

/*
 * Arguments    : float a
 *                float x[9]
 *                int ix0
 * Return Type  : void
 */
static void d_eml_xscal(float a, float x[9], int ix0)
{
  int k;
  for (k = ix0; k <= ix0 + 2; k++) {
    x[k - 1] *= a;
  }
}

/*
 * Arguments    : float x
 *                double y
 * Return Type  : float
 */
static float eml_div(float x, double y)
{
  return x / (float)y;
}

/*
 * Arguments    : int n
 *                float a
 *                int ix0
 *                float y[12]
 *                int iy0
 * Return Type  : void
 */
static void eml_xaxpy(int n, float a, int ix0, float y[12], int iy0)
{
  int ix;
  int iy;
  int k;
  if ((n < 1) || (a == 0.0F)) {
  } else {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

/*
 * Arguments    : int n
 *                const float x[12]
 *                int ix0
 *                const float y[12]
 *                int iy0
 * Return Type  : float
 */
static float eml_xdotc(int n, const float x[12], int ix0, const float y[12], int
  iy0)
{
  float d;
  int ix;
  int iy;
  int k;
  d = 0.0F;
  if (n < 1) {
  } else {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

/*
 * Arguments    : const float A[12]
 *                float U[12]
 *                float S[3]
 *                float V[9]
 * Return Type  : void
 */
static void eml_xgesvd(const float A[12], float U[12], float S[3], float V[9])
{
  float b_A[12];
  int i;
  float s[3];
  float e[3];
  float work[4];
  float Vf[9];
  int q;
  int qs;
  float ztest0;
  int ii;
  int m;
  float rt;
  float ztest;
  int iter;
  float tiny;
  float snorm;
  int32_T exitg3;
  boolean_T exitg2;
  float f;
  float varargin_1[5];
  float mtmp;
  boolean_T exitg1;
  float sqds;
  for (i = 0; i < 12; i++) {
    b_A[i] = A[i];
  }

  for (i = 0; i < 3; i++) {
    s[i] = 0.0F;
    e[i] = 0.0F;
  }

  for (i = 0; i < 4; i++) {
    work[i] = 0.0F;
  }

  for (i = 0; i < 12; i++) {
    U[i] = 0.0F;
  }

  for (i = 0; i < 9; i++) {
    Vf[i] = 0.0F;
  }

  for (q = 0; q < 3; q++) {
    qs = q + (q << 2);
    ztest0 = eml_xnrm2(4 - q, b_A, qs + 1);
    if (ztest0 > 0.0F) {
      if (b_A[qs] < 0.0F) {
        s[q] = -ztest0;
      } else {
        s[q] = ztest0;
      }

      eml_xscal(4 - q, b_eml_div(1.0, s[q]), b_A, qs + 1);
      b_A[qs]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0F;
    }

    for (ii = q + 1; ii + 1 < 4; ii++) {
      i = q + (ii << 2);
      if (s[q] != 0.0F) {
        eml_xaxpy(4 - q, -c_eml_div(eml_xdotc(4 - q, b_A, qs + 1, b_A, i + 1),
                   b_A[q + (q << 2)]), qs + 1, b_A, i + 1);
      }

      e[ii] = b_A[i];
    }

    for (ii = q; ii + 1 < 5; ii++) {
      U[ii + (q << 2)] = b_A[ii + (q << 2)];
    }

    if (q + 1 <= 1) {
      ztest0 = b_eml_xnrm2(2, e, 2);
      if (ztest0 == 0.0F) {
        e[0] = 0.0F;
      } else {
        if (e[1] < 0.0F) {
          e[0] = -ztest0;
        } else {
          e[0] = ztest0;
        }

        b_eml_xscal(2, b_eml_div(1.0, e[0]), e, 2);
        e[1]++;
      }

      e[0] = -e[0];
      if (e[0] != 0.0F) {
        for (ii = 2; ii < 5; ii++) {
          work[ii - 1] = 0.0F;
        }

        for (ii = 1; ii + 1 < 4; ii++) {
          b_eml_xaxpy(3, e[ii], b_A, 2 + (ii << 2), work, 2);
        }

        for (ii = 1; ii + 1 < 4; ii++) {
          c_eml_xaxpy(3, c_eml_div(-e[ii], e[1]), work, 2, b_A, 2 + (ii << 2));
        }
      }

      for (ii = 1; ii + 1 < 4; ii++) {
        Vf[ii] = e[ii];
      }
    }
  }

  m = 1;
  e[1] = b_A[9];
  e[2] = 0.0F;
  for (q = 2; q > -1; q += -1) {
    qs = q + (q << 2);
    if (s[q] != 0.0F) {
      for (ii = q + 1; ii + 1 < 4; ii++) {
        i = (q + (ii << 2)) + 1;
        eml_xaxpy(4 - q, -c_eml_div(eml_xdotc(4 - q, U, qs + 1, U, i), U[qs]),
                  qs + 1, U, i);
      }

      for (ii = q; ii + 1 < 5; ii++) {
        U[ii + (q << 2)] = -U[ii + (q << 2)];
      }

      U[qs]++;
      for (ii = 1; ii <= q; ii++) {
        U[(ii + (q << 2)) - 1] = 0.0F;
      }
    } else {
      for (ii = 0; ii < 4; ii++) {
        U[ii + (q << 2)] = 0.0F;
      }

      U[qs] = 1.0F;
    }
  }

  for (q = 2; q > -1; q += -1) {
    if ((q + 1 <= 1) && (e[0] != 0.0F)) {
      for (ii = 2; ii < 4; ii++) {
        i = 2 + 3 * (ii - 1);
        d_eml_xaxpy(2, -c_eml_div(b_eml_xdotc(2, Vf, 2, Vf, i), Vf[1]), 2, Vf, i);
      }
    }

    for (ii = 0; ii < 3; ii++) {
      Vf[ii + 3 * q] = 0.0F;
    }

    Vf[q + 3 * q] = 1.0F;
  }

  for (q = 0; q < 3; q++) {
    ztest0 = e[q];
    if (s[q] != 0.0F) {
      rt = (real32_T)fabs(s[q]);
      ztest = c_eml_div(s[q], rt);
      s[q] = rt;
      if (q + 1 < 3) {
        ztest0 = c_eml_div(e[q], ztest);
      }

      c_eml_xscal(ztest, U, (q << 2) + 1);
    }

    if ((q + 1 < 3) && (ztest0 != 0.0F)) {
      rt = (real32_T)fabs(ztest0);
      ztest = c_eml_div(rt, ztest0);
      ztest0 = rt;
      s[q + 1] *= ztest;
      d_eml_xscal(ztest, Vf, 3 * (q + 1) + 1);
    }

    e[q] = ztest0;
  }

  iter = 0;
  tiny = c_eml_div(1.17549435E-38F, 1.1920929E-7F);
  snorm = 0.0F;
  for (ii = 0; ii < 3; ii++) {
    ztest0 = (real32_T)fabs(s[ii]);
    ztest = (real32_T)fabs(e[ii]);
    if ((ztest0 >= ztest) || rtIsNaNF(ztest)) {
    } else {
      ztest0 = ztest;
    }

    if ((snorm >= ztest0) || rtIsNaNF(ztest0)) {
    } else {
      snorm = ztest0;
    }
  }

  while ((m + 2 > 0) && (!(iter >= 75))) {
    ii = m;
    do {
      exitg3 = 0;
      q = ii + 1;
      if (ii + 1 == 0) {
        exitg3 = 1;
      } else {
        ztest0 = (real32_T)fabs(e[ii]);
        if ((ztest0 <= 1.1920929E-7F * ((real32_T)fabs(s[ii]) + (real32_T)fabs
              (s[ii + 1]))) || (ztest0 <= tiny) || ((iter > 20) && (ztest0 <=
              1.1920929E-7F * snorm))) {
          e[ii] = 0.0F;
          exitg3 = 1;
        } else {
          ii--;
        }
      }
    } while (exitg3 == 0);

    if (ii + 1 == m + 1) {
      i = 4;
    } else {
      qs = m + 2;
      i = m + 2;
      exitg2 = false;
      while ((!exitg2) && (i >= ii + 1)) {
        qs = i;
        if (i == ii + 1) {
          exitg2 = true;
        } else {
          ztest0 = 0.0F;
          if (i < m + 2) {
            ztest0 = (real32_T)fabs(e[i - 1]);
          }

          if (i > ii + 2) {
            ztest0 += (real32_T)fabs(e[i - 2]);
          }

          ztest = (real32_T)fabs(s[i - 1]);
          if ((ztest <= 1.1920929E-7F * ztest0) || (ztest <= tiny)) {
            s[i - 1] = 0.0F;
            exitg2 = true;
          } else {
            i--;
          }
        }
      }

      if (qs == ii + 1) {
        i = 3;
      } else if (qs == m + 2) {
        i = 1;
      } else {
        i = 2;
        q = qs;
      }
    }

    switch (i) {
     case 1:
      f = e[m];
      e[m] = 0.0F;
      for (ii = m; ii + 1 >= q + 1; ii--) {
        ztest0 = s[ii];
        eml_xrotg(&ztest0, &f, &ztest, &rt);
        s[ii] = ztest0;
        if (ii + 1 > q + 1) {
          f = -rt * e[0];
          e[0] *= ztest;
        }

        eml_xrot(Vf, 3 * ii + 1, 3 * (m + 1) + 1, ztest, rt);
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0F;
      for (ii = q; ii + 1 <= m + 2; ii++) {
        eml_xrotg(&s[ii], &f, &ztest, &rt);
        f = -rt * e[ii];
        e[ii] *= ztest;
        b_eml_xrot(U, (ii << 2) + 1, ((q - 1) << 2) + 1, ztest, rt);
      }
      break;

     case 3:
      varargin_1[0] = (real32_T)fabs(s[m + 1]);
      varargin_1[1] = (real32_T)fabs(s[m]);
      varargin_1[2] = (real32_T)fabs(e[m]);
      varargin_1[3] = (real32_T)fabs(s[q]);
      varargin_1[4] = (real32_T)fabs(e[q]);
      i = 1;
      mtmp = varargin_1[0];
      if (rtIsNaNF(varargin_1[0])) {
        ii = 2;
        exitg1 = false;
        while ((!exitg1) && (ii < 6)) {
          i = ii;
          if (!rtIsNaNF(varargin_1[ii - 1])) {
            mtmp = varargin_1[ii - 1];
            exitg1 = true;
          } else {
            ii++;
          }
        }
      }

      if (i < 5) {
        while (i + 1 < 6) {
          if (varargin_1[i] > mtmp) {
            mtmp = varargin_1[i];
          }

          i++;
        }
      }

      f = c_eml_div(s[m + 1], mtmp);
      ztest0 = c_eml_div(s[m], mtmp);
      ztest = c_eml_div(e[m], mtmp);
      sqds = c_eml_div(s[q], mtmp);
      rt = eml_div((ztest0 + f) * (ztest0 - f) + ztest * ztest, 2.0);
      ztest0 = f * ztest;
      ztest0 *= ztest0;
      ztest = 0.0F;
      if ((rt != 0.0F) || (ztest0 != 0.0F)) {
        ztest = (real32_T)sqrt(rt * rt + ztest0);
        if (rt < 0.0F) {
          ztest = -ztest;
        }

        ztest = c_eml_div(ztest0, rt + ztest);
      }

      f = (sqds + f) * (sqds - f) + ztest;
      ztest0 = sqds * c_eml_div(e[q], mtmp);
      for (ii = q + 1; ii <= m + 1; ii++) {
        eml_xrotg(&f, &ztest0, &ztest, &rt);
        if (ii > q + 1) {
          e[0] = f;
        }

        f = ztest * s[ii - 1] + rt * e[ii - 1];
        e[ii - 1] = ztest * e[ii - 1] - rt * s[ii - 1];
        ztest0 = rt * s[ii];
        s[ii] *= ztest;
        eml_xrot(Vf, 3 * (ii - 1) + 1, 3 * ii + 1, ztest, rt);
        s[ii - 1] = f;
        eml_xrotg(&s[ii - 1], &ztest0, &ztest, &rt);
        f = ztest * e[ii - 1] + rt * s[ii];
        s[ii] = -rt * e[ii - 1] + ztest * s[ii];
        ztest0 = rt * e[ii];
        e[ii] *= ztest;
        b_eml_xrot(U, ((ii - 1) << 2) + 1, (ii << 2) + 1, ztest, rt);
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0F) {
        s[q] = -s[q];
        d_eml_xscal(-1.0F, Vf, 3 * q + 1);
      }

      i = q + 1;
      while ((q + 1 < 3) && (s[q] < s[i])) {
        rt = s[q];
        s[q] = s[i];
        s[i] = rt;
        eml_xswap(Vf, 3 * q + 1, 3 * (q + 1) + 1);
        b_eml_xswap(U, (q << 2) + 1, ((q + 1) << 2) + 1);
        q = i;
        i++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  for (ii = 0; ii < 3; ii++) {
    S[ii] = s[ii];
    for (i = 0; i < 3; i++) {
      V[i + 3 * ii] = Vf[i + 3 * ii];
    }
  }
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
 * Arguments    : float x[9]
 *                int ix0
 *                int iy0
 *                float c
 *                float s
 * Return Type  : void
 */
static void eml_xrot(float x[9], int ix0, int iy0, float c, float s)
{
  int ix;
  int iy;
  int k;
  float temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 3; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
  }
}

/*
 * Arguments    : float *a
 *                float *b
 *                float *c
 *                float *s
 * Return Type  : void
 */
static void eml_xrotg(float *a, float *b, float *c, float *s)
{
  float roe;
  float absa;
  float absb;
  float scale;
  float ads;
  float bds;
  roe = *b;
  absa = (real32_T)fabs(*a);
  absb = (real32_T)fabs(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0F) {
    *s = 0.0F;
    *c = 1.0F;
    ads = 0.0F;
    scale = 0.0F;
  } else {
    ads = absa / scale;
    bds = absb / scale;
    ads = scale * (real32_T)sqrt(ads * ads + bds * bds);
    if (roe < 0.0F) {
      ads = -ads;
    }

    *c = *a / ads;
    *s = *b / ads;
    if (absa > absb) {
      scale = *s;
    } else if (*c != 0.0F) {
      scale = 1.0F / *c;
    } else {
      scale = 1.0F;
    }
  }

  *a = ads;
  *b = scale;
}

/*
 * Arguments    : int n
 *                float a
 *                float x[12]
 *                int ix0
 * Return Type  : void
 */
static void eml_xscal(int n, float a, float x[12], int ix0)
{
  int i1;
  int k;
  i1 = (ix0 + n) - 1;
  for (k = ix0; k <= i1; k++) {
    x[k - 1] *= a;
  }
}

/*
 * Arguments    : float x[9]
 *                int ix0
 *                int iy0
 * Return Type  : void
 */
static void eml_xswap(float x[9], int ix0, int iy0)
{
  int ix;
  int iy;
  int k;
  float temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 3; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

/*
 * Arguments    : const float q_target[4]
 *                const float q_now[4]
 *                float dt
 *                float delt_angle[3]
 * Return Type  : void
 */
void caculate_delta(const float q_target[4], const float q_now[4], float dt,
                    float delt_angle[3])
{
  float X[12];
  int i0;
  float b_q_now[12];
  float fv0[12];
  int k;
  float V[9];
  float s[3];
  float S[9];
  float tol;
  int r;
  int vcol;
  int ar;
  int ic;
  int ib;
  int ia;
  float b_q_target[4];
  (void)dt;

  /* delt_q=normlise_quaternion(delt_q); */
  for (i0 = 0; i0 < 12; i0++) {
    X[i0] = 0.0F;
  }

  b_q_now[0] = -q_now[1];
  b_q_now[4] = -q_now[2];
  b_q_now[8] = -q_now[3];
  b_q_now[1] = q_now[0];
  b_q_now[5] = -q_now[3];
  b_q_now[9] = q_now[2];
  b_q_now[2] = q_now[3];
  b_q_now[6] = q_now[0];
  b_q_now[10] = -q_now[1];
  b_q_now[3] = -q_now[2];
  b_q_now[7] = q_now[1];
  b_q_now[11] = q_now[0];
  for (i0 = 0; i0 < 3; i0++) {
    for (k = 0; k < 4; k++) {
      fv0[k + (i0 << 2)] = 0.5F * b_q_now[k + (i0 << 2)];
    }
  }

  eml_xgesvd(fv0, b_q_now, s, V);
  for (i0 = 0; i0 < 9; i0++) {
    S[i0] = 0.0F;
  }

  for (k = 0; k < 3; k++) {
    S[k + 3 * k] = s[k];
  }

  tol = 4.0F * S[0] * 1.1920929E-7F;
  r = 0;
  k = 0;
  while ((k + 1 < 4) && (S[k + 3 * k] > tol)) {
    r++;
    k++;
  }

  if (r > 0) {
    vcol = 0;
    for (ar = 0; ar + 1 <= r; ar++) {
      tol = 1.0F / S[ar + 3 * ar];
      for (k = vcol; k + 1 <= vcol + 3; k++) {
        V[k] *= tol;
      }

      vcol += 3;
    }

    for (k = 0; k < 11; k += 3) {
      for (ic = k; ic + 1 <= k + 3; ic++) {
        X[ic] = 0.0F;
      }
    }

    vcol = -1;
    for (k = 0; k < 11; k += 3) {
      ar = 0;
      vcol++;
      i0 = (vcol + ((r - 1) << 2)) + 1;
      for (ib = vcol; ib + 1 <= i0; ib += 4) {
        if (b_q_now[ib] != 0.0F) {
          ia = ar;
          for (ic = k; ic + 1 <= k + 3; ic++) {
            ia++;
            X[ic] += b_q_now[ib] * V[ia - 1];
          }
        }

        ar += 3;
      }
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    b_q_target[i0] = q_target[i0] - q_now[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    delt_angle[i0] = 0.0F;
    for (k = 0; k < 4; k++) {
      delt_angle[i0] += X[i0 + 3 * k] * b_q_target[k];
    }
  }
}

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void caculate_delta_terminate(void)
{
  /* (no terminate code required) */
}

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void caculate_delta_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * File trailer for caculate_delta_terminate.c
 *
 * [EOF]
 */

/*
 * File trailer for caculate_delta.c
 *
 * [EOF]
 */
