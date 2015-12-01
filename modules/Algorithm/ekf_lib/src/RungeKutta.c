/*
 * File: RungeKutta.c
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
 * 2nd order runge-kutta implement
 * Arguments    : float X[13]
 *                const float U[6]
 *                float dT
 * Return Type  : void
 */
void RungeKutta(float X[13], const float U[6], float dT)
{
  double dv1[13];
  int i8;

  /*  k1=f(X,U); */
  /*  k2=f(X+0.5*dT*k1,U); */
  /*  k3=f(X+0.5*dT*k2,U); */
  /*  k4=f(X+dT*k3,U); */
  /*  X=Xlast+dT*(k1+2*k2+2*k3+k4/6); */
  
  //Just implement 1st order,For acuraccy should use four order RungeKutta
  f(X, U, dv1);
  for (i8 = 0; i8 < 13; i8++) {
    X[i8] += dT * (float)dv1[i8];
  }

  normlise_quaternion(X);
}

/*
 * File trailer for RungeKutta.c
 *
 * [EOF]
 */
