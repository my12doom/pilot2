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
 * 1nd order runge-kutta implement
 * Arguments    : float X[13]
 *                const float U[6]
 *                float dT
 * Return Type  : void
 */
//void RungeKutta(float X[13], const float U[6], float dT)
//{
//  double dv1[13];
//  int i8;

//  /*  k1=f(X,U); */
//  /*  k2=f(X+0.5*dT*k1,U); */
//  /*  k3=f(X+0.5*dT*k2,U); */
//  /*  k4=f(X+dT*k3,U); */
//  /*  X=Xlast+dT*(k1+2*k2+2*k3+k4/6); */
//  
//  //Just implement 1st order,For acuraccy should use four order RungeKutta
//  f(X, U, dv1);
//  for (i8 = 0; i8 < 13; i8++) {
//    X[i8] += dT * (float)dv1[i8];
//  }

//  normlise_quaternion(X);
//}


/* Function Definitions */
/*
 * 4nd order runge-kutta implement
 * Arguments    : float X[13]
 *                const float U[6]
 *                float dT
 * Return Type  : void
 */
void RungeKutta(float X[13], const float U[6], float dT)
{
  float Xlast[13];
  int i;
  double k1[13];
  float y;
  float b_X[13];
  double k2[13];
  float b_y;
  double k3[13];
  double dv2[13];
  for (i = 0; i < 13; i++) {
    Xlast[i] = X[i];
  }

  f(X, U, k1);
  y = 0.5F * dT;
  for (i = 0; i < 13; i++) {
    X[i] += y * (float)k1[i];
  }

  y = 0.5F * dT;
  for (i = 0; i < 13; i++) {
    b_X[i] = X[i] + y * (float)k1[i];
  }

  f(b_X, U, k2);
  y = 0.5F * dT;
  b_y = 0.5F * dT;
  for (i = 0; i < 13; i++) {
    b_X[i] = (Xlast[i] + y * (float)k2[i]) + b_y * (float)k2[i];
  }

  f(b_X, U, k3);
  y = 0.5F * dT;

  /*  X=Xlast+dT*f(X,U); */
  for (i = 0; i < 13; i++) {
    b_X[i] = (Xlast[i] + y * (float)k3[i]) + dT * (float)k3[i];
  }

  f(b_X, U, dv2);
  for (i = 0; i < 13; i++) {
    X[i] = Xlast[i] + dT * (float)(((k1[i] + 2.0 * k2[i]) + 2.0 * k3[i]) + dv2[i])
      / 6.0F;
  }

  normlise_quaternion(X);
}

/*
 * File trailer for RungeKutta.c
 *
 * [EOF]
 */
