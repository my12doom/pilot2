/*
 * File: init_ekf_matrix.c
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

/* Function Definitions */

/*
 * Arguments    : double Be[3]
 *                double P[169]
 *                double X[13]
 *                double Q[81]
 *                double R[64]
 * Return Type  : void
 */
void init_ekf_matrix(double Be[3], double P[169], double X[13], double Q[81],
                     double R[64])
{
  int i;
  static const double dv0[3] = { 0.8906, 0.0, 0.4547 };

  static const signed char v[13] = { 100, 100, 100, 100, 100, 100, 10, 10, 10,
    10, 100, 100, 100 };

  static const double b_v[9] = { 1.0E-15, 1.0E-15, 1.0E-15, 0.03, 0.03, 0.5,
    6.0E-6, 6.0E-6, 6.0E-6 };

  static const double c_v[8] = { 0.0001, 0.0001, 0.0035, 0.8, 0.8, 0.005, 0.005,
    0.005 };

  for (i = 0; i < 3; i++) {
    Be[i] = dv0[i];
  }

  /* local mageetic unit vector */
  memset(&P[0], 0, 169U * sizeof(double));
  for (i = 0; i < 13; i++) {
    P[i + 13 * i] = v[i];

    /* initial position variance (m^2),initial2 velocity variance (m/s)^2,initial quaternion variance,initial gyro bias variance (rad/s)^2 */
    X[i] = 0.0;
  }

  X[5] = 1.0;

  /* earth gravity */
  /* gyro noise variance,accelerometer noise variance,gyro bias random walk variance  */
  memset(&Q[0], 0, 81U * sizeof(double));
  for (i = 0; i < 9; i++) {
    Q[i + 9 * i] = b_v[i];
  }

  /* noise:Pgps_x,Pgps_y,Pbaro_z,Vgps_x,Vgps_y,Mx,My,Mz */
  memset(&R[0], 0, sizeof(double) << 6);
  for (i = 0; i < 8; i++) {
    R[i + (i << 3)] = c_v[i];
  }
}

/*
 * File trailer for init_ekf_matrix.c
 *
 * [EOF]
 */
