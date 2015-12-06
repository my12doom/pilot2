/*
 * File: init_ekf_matrix.c
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
 * Arguments    : float Be[3]
 *                float P[169]
 *                float X[13]
 *                float Q[81]
 *                float R[64]
 * Return Type  : void
 */
void init_ekf_matrix(float Be[3], float P[169], float X[13], float Q[81],
                     float R[64])
{
 int i;
	static const float dv0[3] = { 0.8906, 0.0, 0.4547 };//This is depend on where u r

  static const float v[13] = { 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100 };  //P

  static const float b_v[9] = { 1.0E-17, 1.0E-17, 1.0E-17, 3.5E-2, 3.5E-2, 0.5,
    8.0E-6, 8.0E-6, 8.0E-6 };

  static const float c_v[8] = { 1.0E-3, 1.0E-3, 3.5E-3, 0.08, 0.08, 0.005,
    0.005, 0.005 };
//  static const float dv0[3] = { 0.8906, 0.0, 0.4547 };//This is depend on where u r

//  static const float v[13] = { 100, 100, 100, 100, 100, 100, 100,
//    100, 100, 100, 100, 100, 100 };  //P

//  static const float b_v[9] = { 1.0E-17, 1.0E-17, 1.0E-17, 3.5E-3, 3.5E-3, 0.5,
//	6.0E-6, 6.0E-6, 6.0E-6 };

//  static const float c_v[8] = { 1, 1, 8E-2, 1, 1, 0.005,
//    0.005, 0.005 };// with gps { 0.001, 0.001, 0.02, 0.8, 0.8, 0.005,0.005, 0.005 };

  for (i = 0; i < 3; i++) {
    Be[i] = dv0[i];
  }

  /* local mageetic unit vector */
  memset(&P[0], 0, 169U * sizeof(float));
  for (i = 0; i < 13; i++) {
    P[i + 13 * i] = v[i];

    /* initial position variance (m^2),initial2 velocity variance (m/s)^2,initial quaternion variance,initial gyro bias variance (rad/s)^2 */
    X[i] = 0.0;
  }

  X[5] = 1.0;

  /* earth gravity */
  /* gyro noise variance,accelerometer noise variance,gyro bias random walk variance  */
  memset(&Q[0], 0, 81U * sizeof(float));
  for (i = 0; i < 9; i++) {
    Q[i + 9 * i] = b_v[i];
  }

  /* noise:Pgps_x,Pgps_y,Pbaro_z,Vgps_x,Vgps_y,Mx,My,Mz */
  memset(&R[0], 0, sizeof(float) << 6);
  for (i = 0; i < 8; i++) {
    R[i + (i << 3)] = c_v[i];
  }
}

/*
 * File trailer for init_ekf_matrix.c
 *
 * [EOF]
 */
