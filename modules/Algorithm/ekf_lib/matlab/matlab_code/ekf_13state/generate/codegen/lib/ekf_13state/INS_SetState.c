/*
 * File: INS_SetState.c
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

/* Function Definitions */

/*
 * Arguments    : float p_x
 *                float p_y
 *                float p_z
 *                float v_x
 *                float v_y
 *                float v_z
 *                float q0
 *                float q1
 *                float q2
 *                float q3
 *                float gyro_x_bias
 *                float gyro_y_bias
 *                float gyro_z_bias
 *                double X[13]
 * Return Type  : void
 */
void INS_SetState(float p_x, float p_y, float p_z, float v_x, float v_y, float
                  v_z, float q0, float q1, float q2, float q3, float gyro_x_bias,
                  float gyro_y_bias, float gyro_z_bias, double X[13])
{
  X[0] = p_x;
  X[1] = p_y;
  X[2] = p_z;
  X[3] = v_x;
  X[4] = v_y;
  X[5] = v_z;
  X[6] = q0;
  X[7] = q1;
  X[8] = q2;
  X[9] = q3;
  X[10] = gyro_x_bias;
  X[11] = gyro_y_bias;
  X[12] = gyro_z_bias;
}

/*
 * File trailer for INS_SetState.c
 *
 * [EOF]
 */
