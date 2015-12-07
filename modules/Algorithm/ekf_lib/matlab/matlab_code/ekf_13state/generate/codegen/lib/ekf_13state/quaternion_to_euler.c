/*
 * File: quaternion_to_euler.c
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

/* Function Declarations */
static float rt_atan2f_snf(float u0, float u1);

/* Function Definitions */

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_atan2f_snf(float u0, float u1)
{
  float y;
  int b_u0;
  int b_u1;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0F) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = (real32_T)atan2((float)b_u0, (float)b_u1);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(float)(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (real32_T)atan2(u0, u1);
  }

  return y;
}

/*
 * Arguments    : signed char is_radian
 *                float q0
 *                float q1
 *                float q2
 *                float q3
 *                float *roll
 *                float *pitch
 *                float *yaw
 * Return Type  : void
 */
void quaternion_to_euler(signed char is_radian, float q0, float q1, float q2,
  float q3, float *roll, float *pitch, float *yaw)
{
  float x;
  *roll = rt_atan2f_snf(2.0F * (q2 * q3 + q0 * q1), ((q0 * q0 - q1 * q1) - q2 *
    q2) + q3 * q3);
  x = (real32_T)asin(2.0F * (q1 * q3 - q0 * q2));
  *pitch = -x;
  *yaw = rt_atan2f_snf(2.0F * (q1 * q2 + q0 * q3), ((q0 * q0 + q1 * q1) - q2 *
    q2) - q3 * q3);
  if (is_radian == 0) {
    *roll = *roll * 180.0F / 3.14159274F;
    *pitch = -x * 180.0F / 3.14159274F;
    *yaw = *yaw * 180.0F / 3.14159274F;
  }
}

/*
 * File trailer for quaternion_to_euler.c
 *
 * [EOF]
 */
