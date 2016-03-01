/*
 * File: init_quaternion_by_euler.c
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
 * Arguments    : float roll
 *                float pitch
 *                float yaw
 *                float *q0
 *                float *q1
 *                float *q2
 *                float *q3
 * Return Type  : void
 */
void init_quaternion_by_euler(float roll, float pitch, float yaw, float *q0,
  float *q1, float *q2, float *q3)
{
	float sq;
  roll = roll * 3.14159274F / 180.0F;
  pitch = pitch * 3.14159274F / 180.0F;
  yaw = yaw * 3.14159274F / 180.0F;
  *q0 = (real32_T)cos(yaw / 2.0F) * (real32_T)cos(pitch / 2.0F) * (real32_T)cos
    (roll / 2.0F) + (real32_T)sin(yaw / 2.0F) * (real32_T)sin(pitch / 2.0F) *
    (real32_T)sin(roll / 2.0F);
  *q1 = (real32_T)cos(yaw / 2.0F) * (real32_T)cos(pitch / 2.0F) * (real32_T)sin
    (roll / 2.0F) - (real32_T)sin(yaw / 2.0F) * (real32_T)sin(pitch / 2.0F) *
    (real32_T)cos(roll / 2.0F);
  *q2 = (real32_T)cos(yaw / 2.0F) * (real32_T)sin(pitch / 2.0F) * (real32_T)cos
    (roll / 2.0F) + (real32_T)sin(yaw / 2.0F) * (real32_T)cos(pitch / 2.0F) *
    (real32_T)sin(roll / 2.0F);
  *q3 = (real32_T)sin(yaw / 2.0F) * (real32_T)cos(pitch / 2.0F) * (real32_T)cos
    (roll / 2.0F) - (real32_T)cos(yaw / 2.0F) * (real32_T)sin(pitch / 2.0F) *
    (real32_T)sin(roll / 2.0F);

  // normalise
  sq = 1.0f/sqrt(*q0**q0 + *q1**q1 + *q2**q2 + *q3**q3);
	
  *q0 *= sq;
  *q1 *= sq;
  *q2 *= sq;
  *q3 *= sq;
}

/*
 * File trailer for init_quaternion_by_euler.c
 *
 * [EOF]
 */
