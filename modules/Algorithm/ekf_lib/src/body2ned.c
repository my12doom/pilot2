/*
 * File: body2ned.c
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

/* Function Definitions */

/*
 * Arguments    : const float q_now[4]
 *                const float vector_body[3]
 *                float vector_ned[3]
 * Return Type  : void
 */
void body2ned(const float q_now[4], const float vector_body[3], float
              vector_ned[3])
{
  float b_q_now[9];
  int i0;
  int i1;
  b_q_now[0] = ((q_now[0] * q_now[0] + q_now[1] * q_now[1]) - q_now[2] * q_now[2])
    - q_now[3] * q_now[3];
  b_q_now[1] = 2.0F * (q_now[1] * q_now[2] + q_now[0] * q_now[3]);
  b_q_now[2] = 2.0F * (q_now[1] * q_now[3] - q_now[0] * q_now[2]);
  b_q_now[3] = 2.0F * (q_now[1] * q_now[2] - q_now[0] * q_now[3]);
  b_q_now[4] = ((q_now[0] * q_now[0] - q_now[1] * q_now[1]) + q_now[2] * q_now[2])
    - q_now[3] * q_now[3];
  b_q_now[5] = 2.0F * (q_now[2] * q_now[3] + q_now[0] * q_now[1]);
  b_q_now[6] = 2.0F * (q_now[1] * q_now[3] + q_now[0] * q_now[2]);
  b_q_now[7] = 2.0F * (q_now[2] * q_now[3] - q_now[0] * q_now[1]);
  b_q_now[8] = ((q_now[0] * q_now[0] - q_now[1] * q_now[1]) - q_now[2] * q_now[2])
    + q_now[3] * q_now[3];
  for (i0 = 0; i0 < 3; i0++) {
    vector_ned[i0] = 0.0F;
    for (i1 = 0; i1 < 3; i1++) {
      vector_ned[i0] += b_q_now[i0 + 3 * i1] * vector_body[i1];
    }
  }
}

/*
 * File trailer for body2ned.c
 *
 * [EOF]
 */
