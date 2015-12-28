/*
 * File: ned2body.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 28-Dec-2015 15:54:29
 */

#ifndef __NED2BODY_H__
#define __NED2BODY_H__

/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "ekf_13state_types.h"

/* Function Declarations */
extern void ned2body(const float q_now[4], const float vector_ned[3], float
                     vector_body[3]);

#endif

/*
 * File trailer for ned2body.h
 *
 * [EOF]
 */
