/*
 * File: LinearFG.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 03-Dec-2015 17:00:31
 */

#ifndef __LINEARFG_H__
#define __LINEARFG_H__

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
extern void LinearFG(const float X[13], const float U[6], double F[169], double
                     G[117]);

#endif

/*
 * File trailer for LinearFG.h
 *
 * [EOF]
 */
