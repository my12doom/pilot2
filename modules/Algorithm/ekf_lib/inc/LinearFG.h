/*
 * File: LinearFG.h
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 26-Nov-2015 12:02:43
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


#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void LinearFG(const float X[13], const float U[6], float F[169], float
                     G[117]);



#ifdef __cplusplus
}
#endif

#endif

/*
 * File trailer for LinearFG.h
 *
 * [EOF]
 */
