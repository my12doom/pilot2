/* 
 * File: _coder_ekf_13state_api.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 26-Nov-2015 12:02:43 
 */

#ifndef ___CODER_EKF_13STATE_API_H__
#define ___CODER_EKF_13STATE_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */
extern void ekf_13state_initialize(emlrtContext *aContext);
extern void ekf_13state_terminate(void);
extern void ekf_13state_atexit(void);
extern void f_api(const mxArray *prhs[2], const mxArray *plhs[1]);
extern void f(float X[13], float U[6], double Xresult[13]);
extern void h_api(const mxArray *prhs[2], const mxArray *plhs[1]);
extern void h(float X[13], float Be[3], double Y[8]);
extern void init_ekf_matrix_api(const mxArray *plhs[5]);
extern void init_ekf_matrix(double Be[3], double P[169], double X[13], double Q[81], double R[64]);
extern void init_quaternion_by_euler_api(const mxArray * const prhs[3], const mxArray *plhs[4]);
extern void init_quaternion_by_euler(float roll, float pitch, float yaw, float *q0, float *q1, float *q2, float *q3);
extern void INS_Correction_api(const mxArray *prhs[6], const mxArray *plhs[2]);
extern void INS_CovariancePrediction_api(const mxArray *prhs[5], const mxArray *plhs[1]);
extern void INS_SetState_api(const mxArray * const prhs[13], const mxArray *plhs[1]);
extern void INS_SetState(float p_x, float p_y, float p_z, float v_x, float v_y, float v_z, float q0, float q1, float q2, float q3, float gyro_x_bias, float gyro_y_bias, float gyro_z_bias, double X[13]);
extern void INSSetMagNorth_api(const mxArray *prhs[1], const mxArray *plhs[1]);
extern void LinearFG_api(const mxArray *prhs[2], const mxArray *plhs[2]);
extern void LinearFG(float X[13], float U[6], double F[169], double G[117]);
extern void LinearizeH_api(const mxArray *prhs[2], const mxArray *plhs[1]);
extern void LinearizeH(float X[13], float Be[3], double H[104]);
extern void normlise_quaternion_api(const mxArray *prhs[1], const mxArray *plhs[1]);
extern void quaternion_to_euler_api(const mxArray * const prhs[5], const mxArray *plhs[3]);
extern void quaternion_to_euler(signed char is_radian, float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw);
extern void RungeKutta_api(const mxArray *prhs[3], const mxArray *plhs[1]);
extern void SerialUpdate_api(const mxArray *prhs[6], const mxArray *plhs[2]);
extern void ekf_13state_xil_terminate(void);
extern void INS_Correction(float Mag_data[3], float Pos[3], float Vel[2], float X[13], float R[64], float P[169]);
extern void INS_CovariancePrediction(float F[169], float G[117], float Q[81], float dT, float P[169]);
extern void INSSetMagNorth(float Be[3]);
extern void normlise_quaternion(float X[13]);
extern void RungeKutta(float X[13], float U[6], float dT);
extern void SerialUpdate(float H[104], float R[64], float Z[8], float Y[8], float P[169], float X[13]);

#endif
/* 
 * File trailer for _coder_ekf_13state_api.h 
 *  
 * [EOF] 
 */
