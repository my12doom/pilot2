/*
 * File: _coder_ekf_13state_api.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 28-Dec-2015 15:54:29
 */

/* Include files */
#include "_coder_ekf_13state_api.h"

/* Function Declarations */
static float (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *q_now,
  const char *identifier))[4];
static float (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[4];
static float (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *vector_body, const char *identifier))[3];
static float (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[3];
static const mxArray *emlrt_marshallOut(const float u[3]);
static float (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *X, const
  char *identifier))[13];
static float (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[13];
static float (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *U, const
  char *identifier))[6];
static float (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[6];
static const mxArray *b_emlrt_marshallOut(const double u[13]);
static const mxArray *c_emlrt_marshallOut(const double u[8]);
static const mxArray *d_emlrt_marshallOut(const double u[3]);
static const mxArray *e_emlrt_marshallOut(const double u[169]);
static const mxArray *f_emlrt_marshallOut(const double u[81]);
static const mxArray *g_emlrt_marshallOut(const double u[64]);
static float i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *roll, const
  char *identifier);
static float j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static const mxArray *h_emlrt_marshallOut(const float u);
static float (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *R, const
  char *identifier))[64];
static float (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[64];
static float (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *P, const
  char *identifier))[169];
static float (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[169];
static void i_emlrt_marshallOut(const float u[13], const mxArray *y);
static void j_emlrt_marshallOut(const float u[169], const mxArray *y);
static float (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *G, const
  char *identifier))[117];
static float (*p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[117];
static float (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *Q, const
  char *identifier))[81];
static float (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[81];
static void k_emlrt_marshallOut(const float u[3], const mxArray *y);
static const mxArray *l_emlrt_marshallOut(const double u[117]);
static const mxArray *m_emlrt_marshallOut(const double u[104]);
static float (*s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *H, const
  char *identifier))[104];
static float (*t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[104];
static float (*u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *Z, const
  char *identifier))[8];
static float (*v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[8];
static float (*w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4];
static float (*x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3];
static float (*y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[13];
static float (*ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6];
static float bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static float (*cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[64];
static float (*db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[169];
static float (*eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[117];
static float (*fb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[81];
static float (*gb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[104];
static float (*hb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[8];

/* Function Definitions */

/*
 * Arguments    : emlrtContext *aContext
 * Return Type  : void
 */
void ekf_13state_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ekf_13state_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ekf_13state_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  ekf_13state_xil_terminate();
}

/*
 * Arguments    : const mxArray *prhs[2]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void body2ned_api(const mxArray *prhs[2], const mxArray *plhs[1])
{
  float (*vector_ned)[3];
  float (*q_now)[4];
  float (*vector_body)[3];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  vector_ned = (float (*)[3])mxMalloc(sizeof(float [3]));
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);

  /* Marshall function inputs */
  q_now = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "q_now");
  vector_body = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "vector_body");

  /* Invoke the target function */
  body2ned(*q_now, *vector_body, *vector_ned);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*vector_ned);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *q_now
 *                const char *identifier
 * Return Type  : float (*)[4]
 */
static float (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *q_now,
  const char *identifier))[4]
{
  float (*y)[4];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(q_now), &thisId);
  emlrtDestroyArray(&q_now);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float (*)[4]
 */
  static float (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[4]
{
  float (*y)[4];
  y = w_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *vector_body
 *                const char *identifier
 * Return Type  : float (*)[3]
 */
static float (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *vector_body, const char *identifier))[3]
{
  float (*y)[3];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = d_emlrt_marshallIn(sp, emlrtAlias(vector_body), &thisId);
  emlrtDestroyArray(&vector_body);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float (*)[3]
 */
  static float (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[3]
{
  float (*y)[3];
  y = x_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const float u[3]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const float u[3])
{
  const mxArray *y;
  static const int iv0[1] = { 0 };

  const mxArray *m0;
  static const int iv1[1] = { 3 };

  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxSINGLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)u);
  emlrtSetDimensions((mxArray *)m0, iv1, 1);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const mxArray *prhs[2]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void f_api(const mxArray *prhs[2], const mxArray *plhs[1])
{
  double (*Xresult)[13];
  float (*X)[13];
  float (*U)[6];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  Xresult = (double (*)[13])mxMalloc(sizeof(double [13]));
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);

  /* Marshall function inputs */
  X = e_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "X");
  U = g_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "U");

  /* Invoke the target function */
  f(*X, *U, *Xresult);

  /* Marshall function outputs */
  plhs[0] = b_emlrt_marshallOut(*Xresult);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *X
 *                const char *identifier
 * Return Type  : float (*)[13]
 */
static float (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *X, const
  char *identifier))[13]
{
  float (*y)[13];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = f_emlrt_marshallIn(sp, emlrtAlias(X), &thisId);
  emlrtDestroyArray(&X);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float (*)[13]
 */
  static float (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[13]
{
  float (*y)[13];
  y = y_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *U
 *                const char *identifier
 * Return Type  : float (*)[6]
 */
static float (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *U, const
  char *identifier))[6]
{
  float (*y)[6];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = h_emlrt_marshallIn(sp, emlrtAlias(U), &thisId);
  emlrtDestroyArray(&U);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float (*)[6]
 */
  static float (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[6]
{
  float (*y)[6];
  y = ab_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const double u[13]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const double u[13])
{
  const mxArray *y;
  static const int iv2[1] = { 0 };

  const mxArray *m1;
  static const int iv3[1] = { 13 };

  y = NULL;
  m1 = emlrtCreateNumericArray(1, iv2, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m1, (void *)u);
  emlrtSetDimensions((mxArray *)m1, iv3, 1);
  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const mxArray *prhs[2]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void h_api(const mxArray *prhs[2], const mxArray *plhs[1])
{
  double (*Y)[8];
  float (*X)[13];
  float (*Be)[3];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  Y = (double (*)[8])mxMalloc(sizeof(double [8]));
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);

  /* Marshall function inputs */
  X = e_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "X");
  Be = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "Be");

  /* Invoke the target function */
  h(*X, *Be, *Y);

  /* Marshall function outputs */
  plhs[0] = c_emlrt_marshallOut(*Y);
}

/*
 * Arguments    : const double u[8]
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrt_marshallOut(const double u[8])
{
  const mxArray *y;
  static const int iv4[1] = { 0 };

  const mxArray *m2;
  static const int iv5[1] = { 8 };

  y = NULL;
  m2 = emlrtCreateNumericArray(1, iv4, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m2, (void *)u);
  emlrtSetDimensions((mxArray *)m2, iv5, 1);
  emlrtAssign(&y, m2);
  return y;
}

/*
 * Arguments    : const mxArray *plhs[5]
 * Return Type  : void
 */
void init_ekf_matrix_api(const mxArray *plhs[5])
{
  double (*Be)[3];
  double (*P)[169];
  double (*X)[13];
  double (*Q)[81];
  double (*R)[64];
  Be = (double (*)[3])mxMalloc(sizeof(double [3]));
  P = (double (*)[169])mxMalloc(sizeof(double [169]));
  X = (double (*)[13])mxMalloc(sizeof(double [13]));
  Q = (double (*)[81])mxMalloc(sizeof(double [81]));
  R = (double (*)[64])mxMalloc(sizeof(double [64]));

  /* Invoke the target function */
  init_ekf_matrix(*Be, *P, *X, *Q, *R);

  /* Marshall function outputs */
  plhs[0] = d_emlrt_marshallOut(*Be);
  plhs[1] = e_emlrt_marshallOut(*P);
  plhs[2] = b_emlrt_marshallOut(*X);
  plhs[3] = f_emlrt_marshallOut(*Q);
  plhs[4] = g_emlrt_marshallOut(*R);
}

/*
 * Arguments    : const double u[3]
 * Return Type  : const mxArray *
 */
static const mxArray *d_emlrt_marshallOut(const double u[3])
{
  const mxArray *y;
  static const int iv6[1] = { 0 };

  const mxArray *m3;
  static const int iv7[1] = { 3 };

  y = NULL;
  m3 = emlrtCreateNumericArray(1, iv6, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m3, (void *)u);
  emlrtSetDimensions((mxArray *)m3, iv7, 1);
  emlrtAssign(&y, m3);
  return y;
}

/*
 * Arguments    : const double u[169]
 * Return Type  : const mxArray *
 */
static const mxArray *e_emlrt_marshallOut(const double u[169])
{
  const mxArray *y;
  static const int iv8[2] = { 0, 0 };

  const mxArray *m4;
  static const int iv9[2] = { 13, 13 };

  y = NULL;
  m4 = emlrtCreateNumericArray(2, iv8, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m4, (void *)u);
  emlrtSetDimensions((mxArray *)m4, iv9, 2);
  emlrtAssign(&y, m4);
  return y;
}

/*
 * Arguments    : const double u[81]
 * Return Type  : const mxArray *
 */
static const mxArray *f_emlrt_marshallOut(const double u[81])
{
  const mxArray *y;
  static const int iv10[2] = { 0, 0 };

  const mxArray *m5;
  static const int iv11[2] = { 9, 9 };

  y = NULL;
  m5 = emlrtCreateNumericArray(2, iv10, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m5, (void *)u);
  emlrtSetDimensions((mxArray *)m5, iv11, 2);
  emlrtAssign(&y, m5);
  return y;
}

/*
 * Arguments    : const double u[64]
 * Return Type  : const mxArray *
 */
static const mxArray *g_emlrt_marshallOut(const double u[64])
{
  const mxArray *y;
  static const int iv12[2] = { 0, 0 };

  const mxArray *m6;
  static const int iv13[2] = { 8, 8 };

  y = NULL;
  m6 = emlrtCreateNumericArray(2, iv12, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m6, (void *)u);
  emlrtSetDimensions((mxArray *)m6, iv13, 2);
  emlrtAssign(&y, m6);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[3]
 *                const mxArray *plhs[4]
 * Return Type  : void
 */
void init_quaternion_by_euler_api(const mxArray * const prhs[3], const mxArray
  *plhs[4])
{
  float roll;
  float pitch;
  float yaw;
  float q3;
  float q2;
  float q1;
  float q0;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  roll = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "roll");
  pitch = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "pitch");
  yaw = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "yaw");

  /* Invoke the target function */
  init_quaternion_by_euler(roll, pitch, yaw, &q0, &q1, &q2, &q3);

  /* Marshall function outputs */
  plhs[0] = h_emlrt_marshallOut(q0);
  plhs[1] = h_emlrt_marshallOut(q1);
  plhs[2] = h_emlrt_marshallOut(q2);
  plhs[3] = h_emlrt_marshallOut(q3);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *roll
 *                const char *identifier
 * Return Type  : float
 */
static float i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *roll, const
  char *identifier)
{
  float y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = j_emlrt_marshallIn(sp, emlrtAlias(roll), &thisId);
  emlrtDestroyArray(&roll);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float
 */
static float j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  float y;
  y = bb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const float u
 * Return Type  : const mxArray *
 */
static const mxArray *h_emlrt_marshallOut(const float u)
{
  const mxArray *y;
  const mxArray *m7;
  y = NULL;
  m7 = emlrtCreateNumericMatrix(1, 1, mxSINGLE_CLASS, mxREAL);
  *(float *)mxGetData(m7) = u;
  emlrtAssign(&y, m7);
  return y;
}

/*
 * Arguments    : const mxArray *prhs[7]
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void INS_Correction_api(const mxArray *prhs[7], const mxArray *plhs[2])
{
  float (*Mag_data)[3];
  float (*Pos)[3];
  float (*Vel)[3];
  float (*X)[13];
  float (*R)[64];
  float (*P)[169];
  float (*Be)[3];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);
  prhs[2] = emlrtProtectR2012b(prhs[2], 2, false, -1);
  prhs[3] = emlrtProtectR2012b(prhs[3], 3, true, -1);
  prhs[4] = emlrtProtectR2012b(prhs[4], 4, false, -1);
  prhs[5] = emlrtProtectR2012b(prhs[5], 5, true, -1);
  prhs[6] = emlrtProtectR2012b(prhs[6], 6, false, -1);

  /* Marshall function inputs */
  Mag_data = c_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "Mag_data");
  Pos = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "Pos");
  Vel = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "Vel");
  X = e_emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "X");
  R = k_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "R");
  P = m_emlrt_marshallIn(&st, emlrtAlias(prhs[5]), "P");
  Be = c_emlrt_marshallIn(&st, emlrtAlias(prhs[6]), "Be");

  /* Invoke the target function */
  INS_Correction(*Mag_data, *Pos, *Vel, *X, *R, *P, *Be);

  /* Marshall function outputs */
  i_emlrt_marshallOut(*X, prhs[3]);
  plhs[0] = prhs[3];
  j_emlrt_marshallOut(*P, prhs[5]);
  plhs[1] = prhs[5];
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *R
 *                const char *identifier
 * Return Type  : float (*)[64]
 */
static float (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *R, const
  char *identifier))[64]
{
  float (*y)[64];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = l_emlrt_marshallIn(sp, emlrtAlias(R), &thisId);
  emlrtDestroyArray(&R);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float (*)[64]
 */
  static float (*l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[64]
{
  float (*y)[64];
  y = cb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *P
 *                const char *identifier
 * Return Type  : float (*)[169]
 */
static float (*m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *P, const
  char *identifier))[169]
{
  float (*y)[169];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = n_emlrt_marshallIn(sp, emlrtAlias(P), &thisId);
  emlrtDestroyArray(&P);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float (*)[169]
 */
  static float (*n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[169]
{
  float (*y)[169];
  y = db_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const float u[13]
 *                const mxArray *y
 * Return Type  : void
 */
static void i_emlrt_marshallOut(const float u[13], const mxArray *y)
{
  static const int iv14[1] = { 13 };

  mxSetData((mxArray *)y, (void *)u);
  emlrtSetDimensions((mxArray *)y, iv14, 1);
}

/*
 * Arguments    : const float u[169]
 *                const mxArray *y
 * Return Type  : void
 */
static void j_emlrt_marshallOut(const float u[169], const mxArray *y)
{
  static const int iv15[2] = { 13, 13 };

  mxSetData((mxArray *)y, (void *)u);
  emlrtSetDimensions((mxArray *)y, iv15, 2);
}

/*
 * Arguments    : const mxArray *prhs[5]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void INS_CovariancePrediction_api(const mxArray *prhs[5], const mxArray *plhs[1])
{
  float (*F)[169];
  float (*G)[117];
  float (*Q)[81];
  float dT;
  float (*P)[169];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);
  prhs[2] = emlrtProtectR2012b(prhs[2], 2, false, -1);
  prhs[4] = emlrtProtectR2012b(prhs[4], 4, true, -1);

  /* Marshall function inputs */
  F = m_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "F");
  G = o_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "G");
  Q = q_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "Q");
  dT = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "dT");
  P = m_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "P");

  /* Invoke the target function */
  INS_CovariancePrediction(*F, *G, *Q, dT, *P);

  /* Marshall function outputs */
  j_emlrt_marshallOut(*P, prhs[4]);
  plhs[0] = prhs[4];
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *G
 *                const char *identifier
 * Return Type  : float (*)[117]
 */
static float (*o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *G, const
  char *identifier))[117]
{
  float (*y)[117];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = p_emlrt_marshallIn(sp, emlrtAlias(G), &thisId);
  emlrtDestroyArray(&G);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float (*)[117]
 */
  static float (*p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[117]
{
  float (*y)[117];
  y = eb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *Q
 *                const char *identifier
 * Return Type  : float (*)[81]
 */
static float (*q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *Q, const
  char *identifier))[81]
{
  float (*y)[81];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = r_emlrt_marshallIn(sp, emlrtAlias(Q), &thisId);
  emlrtDestroyArray(&Q);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float (*)[81]
 */
  static float (*r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[81]
{
  float (*y)[81];
  y = fb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[13]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void INS_SetState_api(const mxArray * const prhs[13], const mxArray *plhs[1])
{
  double (*X)[13];
  float p_x;
  float p_y;
  float p_z;
  float v_x;
  float v_y;
  float v_z;
  float q0;
  float q1;
  float q2;
  float q3;
  float gyro_x_bias;
  float gyro_y_bias;
  float gyro_z_bias;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  X = (double (*)[13])mxMalloc(sizeof(double [13]));

  /* Marshall function inputs */
  p_x = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "p_x");
  p_y = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "p_y");
  p_z = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "p_z");
  v_x = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "v_x");
  v_y = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "v_y");
  v_z = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "v_z");
  q0 = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "q0");
  q1 = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "q1");
  q2 = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "q2");
  q3 = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[9]), "q3");
  gyro_x_bias = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[10]), "gyro_x_bias");
  gyro_y_bias = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[11]), "gyro_y_bias");
  gyro_z_bias = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[12]), "gyro_z_bias");

  /* Invoke the target function */
  INS_SetState(p_x, p_y, p_z, v_x, v_y, v_z, q0, q1, q2, q3, gyro_x_bias,
               gyro_y_bias, gyro_z_bias, *X);

  /* Marshall function outputs */
  plhs[0] = b_emlrt_marshallOut(*X);
}

/*
 * Arguments    : const mxArray *prhs[1]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void INSSetMagNorth_api(const mxArray *prhs[1], const mxArray *plhs[1])
{
  float (*Be)[3];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, true, -1);

  /* Marshall function inputs */
  Be = c_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "Be");

  /* Invoke the target function */
  INSSetMagNorth(*Be);

  /* Marshall function outputs */
  k_emlrt_marshallOut(*Be, prhs[0]);
  plhs[0] = prhs[0];
}

/*
 * Arguments    : const float u[3]
 *                const mxArray *y
 * Return Type  : void
 */
static void k_emlrt_marshallOut(const float u[3], const mxArray *y)
{
  static const int iv16[1] = { 3 };

  mxSetData((mxArray *)y, (void *)u);
  emlrtSetDimensions((mxArray *)y, iv16, 1);
}

/*
 * Arguments    : const mxArray *prhs[2]
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void LinearFG_api(const mxArray *prhs[2], const mxArray *plhs[2])
{
  double (*F)[169];
  double (*G)[117];
  float (*X)[13];
  float (*U)[6];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  F = (double (*)[169])mxMalloc(sizeof(double [169]));
  G = (double (*)[117])mxMalloc(sizeof(double [117]));
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);

  /* Marshall function inputs */
  X = e_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "X");
  U = g_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "U");

  /* Invoke the target function */
  LinearFG(*X, *U, *F, *G);

  /* Marshall function outputs */
  plhs[0] = e_emlrt_marshallOut(*F);
  plhs[1] = l_emlrt_marshallOut(*G);
}

/*
 * Arguments    : const double u[117]
 * Return Type  : const mxArray *
 */
static const mxArray *l_emlrt_marshallOut(const double u[117])
{
  const mxArray *y;
  static const int iv17[2] = { 0, 0 };

  const mxArray *m8;
  static const int iv18[2] = { 13, 9 };

  y = NULL;
  m8 = emlrtCreateNumericArray(2, iv17, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m8, (void *)u);
  emlrtSetDimensions((mxArray *)m8, iv18, 2);
  emlrtAssign(&y, m8);
  return y;
}

/*
 * Arguments    : const mxArray *prhs[2]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void LinearizeH_api(const mxArray *prhs[2], const mxArray *plhs[1])
{
  double (*H)[104];
  float (*X)[13];
  float (*Be)[3];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  H = (double (*)[104])mxMalloc(sizeof(double [104]));
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);

  /* Marshall function inputs */
  X = e_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "X");
  Be = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "Be");

  /* Invoke the target function */
  LinearizeH(*X, *Be, *H);

  /* Marshall function outputs */
  plhs[0] = m_emlrt_marshallOut(*H);
}

/*
 * Arguments    : const double u[104]
 * Return Type  : const mxArray *
 */
static const mxArray *m_emlrt_marshallOut(const double u[104])
{
  const mxArray *y;
  static const int iv19[2] = { 0, 0 };

  const mxArray *m9;
  static const int iv20[2] = { 8, 13 };

  y = NULL;
  m9 = emlrtCreateNumericArray(2, iv19, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m9, (void *)u);
  emlrtSetDimensions((mxArray *)m9, iv20, 2);
  emlrtAssign(&y, m9);
  return y;
}

/*
 * Arguments    : const mxArray *prhs[2]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void ned2body_api(const mxArray *prhs[2], const mxArray *plhs[1])
{
  float (*vector_body)[3];
  float (*q_now)[4];
  float (*vector_ned)[3];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  vector_body = (float (*)[3])mxMalloc(sizeof(float [3]));
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);

  /* Marshall function inputs */
  q_now = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "q_now");
  vector_ned = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "vector_ned");

  /* Invoke the target function */
  ned2body(*q_now, *vector_ned, *vector_body);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*vector_body);
}

/*
 * Arguments    : const mxArray *prhs[1]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void normlise_quaternion_api(const mxArray *prhs[1], const mxArray *plhs[1])
{
  float (*X)[13];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, true, -1);

  /* Marshall function inputs */
  X = e_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "X");

  /* Invoke the target function */
  normlise_quaternion(*X);

  /* Marshall function outputs */
  i_emlrt_marshallOut(*X, prhs[0]);
  plhs[0] = prhs[0];
}

/*
 * Arguments    : const mxArray * const prhs[5]
 *                const mxArray *plhs[3]
 * Return Type  : void
 */
void quaternion_to_euler_api(const mxArray * const prhs[5], const mxArray *plhs
  [3])
{
  float is_radian;
  float q0;
  float q1;
  float q2;
  float q3;
  float yaw;
  float pitch;
  float roll;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  is_radian = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "is_radian");
  q0 = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "q0");
  q1 = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "q1");
  q2 = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "q2");
  q3 = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "q3");

  /* Invoke the target function */
  quaternion_to_euler(is_radian, q0, q1, q2, q3, &roll, &pitch, &yaw);

  /* Marshall function outputs */
  plhs[0] = h_emlrt_marshallOut(roll);
  plhs[1] = h_emlrt_marshallOut(pitch);
  plhs[2] = h_emlrt_marshallOut(yaw);
}

/*
 * Arguments    : const mxArray *prhs[3]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void RungeKutta_api(const mxArray *prhs[3], const mxArray *plhs[1])
{
  float (*X)[13];
  float (*U)[6];
  float dT;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, true, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);

  /* Marshall function inputs */
  X = e_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "X");
  U = g_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "U");
  dT = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "dT");

  /* Invoke the target function */
  RungeKutta(*X, *U, dT);

  /* Marshall function outputs */
  i_emlrt_marshallOut(*X, prhs[0]);
  plhs[0] = prhs[0];
}

/*
 * Arguments    : const mxArray *prhs[6]
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void SerialUpdate_api(const mxArray *prhs[6], const mxArray *plhs[2])
{
  float (*H)[104];
  float (*R)[64];
  float (*Z)[8];
  float (*Y)[8];
  float (*P)[169];
  float (*X)[13];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);
  prhs[2] = emlrtProtectR2012b(prhs[2], 2, false, -1);
  prhs[3] = emlrtProtectR2012b(prhs[3], 3, false, -1);
  prhs[4] = emlrtProtectR2012b(prhs[4], 4, true, -1);
  prhs[5] = emlrtProtectR2012b(prhs[5], 5, true, -1);

  /* Marshall function inputs */
  H = s_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "H");
  R = k_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "R");
  Z = u_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "Z");
  Y = u_emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "Y");
  P = m_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "P");
  X = e_emlrt_marshallIn(&st, emlrtAlias(prhs[5]), "X");

  /* Invoke the target function */
  SerialUpdate(*H, *R, *Z, *Y, *P, *X);

  /* Marshall function outputs */
  i_emlrt_marshallOut(*X, prhs[5]);
  plhs[0] = prhs[5];
  j_emlrt_marshallOut(*P, prhs[4]);
  plhs[1] = prhs[4];
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *H
 *                const char *identifier
 * Return Type  : float (*)[104]
 */
static float (*s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *H, const
  char *identifier))[104]
{
  float (*y)[104];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = t_emlrt_marshallIn(sp, emlrtAlias(H), &thisId);
  emlrtDestroyArray(&H);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float (*)[104]
 */
  static float (*t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[104]
{
  float (*y)[104];
  y = gb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *Z
 *                const char *identifier
 * Return Type  : float (*)[8]
 */
static float (*u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *Z, const
  char *identifier))[8]
{
  float (*y)[8];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = v_emlrt_marshallIn(sp, emlrtAlias(Z), &thisId);
  emlrtDestroyArray(&Z);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : float (*)[8]
 */
  static float (*v_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[8]
{
  float (*y)[8];
  y = hb_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float (*)[4]
 */
static float (*w_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4]
{
  float (*ret)[4];
  int iv21[1];
  iv21[0] = 4;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 1U, iv21);
  ret = (float (*)[4])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float (*)[3]
 */
  static float (*x_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[3]
{
  float (*ret)[3];
  int iv22[1];
  iv22[0] = 3;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 1U, iv22);
  ret = (float (*)[3])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float (*)[13]
 */
static float (*y_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[13]
{
  float (*ret)[13];
  int iv23[1];
  iv23[0] = 13;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 1U, iv23);
  ret = (float (*)[13])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float (*)[6]
 */
  static float (*ab_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6]
{
  float (*ret)[6];
  int iv24[1];
  iv24[0] = 6;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 1U, iv24);
  ret = (float (*)[6])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float
 */
static float bb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  float ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 0U, 0);
  ret = *(float *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float (*)[64]
 */
static float (*cb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[64]
{
  float (*ret)[64];
  int iv25[2];
  int i;
  for (i = 0; i < 2; i++) {
    iv25[i] = 8;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 2U, iv25);
  ret = (float (*)[64])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float (*)[169]
 */
  static float (*db_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[169]
{
  float (*ret)[169];
  int iv26[2];
  int i;
  for (i = 0; i < 2; i++) {
    iv26[i] = 13;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 2U, iv26);
  ret = (float (*)[169])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float (*)[117]
 */
static float (*eb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[117]
{
  float (*ret)[117];
  int iv27[2];
  int i0;
  for (i0 = 0; i0 < 2; i0++) {
    iv27[i0] = 13 + -4 * i0;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 2U, iv27);
  ret = (float (*)[117])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float (*)[81]
 */
  static float (*fb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[81]
{
  float (*ret)[81];
  int iv28[2];
  int i;
  for (i = 0; i < 2; i++) {
    iv28[i] = 9;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 2U, iv28);
  ret = (float (*)[81])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float (*)[104]
 */
static float (*gb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[104]
{
  float (*ret)[104];
  int iv29[2];
  int i1;
  for (i1 = 0; i1 < 2; i1++) {
    iv29[i1] = 8 + 5 * i1;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 2U, iv29);
  ret = (float (*)[104])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : float (*)[8]
 */
  static float (*hb_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[8]
{
  float (*ret)[8];
  int iv30[1];
  iv30[0] = 8;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "single", false, 1U, iv30);
  ret = (float (*)[8])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * File trailer for _coder_ekf_13state_api.c
 *
 * [EOF]
 */
