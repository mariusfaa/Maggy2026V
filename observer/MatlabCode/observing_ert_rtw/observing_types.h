/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: observing_types.h
 *
 * Code generated for Simulink model 'observing'.
 *
 * Model version                  : 1.125
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Fri Apr 24 14:58:20 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef observing_types_h_
#define observing_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_struct_4pfJnL8EiWDH6PDuNDqsKC_
#define DEFINED_TYPEDEF_FOR_struct_4pfJnL8EiWDH6PDuNDqsKC_

typedef struct {
  real_T alpha;
  real_T beta;
  real_T kappa;
} struct_4pfJnL8EiWDH6PDuNDqsKC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_dZd4pOWAPoKYGFDw6kFnYF_
#define DEFINED_TYPEDEF_FOR_struct_dZd4pOWAPoKYGFDw6kFnYF_

typedef struct {
  real_T zEq;
  real_T xLp[6];
  real_T uLp[4];
  real_T dt;
  real_T nx;
  real_T nz;
  real_T delta;
  real_T A[36];
  real_T B[24];
  real_T C[18];
  real_T D[12];
  real_T Ad[36];
  real_T Bd[24];
  real_T Cd[18];
  real_T Dd[12];
  real_T P0[36];
  real_T R[9];
  real_T NSD[9];
  real_T G[18];
  real_T Q[36];
  real_T Qd[36];
  struct_4pfJnL8EiWDH6PDuNDqsKC ukf_params;
  real_T Wm[13];
  real_T Wc[13];
  real_T lambda;
} struct_dZd4pOWAPoKYGFDw6kFnYF;

#endif

/* Parameters (default storage) */
typedef struct P_observing_T_ P_observing_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_observing_T RT_MODEL_observing_T;

#endif                                 /* observing_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
