/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: observing.c
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

#include "observing.h"
#include "rtwtypes.h"
#include "observing_types.h"
#include <string.h>
#include <math.h>
#include "observing_private.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"

/* Block signals (default storage) */
B_observing_T observing_B;

/* Block states (default storage) */
DW_observing_T observing_DW;

/* Real-time model */
static RT_MODEL_observing_T observing_M_;
RT_MODEL_observing_T *const observing_M = &observing_M_;

/* Forward declaration for local functions */
static real_T observing_rt_hypotd_snf(real_T u0, real_T u1);
static real_T observing_rt_powd_snf(real_T u0, real_T u1);
static void computeFieldCircularWireCartesi(const real_T x[100], const real_T y
  [100], const real_T z[100], real_T r, real_T b_I, real_T bx[100], real_T by
  [100], real_T bz[100]);
static void observing_stateTrans(const real_T x[6], const real_T u[4], real_T dt,
  real_T xn[6]);
static void observing_binary_expand_op_11(real_T in1_data[], const int32_T
  in1_size[2], int32_T in2, const real_T in3[6], const real_T in4_data[], const
  int32_T *in4_size, const real_T in5[4], real_T in6, real_T in7);
static void observing_binary_expand_op_12(real_T in1_data[], int32_T in1_size[2],
  const real_T in2_data[], const int32_T in2_size[2], const real_T in3[36],
  const struct_dZd4pOWAPoKYGFDw6kFnYF *in4);
static void observing_binary_expand_op_1(real_T in1_data[], int32_T *in1_size,
  const real_T in2_data[], const int32_T in2_size[2], int32_T in3, const real_T
  in4_data[], const int32_T *in4_size);
static void observing_binary_expand_op(real_T in1_data[], const int32_T
  in1_size[2], const struct_dZd4pOWAPoKYGFDw6kFnYF *in2, int32_T in3, const
  real_T in4_data[], const int32_T *in4_size);
static void observing_binary_expand_op_10(real_T in1[36], const int8_T in2_data[],
  const int32_T in2_size[2], const real_T in3[18], const
  struct_dZd4pOWAPoKYGFDw6kFnYF *in4);
static void observing_ellipke(real_T m, real_T *k, real_T *e);
static void computeFieldCircularWireCarte_o(real_T x, real_T y, real_T z, real_T
  r, real_T b_I, real_T *bx, real_T *by, real_T *bz);
static void o_maglevSystemMeasurements_xred(const real_T x[6], const real_T u[4],
  real_T y[3]);
static void observing_binary_expand_op_7(real_T in1_data[], const int32_T
  in1_size[2], int32_T in2, const struct_dZd4pOWAPoKYGFDw6kFnYF *in3, const
  real_T in4_data[], const int32_T *in4_size, real_T in5);
static void observing_binary_expand_op_9(real_T in1[9], const real_T in2_data[],
  const int32_T in2_size[2], const real_T in3[36], const
  struct_dZd4pOWAPoKYGFDw6kFnYF *in4);
static void observing_binary_expand_op_8(real_T in1_data[], int32_T in1_size[2],
  const int8_T in2_data[], const int32_T in2_size[2], const real_T in3[18],
  const real_T in4_data[], const int32_T in4_size[2], const real_T in5[36],
  const struct_dZd4pOWAPoKYGFDw6kFnYF *in6);
static void observing_chol(real_T A[36]);
static void observing_binary_expand_op_3(real_T in1[9], const
  struct_dZd4pOWAPoKYGFDw6kFnYF *in2, int32_T in3, const real_T in4_data[],
  const int32_T *in4_size);
static void observing_binary_expand_op_2(real_T in1_data[], int32_T in1_size[2],
  const struct_dZd4pOWAPoKYGFDw6kFnYF *in2, int32_T in3, const real_T in4[78],
  const real_T in5[6], const real_T in6_data[], const int32_T *in6_size);
static void observing_binary_expand_op_6(real_T in1_data[], int32_T *in1_size,
  const real_T in2[6], const real_T in3_data[], const int32_T in3_size[2], const
  real_T in4[3], const real_T in5_data[], const int32_T *in5_size);
static void observing_binary_expand_op_5(real_T in1_data[], int32_T in1_size[2],
  const real_T in2[36], const real_T in3_data[], const int32_T in3_size[2],
  const real_T in4[9]);
int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

static real_T observing_rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T y;
  observing_B.a_a = fabs(u0);
  observing_B.b = fabs(u1);
  if (observing_B.a_a < observing_B.b) {
    observing_B.a_a /= observing_B.b;
    y = sqrt(observing_B.a_a * observing_B.a_a + 1.0) * observing_B.b;
  } else if (observing_B.a_a > observing_B.b) {
    observing_B.b /= observing_B.a_a;
    y = sqrt(observing_B.b * observing_B.b + 1.0) * observing_B.a_a;
  } else if (rtIsNaN(observing_B.b)) {
    y = (rtNaN);
  } else {
    y = observing_B.a_a * 1.4142135623730951;
  }

  return y;
}

static real_T observing_rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    observing_B.d2 = fabs(u0);
    observing_B.d3 = fabs(u1);
    if (rtIsInf(u1)) {
      if (observing_B.d2 == 1.0) {
        y = 1.0;
      } else if (observing_B.d2 > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (observing_B.d3 == 0.0) {
      y = 1.0;
    } else if (observing_B.d3 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S5>/MATLAB Function' */
static void computeFieldCircularWireCartesi(const real_T x[100], const real_T y
  [100], const real_T z[100], real_T r, real_T b_I, real_T bx[100], real_T by
  [100], real_T bz[100])
{
  int32_T d_size_idx_1;
  int32_T k;
  int32_T trueCount;
  int8_T d;
  boolean_T indices;
  observing_B.b_x_f = 1.2566370614359173E-6 * b_I;
  observing_B.a_j = 4.0 * r;
  observing_B.c = r * r;
  trueCount = 0;
  for (k = 0; k < 100; k++) {
    observing_B.y = y[k];
    observing_B.s0 = x[k];
    observing_B.phi[k] = rt_atan2d_snf(observing_B.y, observing_B.s0);
    observing_B.s0 = observing_rt_hypotd_snf(observing_B.s0, observing_B.y);
    observing_B.rho[k] = observing_B.s0;
    bz[k] = observing_B.b_x_f / (sqrt(r * observing_B.s0) * 12.566370614359172);
    observing_B.i1 = r + observing_B.s0;
    observing_B.y = z[k];
    observing_B.y *= observing_B.y;
    observing_B.s0 = fmin(observing_B.a_j * observing_B.s0 / (observing_B.i1 *
      observing_B.i1 + observing_B.y), 1.0);
    observing_B.k2[k] = observing_B.s0;
    if (observing_B.s0 == 1.0) {
      observing_B.i1 = (rtInf);
      observing_B.a1 = 1.0;
    } else {
      observing_B.a0 = 1.0;
      observing_B.b0 = sqrt(1.0 - observing_B.s0);
      observing_B.i1 = 0.0;
      observing_B.w1 = 1.0;
      observing_B.a1 = 1.0;
      while (observing_B.w1 > 2.2204460492503131E-16) {
        observing_B.a1 = (observing_B.a0 + observing_B.b0) / 2.0;
        observing_B.b1 = sqrt(observing_B.a0 * observing_B.b0);
        observing_B.a0 = (observing_B.a0 - observing_B.b0) / 2.0;
        observing_B.i1++;
        observing_B.w1 = observing_rt_powd_snf(2.0, observing_B.i1) *
          observing_B.a0 * observing_B.a0;
        observing_B.s0 += observing_B.w1;
        observing_B.a0 = observing_B.a1;
        observing_B.b0 = observing_B.b1;
      }

      observing_B.i1 = 3.1415926535897931 / (2.0 * observing_B.a1);
      observing_B.a1 = (1.0 - observing_B.s0 / 2.0) * observing_B.i1;
    }

    observing_B.s0 = observing_B.rho[k];
    observing_B.w1 = observing_B.s0 - r;
    observing_B.b1 = bz[k];
    observing_B.a0 = sqrt(observing_B.k2[k]);
    observing_B.b0 = observing_B.s0 * observing_B.s0;
    observing_B.w1 = observing_B.w1 * observing_B.w1 + observing_B.y;
    by[k] = (observing_B.i1 - ((observing_B.b0 + observing_B.c) + observing_B.y)
             / observing_B.w1 * observing_B.a1) * (-(z[k] / observing_B.s0) *
      observing_B.b1 * observing_B.a0);
    observing_B.k2[k] = observing_B.a0;
    bz[k] = (observing_B.i1 - ((observing_B.b0 - observing_B.c) + observing_B.y)
             / observing_B.w1 * observing_B.a1) * (observing_B.b1 *
      observing_B.a0);
    indices = (observing_B.s0 < 1.0E-6);
    observing_B.indices[k] = indices;
    if (indices) {
      trueCount++;
    }
  }

  d_size_idx_1 = trueCount;
  trueCount = 0;
  for (k = 0; k < 100; k++) {
    if (observing_B.indices[k]) {
      observing_B.d_data[trueCount] = (int8_T)k;
      trueCount++;
    }
  }

  for (k = 0; k < d_size_idx_1; k++) {
    d = observing_B.d_data[k];
    observing_B.phi[d] = 0.0;
    by[d] = 0.0;
  }

  observing_B.b_x_f = observing_B.c * 1.2566370614359173E-6 * b_I;
  trueCount = 0;
  for (k = 0; k < 100; k++) {
    if (observing_B.indices[k]) {
      observing_B.a_j = z[k];
      bz[observing_B.d_data[trueCount]] = observing_B.b_x_f /
        (observing_rt_powd_snf(observing_B.a_j * observing_B.a_j + observing_B.c,
          1.5) * 2.0);
      trueCount++;
    }

    observing_B.a_j = observing_B.phi[k];
    observing_B.y = by[k];
    bx[k] = observing_B.y * cos(observing_B.a_j);
    observing_B.a_j = sin(observing_B.a_j);
    observing_B.phi[k] = observing_B.a_j;
    by[k] = observing_B.y * observing_B.a_j;
  }
}

/* Function for MATLAB Function: '<S5>/MATLAB Function' */
static void observing_stateTrans(const real_T x[6], const real_T u[4], real_T dt,
  real_T xn[6])
{
  int32_T ia;
  int32_T iac;
  int32_T ix;
  int32_T p_tmp;
  static const real_T b[300] = { 0.025, 0.0, 0.0, 0.024950668210706791,
    0.0015697629882328345, 0.0, 0.024802867532861948, 0.0031333308391076065, 0.0,
    0.02455718126821722, 0.0046845328646431157, 0.0, 0.024214579028215777,
    0.00621724717912137, 0.0, 0.023776412907378839, 0.0077254248593736849, 0.0,
    0.023244412147206284, 0.00920311381711695, 0.0, 0.022620676311650489,
    0.010644482289126818, 0.0, 0.02190766700109659, 0.012043841852542883, 0.0,
    0.021108198137550379, 0.013395669874474917, 0.0, 0.020225424859373689,
    0.014694631307311828, 0.0, 0.019262831069394731, 0.015935599743717246, 0.0,
    0.01822421568553529, 0.01711367764821722, 0.0, 0.01711367764821722,
    0.01822421568553529, 0.0, 0.015935599743717242, 0.019262831069394734, 0.0,
    0.014694631307311827, 0.020225424859373689, 0.0, 0.013395669874474915,
    0.021108198137550379, 0.0, 0.01204384185254288, 0.021907667001096594, 0.0,
    0.010644482289126816, 0.022620676311650489, 0.0, 0.0092031138171169476,
    0.023244412147206288, 0.0, 0.0077254248593736866, 0.023776412907378839, 0.0,
    0.0062172471791213692, 0.024214579028215777, 0.0, 0.0046845328646431131,
    0.02455718126821722, 0.0, 0.0031333308391076065, 0.024802867532861948, 0.0,
    0.0015697629882328327, 0.024950668210706791, 0.0, -4.0203066241915918E-18,
    0.025, 0.0, -0.0015697629882328351, 0.024950668210706791, 0.0,
    -0.0031333308391076095, 0.024802867532861945, 0.0, -0.0046845328646431209,
    0.024557181268217217, 0.0, -0.0062172471791213718, 0.024214579028215777, 0.0,
    -0.0077254248593736892, 0.023776412907378839, 0.0, -0.00920311381711695,
    0.023244412147206288, 0.0, -0.010644482289126818, 0.022620676311650489, 0.0,
    -0.012043841852542887, 0.021907667001096587, 0.0, -0.013395669874474922,
    0.021108198137550375, 0.0, -0.014694631307311827, 0.020225424859373689, 0.0,
    -0.015935599743717246, 0.019262831069394734, 0.0, -0.01711367764821722,
    0.018224215685535287, 0.0, -0.018224215685535294, 0.017113677648217213, 0.0,
    -0.019262831069394734, 0.015935599743717239, 0.0, -0.020225424859373686,
    0.014694631307311832, 0.0, -0.021108198137550379, 0.013395669874474917, 0.0,
    -0.02190766700109659, 0.012043841852542881, 0.0, -0.022620676311650489,
    0.010644482289126813, 0.0, -0.023244412147206288, 0.0092031138171169442, 0.0,
    -0.023776412907378842, 0.007725424859373678, 0.0, -0.024214579028215777,
    0.0062172471791213709, 0.0, -0.02455718126821722, 0.0046845328646431149, 0.0,
    -0.024802867532861948, 0.0031333308391076026, 0.0, -0.024950668210706791,
    0.0015697629882328286, 0.0, -0.025, -8.0406132483831836E-18, 0.0,
    -0.024950668210706791, -0.0015697629882328338, 0.0, -0.024802867532861948,
    -0.0031333308391076073, 0.0, -0.024557181268217217, -0.0046845328646431192,
    0.0, -0.024214579028215777, -0.0062172471791213761, 0.0,
    -0.023776412907378839, -0.0077254248593736936, 0.0, -0.023244412147206281,
    -0.009203113817116958, 0.0, -0.022620676311650489, -0.010644482289126816,
    0.0, -0.02190766700109659, -0.012043841852542885, 0.0, -0.021108198137550375,
    -0.01339566987447492, 0.0, -0.020225424859373682, -0.014694631307311835, 0.0,
    -0.019262831069394727, -0.015935599743717249, 0.0, -0.01822421568553529,
    -0.01711367764821722, 0.0, -0.017113677648217217, -0.01822421568553529, 0.0,
    -0.015935599743717239, -0.019262831069394734, 0.0, -0.014694631307311832,
    -0.020225424859373686, 0.0, -0.013395669874474908, -0.021108198137550382,
    0.0, -0.012043841852542883, -0.02190766700109659, 0.0, -0.010644482289126804,
    -0.022620676311650496, 0.0, -0.0092031138171169459, -0.023244412147206288,
    0.0, -0.0077254248593736892, -0.023776412907378839, 0.0,
    -0.0062172471791213614, -0.02421457902821578, 0.0, -0.0046845328646431157,
    -0.02455718126821722, 0.0, -0.0031333308391075934, -0.024802867532861948,
    0.0, -0.0015697629882328304, -0.024950668210706791, 0.0,
    -4.5924254968025744E-18, -0.025, 0.0, 0.0015697629882328432,
    -0.024950668210706791, 0.0, 0.003133330839107606, -0.024802867532861948, 0.0,
    0.0046845328646431287, -0.024557181268217217, 0.0, 0.0062172471791213735,
    -0.024214579028215777, 0.0, 0.0077254248593736814, -0.023776412907378842,
    0.0, 0.0092031138171169563, -0.023244412147206281, 0.0, 0.010644482289126816,
    -0.022620676311650489, 0.0, 0.012043841852542893, -0.021907667001096587, 0.0,
    0.01339566987447492, -0.021108198137550375, 0.0, 0.014694631307311842,
    -0.020225424859373675, 0.0, 0.015935599743717249, -0.019262831069394727, 0.0,
    0.017113677648217217, -0.01822421568553529, 0.0, 0.018224215685535297,
    -0.017113677648217206, 0.0, 0.019262831069394734, -0.015935599743717242, 0.0,
    0.020225424859373696, -0.014694631307311815, 0.0, 0.021108198137550382,
    -0.013395669874474908, 0.0, 0.02190766700109659, -0.012043841852542883, 0.0,
    0.022620676311650493, -0.010644482289126806, 0.0, 0.023244412147206288,
    -0.0092031138171169476, 0.0, 0.023776412907378846, -0.00772542485937367, 0.0,
    0.02421457902821578, -0.0062172471791213622, 0.0, 0.02455718126821722,
    -0.0046845328646431175, 0.0, 0.024802867532861948, -0.0031333308391075947,
    0.0, 0.024950668210706791, -0.0015697629882328317, 0.0 };

  static const real_T c[4] = { 0.024748737341529166, -0.024748737341529166,
    0.024748737341529166, -0.024748737341529166 };

  static const real_T d[4] = { 0.024748737341529166, 0.024748737341529166,
    -0.024748737341529166, -0.024748737341529166 };

  static const real_T e[4] = { 0.02, 0.0, -0.02, 0.0 };

  static const real_T f[4] = { 0.0, 0.02, 0.0, -0.02 };

  static const real_T g[300] = { 2.1439945078856567E-13, 3501.4087480216976, 0.0,
    -219.85527437276519, 3494.4995176782249, 0.0, -438.84288041990118,
    3473.7990942234464, 0.0, -656.09857410626182, 3439.3891727716132, 0.0,
    -870.76494646355093, 3391.4055535622988, 0.0, -1081.9948073918124,
    3330.037606018891, 0.0, -1288.9545291317058, 3255.5275213939963, 0.0,
    -1490.8273362124266, 3168.1693569512081, 0.0, -1686.816528891339,
    3068.3078754554354, 0.0, -1876.1486273638868, 2956.3371845517681, 0.0,
    -2058.0764243350059, 2832.6991814026615, 0.0, -2231.8819339049551,
    2697.8818087217146, 0.0, -2396.8792251316463, 2552.4171290867, 0.0,
    -2552.4171290866993, 2396.8792251316468, 0.0, -2697.8818087217151,
    2231.8819339049546, 0.0, -2832.699181402661, 2058.0764243350063, 0.0,
    -2956.3371845517677, 1876.1486273638875, 0.0, -3068.3078754554349,
    1686.8165288913397, 0.0, -3168.1693569512076, 1490.8273362124271, 0.0,
    -3255.5275213939963, 1288.9545291317049, 0.0, -3330.037606018891,
    1081.9948073918129, 0.0, -3391.4055535622983, 870.76494646355161, 0.0,
    -3439.3891727716132, 656.09857410626171, 0.0, -3473.799094223446,
    438.84288041990254, 0.0, -3494.4995176782249, 219.85527437276426, 0.0,
    -3501.4087480216976, 4.2879890157713134E-13, 0.0, -3494.4995176782249,
    -219.855274372765, 0.0, -3473.7990942234464, -438.84288041990169, 0.0,
    -3439.3891727716132, -656.09857410626239, 0.0, -3391.4055535622983,
    -870.76494646355229, 0.0, -3330.037606018891, -1081.9948073918122, 0.0,
    -3255.5275213939963, -1288.9545291317056, 0.0, -3168.1693569512081,
    -1490.8273362124264, 0.0, -3068.3078754554349, -1686.81652889134, 0.0,
    -2956.3371845517672, -1876.148627363888, 0.0, -2832.6991814026615,
    -2058.0764243350059, 0.0, -2697.8818087217146, -2231.8819339049546, 0.0,
    -2552.4171290867, -2396.8792251316463, 0.0, -2396.8792251316459,
    -2552.4171290867, 0.0, -2231.8819339049546, -2697.8818087217151, 0.0,
    -2058.0764243350063, -2832.699181402661, 0.0, -1876.1486273638891,
    -2956.3371845517672, 0.0, -1686.8165288913399, -3068.3078754554349, 0.0,
    -1490.8273362124246, -3168.1693569512095, 0.0, -1288.9545291317052,
    -3255.5275213939963, 0.0, -1081.9948073918131, -3330.037606018891, 0.0,
    -870.7649464635532, -3391.4055535622979, 0.0, -656.09857410626194,
    -3439.3891727716132, 0.0, -438.84288041989976, -3473.7990942234464, 0.0,
    -219.85527437276451, -3494.4995176782249, 0.0, -6.43198352365697E-13,
    -3501.4087480216976, 0.0, 219.8552743727632, -3494.4995176782249, 0.0,
    438.84288041990146, -3473.7990942234464, 0.0, 656.09857410626364,
    -3439.3891727716132, 0.0, 870.764946463552, -3391.4055535622983, 0.0,
    1081.9948073918119, -3330.037606018891, 0.0, 1288.9545291317067,
    -3255.5275213939954, 0.0, 1490.8273362124262, -3168.1693569512086, 0.0,
    1686.8165288913412, -3068.307875455434, 0.0, 1876.148627363888,
    -2956.3371845517672, 0.0, 2058.0764243350054, -2832.6991814026619, 0.0,
    2231.881933904956, -2697.8818087217137, 0.0, 2396.8792251316459,
    -2552.4171290867, 0.0, 2552.4171290867012, -2396.8792251316449, 0.0,
    2697.8818087217151, -2231.8819339049546, 0.0, 2832.699181402661,
    -2058.0764243350068, 0.0, 2956.3371845517681, -1876.1486273638864, 0.0,
    3068.3078754554349, -1686.8165288913399, 0.0, 3168.169356951209,
    -1490.8273362124248, 0.0, 3255.5275213939963, -1288.9545291317054, 0.0,
    3330.037606018891, -1081.9948073918133, 0.0, 3391.4055535622988,
    -870.76494646355036, 0.0, 3439.3891727716132, -656.09857410626216, 0.0,
    3473.7990942234464, -438.84288041989993, 0.0, 3494.4995176782249,
    -219.85527437276471, 0.0, 3501.4087480216976, -8.5759780315426267E-13, 0.0,
    3494.4995176782249, 219.8552743727661, 0.0, 3473.7990942234464,
    438.84288041990129, 0.0, 3439.3891727716132, 656.09857410626353, 0.0,
    3391.4055535622983, 870.76494646355172, 0.0, 3330.037606018891,
    1081.9948073918117, 0.0, 3255.5275213939958, 1288.9545291317065, 0.0,
    3168.1693569512086, 1490.827336212426, 0.0, 3068.307875455434,
    1686.8165288913412, 0.0, 2956.3371845517677, 1876.1486273638875, 0.0,
    2832.6991814026605, 2058.0764243350077, 0.0, 2697.8818087217137,
    2231.881933904956, 0.0, 2552.4171290867, 2396.8792251316459, 0.0,
    2396.8792251316454, 2552.4171290867012, 0.0, 2231.8819339049546,
    2697.8818087217146, 0.0, 2058.0764243350045, 2832.6991814026628, 0.0,
    1876.1486273638868, 2956.3371845517681, 0.0, 1686.81652889134,
    3068.3078754554349, 0.0, 1490.8273362124251, 3168.169356951209, 0.0,
    1288.9545291317056, 3255.5275213939963, 0.0, 1081.9948073918106,
    3330.0376060188914, 0.0, 870.7649464635507, 3391.4055535622988, 0.0,
    656.09857410626239, 3439.3891727716132, 0.0, 438.84288041990015,
    3473.7990942234464, 0.0, 219.85527437276491, 3494.4995176782249, 0.0 };

  static const real_T h[101] = { 0.0, 0.062831853071795868, 0.12566370614359174,
    0.1884955592153876, 0.25132741228718347, 0.31415926535897931,
    0.37699111843077521, 0.4398229715025711, 0.50265482457436694,
    0.56548667764616278, 0.62831853071795862, 0.69115038378975457,
    0.75398223686155041, 0.81681408993334625, 0.87964594300514221,
    0.942477796076938, 1.0053096491487339, 1.0681415022205298,
    1.1309733552923256, 1.1938052083641215, 1.2566370614359172,
    1.3194689145077132, 1.3823007675795091, 1.4451326206513049,
    1.5079644737231008, 1.5707963267948968, 1.6336281798666925,
    1.6964600329384885, 1.7592918860102844, 1.8221237390820801,
    1.8849555921538761, 1.9477874452256718, 2.0106192982974678,
    2.0734511513692637, 2.1362830044410597, 2.1991148575128552,
    2.2619467105846511, 2.3247785636564471, 2.387610416728243, 2.450442269800039,
    2.5132741228718345, 2.5761059759436304, 2.6389378290154264,
    2.7017696820872223, 2.7646015351590183, 2.8274333882308142,
    2.8902652413026098, 2.9530970943744057, 3.0159289474462017,
    3.0787608005179976, 3.1415926535897936, 3.2044245066615891,
    3.267256359733385, 3.330088212805181, 3.3929200658769769, 3.4557519189487729,
    3.5185837720205688, 3.5814156250923643, 3.6442474781641603,
    3.7070793312359562, 3.7699111843077522, 3.8327430373795481,
    3.8955748904513436, 3.9584067435231396, 4.0212385965949355,
    4.0840704496667311, 4.1469023027385274, 4.209734155810323,
    4.2725660088821193, 4.3353978619539149, 4.39822971502571, 4.4610615680975068,
    4.5238934211693023, 4.5867252742410987, 4.6495571273128942, 4.71238898038469,
    4.7752208334564861, 4.8380526865282816, 4.900884539600078,
    4.9637163926718735, 5.026548245743669, 5.0893800988154654,
    5.1522119518872609, 5.2150438049590573, 5.2778756580308528,
    5.3407075111026492, 5.4035393641744447, 5.46637121724624, 5.5292030703180366,
    5.5920349233898321, 5.6548667764616285, 5.717698629533424,
    5.7805304826052195, 5.8433623356770159, 5.9061941887488114,
    5.9690260418206078, 6.0318578948924033, 6.0946897479641988,
    6.1575216010359952, 6.2203534541077907, 6.2831853071795862 };

  static const int8_T a[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 };

  static const int8_T b_a[18] = { 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 1 };

  observing_B.x = x[0];
  observing_B.x_e = x[1];
  observing_B.x_b = x[2];
  for (ix = 0; ix < 100; ix++) {
    observing_B.p[3 * ix] = b[3 * ix] + observing_B.x;
    p_tmp = 3 * ix + 1;
    observing_B.p[p_tmp] = b[p_tmp] + observing_B.x_e;
    p_tmp = 3 * ix + 2;
    observing_B.p[p_tmp] = b[p_tmp] + observing_B.x_b;
    observing_B.bx[ix] = 0.0;
    observing_B.by[ix] = 0.0;
    observing_B.bz[ix] = 0.0;
  }

  for (p_tmp = 0; p_tmp < 4; p_tmp++) {
    observing_B.x = c[p_tmp];
    observing_B.x_e = d[p_tmp];
    for (ix = 0; ix < 100; ix++) {
      observing_B.p_m[ix] = observing_B.p[3 * ix] - observing_B.x;
      observing_B.p_c[ix] = observing_B.p[3 * ix + 1] - observing_B.x_e;
      observing_B.p_k[ix] = observing_B.p[3 * ix + 2] - 0.0042;
    }

    computeFieldCircularWireCartesi(observing_B.p_m, observing_B.p_c,
      observing_B.p_k, 0.01, 7321.1273822271851, observing_B.bxTemp,
      observing_B.byTemp, observing_B.bzTemp);
    for (ix = 0; ix < 100; ix++) {
      observing_B.bx[ix] += observing_B.bxTemp[ix];
      observing_B.by[ix] += observing_B.byTemp[ix];
      observing_B.bz[ix] += observing_B.bzTemp[ix];
    }
  }

  for (p_tmp = 0; p_tmp < 4; p_tmp++) {
    observing_B.x = e[p_tmp];
    observing_B.x_e = f[p_tmp];
    for (ix = 0; ix < 100; ix++) {
      observing_B.p_m[ix] = observing_B.p[3 * ix] - observing_B.x;
      observing_B.p_c[ix] = observing_B.p[3 * ix + 1] - observing_B.x_e;
      observing_B.p_k[ix] = observing_B.p[3 * ix + 2] - 0.006;
    }

    computeFieldCircularWireCartesi(observing_B.p_m, observing_B.p_c,
      observing_B.p_k, 0.0052206519382138588, u[p_tmp], observing_B.bxTemp,
      observing_B.byTemp, observing_B.bzTemp);
    for (ix = 0; ix < 100; ix++) {
      observing_B.bx[ix] += observing_B.bxTemp[ix] * 480.0;
      observing_B.by[ix] += observing_B.byTemp[ix] * 480.0;
      observing_B.bz[ix] += observing_B.bzTemp[ix] * 480.0;
    }
  }

  for (ix = 0; ix < 100; ix++) {
    observing_B.p[3 * ix] = observing_B.bx[ix];
    observing_B.p[3 * ix + 1] = observing_B.by[ix];
    observing_B.p[3 * ix + 2] = observing_B.bz[ix];
  }

  for (ix = 0; ix <= 297; ix += 3) {
    for (p_tmp = ix + 1; p_tmp <= ix + 1; p_tmp++) {
      observing_B.x = g[p_tmp];
      observing_B.x_e = g[p_tmp + 1];
      observing_B.x_b = observing_B.p[p_tmp + 1];
      observing_B.p_b = observing_B.p[p_tmp];
      observing_B.F[p_tmp - 1] = observing_B.x * observing_B.x_b -
        observing_B.x_e * observing_B.p_b;
      observing_B.g = g[p_tmp - 1];
      observing_B.p_d = observing_B.p[p_tmp - 1];
      observing_B.F[p_tmp] = observing_B.x_e * observing_B.p_d - observing_B.g *
        observing_B.x_b;
      observing_B.F[p_tmp + 1] = observing_B.g * observing_B.p_b - observing_B.x
        * observing_B.p_d;
    }
  }

  for (ix = 0; ix < 100; ix++) {
    observing_B.A[ix] = observing_B.F[3 * ix];
  }

  observing_B.A[100] = observing_B.F[0];
  observing_B.b_x[0] = 0.031415926535897934;
  for (ix = 0; ix < 99; ix++) {
    observing_B.b_x[ix + 1] = (h[ix + 2] - h[ix]) * 0.5;
  }

  observing_B.b_x[100] = 0.031415926535897754;
  observing_B.x_e = 0.0;
  ix = 0;
  for (p_tmp = 0; p_tmp < 101; p_tmp++) {
    iac = p_tmp + 1;
    for (ia = iac; ia <= iac; ia++) {
      observing_B.x_e += observing_B.A[ia - 1] * observing_B.b_x[ix];
    }

    ix++;
  }

  for (ix = 0; ix < 100; ix++) {
    observing_B.A[ix] = observing_B.F[3 * ix + 1];
  }

  observing_B.A[100] = observing_B.F[1];
  observing_B.c_x[0] = 0.031415926535897934;
  for (ix = 0; ix < 99; ix++) {
    observing_B.c_x[ix + 1] = (h[ix + 2] - h[ix]) * 0.5;
  }

  observing_B.c_x[100] = 0.031415926535897754;
  observing_B.p_b = 0.0;
  ix = 0;
  for (p_tmp = 0; p_tmp < 101; p_tmp++) {
    iac = p_tmp + 1;
    for (ia = iac; ia <= iac; ia++) {
      observing_B.p_b += observing_B.A[ia - 1] * observing_B.c_x[ix];
    }

    ix++;
  }

  for (ix = 0; ix < 100; ix++) {
    observing_B.A[ix] = observing_B.F[3 * ix + 2];
  }

  observing_B.A[100] = observing_B.F[2];
  observing_B.d_x[0] = 0.031415926535897934;
  for (ix = 0; ix < 99; ix++) {
    observing_B.d_x[ix + 1] = (h[ix + 2] - h[ix]) * 0.5;
  }

  observing_B.d_x[100] = 0.031415926535897754;
  observing_B.x = 0.0;
  ix = 0;
  for (p_tmp = 0; p_tmp < 101; p_tmp++) {
    iac = p_tmp + 1;
    for (ia = iac; ia <= iac; ia++) {
      observing_B.x += observing_B.A[ia - 1] * observing_B.d_x[ix];
    }

    ix++;
  }

  observing_B.x_e *= 0.025;
  observing_B.x_b = observing_B.x_e * 0.0;
  observing_B.p_b = 0.025 * observing_B.p_b - observing_B.x_b;
  observing_B.x = ((0.025 * observing_B.x - observing_B.x_b) - observing_B.p_b *
                   0.0) / 0.06;
  observing_B.x_b = observing_B.x * 0.0;
  observing_B.p_b = (observing_B.p_b - observing_B.x_b) / 0.06;
  observing_B.Y[0] = ((observing_B.x_e - observing_B.x_b) - 0.0 *
                      observing_B.p_b) / 0.06;
  observing_B.Y[1] = observing_B.p_b;
  observing_B.Y[2] = observing_B.x - 9.81;
  for (ix = 0; ix < 6; ix++) {
    observing_B.a[ix] = 0.0;
  }

  for (ix = 0; ix < 6; ix++) {
    observing_B.x = x[ix];
    for (p_tmp = 0; p_tmp < 6; p_tmp++) {
      observing_B.a[p_tmp] += (real_T)a[6 * ix + p_tmp] * observing_B.x;
    }

    observing_B.b_a[ix] = 0.0;
  }

  for (ix = 0; ix < 3; ix++) {
    observing_B.x = observing_B.Y[ix];
    for (p_tmp = 0; p_tmp < 6; p_tmp++) {
      observing_B.b_a[p_tmp] += (real_T)b_a[6 * ix + p_tmp] * observing_B.x;
    }
  }

  for (ix = 0; ix < 6; ix++) {
    xn[ix] = (observing_B.a[ix] + observing_B.b_a[ix]) * dt + x[ix];
  }
}

static void observing_binary_expand_op_11(real_T in1_data[], const int32_T
  in1_size[2], int32_T in2, const real_T in3[6], const real_T in4_data[], const
  int32_T *in4_size, const real_T in5[4], real_T in6, real_T in7)
{
  int32_T i;
  int32_T stride_0_0;

  /* MATLAB Function: '<S5>/MATLAB Function' */
  stride_0_0 = (*in4_size != 1);
  for (i = 0; i < 6; i++) {
    observing_B.in3_b[i] = in4_data[i * stride_0_0] + in3[i];
  }

  observing_stateTrans(observing_B.in3_b, in5, in6, observing_B.dv1);
  for (i = 0; i < 6; i++) {
    observing_B.in3_b[i] = in3[i] - in4_data[i * stride_0_0];
  }

  observing_stateTrans(observing_B.in3_b, in5, in6, observing_B.dv2);
  observing_B.d1 = 2.0 * in7;
  stride_0_0 = in1_size[0];
  for (i = 0; i < stride_0_0; i++) {
    in1_data[i + in1_size[0] * in2] = (observing_B.dv1[i] - observing_B.dv2[i]) /
      observing_B.d1;
  }

  /* End of MATLAB Function: '<S5>/MATLAB Function' */
}

static void observing_binary_expand_op_12(real_T in1_data[], int32_T in1_size[2],
  const real_T in2_data[], const int32_T in2_size[2], const real_T in3[36],
  const struct_dZd4pOWAPoKYGFDw6kFnYF *in4)
{
  real_T tmp;
  int32_T aux_0_1;
  int32_T i;
  int32_T i_0;
  int32_T in2_data_tmp;
  int32_T in2_size_idx_0;
  int32_T stride_0_0;
  int32_T tmp_0;

  /* MATLAB Function: '<S5>/MATLAB Function' incorporates:
   *  UnitDelay: '<S5>/Unit Delay1'
   */
  i = in2_size[0];
  in2_size_idx_0 = in2_size[0];
  for (i_0 = 0; i_0 < 6; i_0++) {
    for (stride_0_0 = 0; stride_0_0 < i; stride_0_0++) {
      observing_B.in2_data_g[stride_0_0 + in2_size_idx_0 * i_0] = 0.0;
    }

    for (stride_0_0 = 0; stride_0_0 < 6; stride_0_0++) {
      tmp = in3[6 * i_0 + stride_0_0];
      for (aux_0_1 = 0; aux_0_1 < i; aux_0_1++) {
        in2_data_tmp = in2_size_idx_0 * i_0 + aux_0_1;
        observing_B.in2_data_g[in2_data_tmp] += in2_data[in2_size[0] *
          stride_0_0 + aux_0_1] * tmp;
      }
    }
  }

  in2_size_idx_0 = in2_size[0];
  for (i_0 = 0; i_0 < i; i_0++) {
    for (stride_0_0 = 0; stride_0_0 < i; stride_0_0++) {
      observing_B.in2_data[stride_0_0 + in2_size_idx_0 * i_0] = 0.0;
    }

    for (stride_0_0 = 0; stride_0_0 < 6; stride_0_0++) {
      tmp_0 = in2_size[0] * stride_0_0;
      tmp = in2_data[i_0 + tmp_0];
      for (aux_0_1 = 0; aux_0_1 < i; aux_0_1++) {
        in2_data_tmp = in2_size_idx_0 * i_0 + aux_0_1;
        observing_B.in2_data[in2_data_tmp] += observing_B.in2_data_g[aux_0_1 +
          tmp_0] * tmp;
      }
    }
  }

  in1_size[0] = 6;
  in1_size[1] = 6;
  stride_0_0 = (in2_size[0] != 1);
  aux_0_1 = 0;
  for (i = 0; i < 6; i++) {
    for (i_0 = 0; i_0 < 6; i_0++) {
      tmp_0 = 6 * i + i_0;
      in1_data[tmp_0] = observing_B.in2_data[i_0 * stride_0_0 + in2_size_idx_0 *
        aux_0_1] + in4->Qd[tmp_0];
    }

    aux_0_1 += stride_0_0;
  }

  /* End of MATLAB Function: '<S5>/MATLAB Function' */
}

static void observing_binary_expand_op_1(real_T in1_data[], int32_T *in1_size,
  const real_T in2_data[], const int32_T in2_size[2], int32_T in3, const real_T
  in4_data[], const int32_T *in4_size)
{
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;

  /* MATLAB Function: '<S5>/MATLAB Function' incorporates:
   *  MATLAB Function: '<S5>/MATLAB Function1'
   */
  loop_ub = *in4_size == 1 ? in2_size[0] : *in4_size;
  *in1_size = loop_ub;
  stride_0_0 = (in2_size[0] != 1);
  stride_1_0 = (*in4_size != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = in2_data[i * stride_0_0 + in2_size[0] * in3] - in4_data[i *
      stride_1_0];
  }

  /* End of MATLAB Function: '<S5>/MATLAB Function' */
}

static void observing_binary_expand_op(real_T in1_data[], const int32_T
  in1_size[2], const struct_dZd4pOWAPoKYGFDw6kFnYF *in2, int32_T in3, const
  real_T in4_data[], const int32_T *in4_size)
{
  int32_T aux_0_1;
  int32_T i;
  int32_T i_0;
  int32_T in1_size_idx_0;
  int32_T loop_ub;
  int32_T loop_ub_0;
  int32_T stride_0_0;

  /* MATLAB Function: '<S5>/MATLAB Function' */
  observing_B.in2_a = in2->Wc[in3];
  i = *in4_size;
  for (i_0 = 0; i_0 < i; i_0++) {
    for (loop_ub_0 = 0; loop_ub_0 < i; loop_ub_0++) {
      observing_B.in4_data[loop_ub_0 + *in4_size * i_0] = in4_data[loop_ub_0] *
        in4_data[i_0];
    }
  }

  loop_ub_0 = in1_size[0];
  in1_size_idx_0 = in1_size[0];
  loop_ub = in1_size[1];
  stride_0_0 = (*in4_size != 1);
  aux_0_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    for (i_0 = 0; i_0 < loop_ub_0; i_0++) {
      int32_T in1_data_tmp;
      in1_data_tmp = in1_size[0] * i + i_0;
      observing_B.in1_data[in1_data_tmp] = observing_B.in4_data[i_0 * stride_0_0
        + *in4_size * aux_0_1] * observing_B.in2_a + in1_data[in1_data_tmp];
    }

    aux_0_1 += stride_0_0;
  }

  for (i = 0; i < loop_ub; i++) {
    for (i_0 = 0; i_0 < loop_ub_0; i_0++) {
      stride_0_0 = in1_size_idx_0 * i;
      in1_data[i_0 + stride_0_0] = observing_B.in1_data[i_0 + stride_0_0];
    }
  }

  /* End of MATLAB Function: '<S5>/MATLAB Function' */
}

static void observing_binary_expand_op_10(real_T in1[36], const int8_T in2_data[],
  const int32_T in2_size[2], const real_T in3[18], const
  struct_dZd4pOWAPoKYGFDw6kFnYF *in4)
{
  int32_T aux_0_1;
  int32_T i;
  int32_T i_0;
  int32_T i_1;
  int32_T stride_0_0;
  int32_T stride_0_1;

  /* MATLAB Function: '<S5>/MATLAB Function1' */
  stride_0_0 = (in2_size[0] != 1);
  stride_0_1 = (in2_size[1] != 1);
  aux_0_1 = 0;
  for (i = 0; i < 6; i++) {
    for (i_0 = 0; i_0 < 6; i_0++) {
      observing_B.in3_p[i_0 + 6 * i] = 0.0;
    }

    for (i_0 = 0; i_0 < 3; i_0++) {
      real_T tmp;
      tmp = in4->Cd[3 * i + i_0];
      for (i_1 = 0; i_1 < 6; i_1++) {
        int32_T in3_tmp;
        in3_tmp = 6 * i + i_1;
        observing_B.in3_p[in3_tmp] += in3[6 * i_0 + i_1] * tmp;
      }
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      i_1 = 6 * i + i_0;
      in1[i_1] = (real_T)in2_data[i_0 * stride_0_0 + in2_size[0] * aux_0_1] -
        observing_B.in3_p[i_1];
    }

    aux_0_1 += stride_0_1;
  }

  /* End of MATLAB Function: '<S5>/MATLAB Function1' */
}

/* Function for MATLAB Function: '<S5>/MATLAB Function1' */
static void observing_ellipke(real_T m, real_T *k, real_T *e)
{
  real_T a1;
  real_T b1;
  if (m == 1.0) {
    *k = (rtInf);
    *e = 1.0;
  } else {
    observing_B.a0_c = 1.0;
    observing_B.b0_p = sqrt(1.0 - m);
    observing_B.s0_p = m;
    observing_B.i1_a = 0.0;
    observing_B.w1_e = 1.0;
    a1 = 1.0;
    while (observing_B.w1_e > 2.2204460492503131E-16) {
      a1 = (observing_B.a0_c + observing_B.b0_p) / 2.0;
      b1 = sqrt(observing_B.a0_c * observing_B.b0_p);
      observing_B.a0_c = (observing_B.a0_c - observing_B.b0_p) / 2.0;
      observing_B.i1_a++;
      observing_B.w1_e = observing_rt_powd_snf(2.0, observing_B.i1_a) *
        observing_B.a0_c * observing_B.a0_c;
      observing_B.s0_p += observing_B.w1_e;
      observing_B.a0_c = a1;
      observing_B.b0_p = b1;
    }

    *k = 3.1415926535897931 / (2.0 * a1);
    *e = (1.0 - observing_B.s0_p / 2.0) * *k;
  }
}

/* Function for MATLAB Function: '<S5>/MATLAB Function1' */
static void computeFieldCircularWireCarte_o(real_T x, real_T y, real_T z, real_T
  r, real_T b_I, real_T *bx, real_T *by, real_T *bz)
{
  int32_T i;
  int32_T trueCount;
  observing_B.f_idx_0_c = rt_atan2d_snf(y, x);
  observing_B.rho_o = observing_rt_hypotd_snf(x, y);
  observing_B.c_n = 1.2566370614359173E-6 * b_I / (sqrt(r * observing_B.rho_o) *
    12.566370614359172);
  observing_B.a_c = r + observing_B.rho_o;
  observing_B.k2_tmp = z * z;
  observing_B.k2_m = fmin(4.0 * r * observing_B.rho_o / (observing_B.a_c *
    observing_B.a_c + observing_B.k2_tmp), 1.0);
  observing_ellipke(observing_B.k2_m, &observing_B.a_c, &observing_B.E);
  observing_B.b_a_m = observing_B.rho_o - r;
  observing_B.k2_m = sqrt(observing_B.k2_m);
  observing_B.d_tmp_m = observing_B.rho_o * observing_B.rho_o;
  observing_B.d_tmp_j = r * r;
  observing_B.b_a_m = observing_B.b_a_m * observing_B.b_a_m + observing_B.k2_tmp;
  observing_B.d_idx_0_h = (observing_B.a_c - ((observing_B.d_tmp_m +
    observing_B.d_tmp_j) + observing_B.k2_tmp) / observing_B.b_a_m *
    observing_B.E) * (-(z / observing_B.rho_o) * observing_B.c_n *
                      observing_B.k2_m);
  *bz = (observing_B.a_c - ((observing_B.d_tmp_m - observing_B.d_tmp_j) +
          observing_B.k2_tmp) / observing_B.b_a_m * observing_B.E) *
    (observing_B.c_n * observing_B.k2_m);
  trueCount = 0;
  if (observing_B.rho_o < 1.0E-6) {
    for (i = 0; i < 1; i++) {
      trueCount++;
    }

    *bz = observing_B.d_tmp_j * 1.2566370614359173E-6 * b_I /
      (observing_rt_powd_snf(observing_B.k2_tmp + observing_B.d_tmp_j, 1.5) *
       2.0);
  }

  if (trueCount - 1 >= 0) {
    observing_B.f_idx_0_c = 0.0;
    observing_B.d_idx_0_h = 0.0;
  }

  *bx = observing_B.d_idx_0_h * cos(observing_B.f_idx_0_c);
  *by = observing_B.d_idx_0_h * sin(observing_B.f_idx_0_c);
}

/* Function for MATLAB Function: '<S5>/MATLAB Function1' */
static void o_maglevSystemMeasurements_xred(const real_T x[6], const real_T u[4],
  real_T y[3])
{
  int32_T i;
  int32_T trueCount;
  computeFieldCircularWireCarte_o(-0.025048737341529168, -0.024748737341529166,
    -0.0042, 0.01, 7321.1273822271851, &observing_B.bxBase, &observing_B.byBase,
    &observing_B.bzBase);
  computeFieldCircularWireCarte_o(0.024448737341529164, -0.024748737341529166,
    -0.0042, 0.01, 7321.1273822271851, &observing_B.bxTemp_j, &observing_B.c_o,
    &observing_B.bzTemp_j);
  observing_B.bxBase += observing_B.bxTemp_j;
  observing_B.byBase += observing_B.c_o;
  observing_B.bzBase += observing_B.bzTemp_j;
  computeFieldCircularWireCarte_o(-0.025048737341529168, 0.024748737341529166,
    -0.0042, 0.01, 7321.1273822271851, &observing_B.bxTemp_j, &observing_B.c_o,
    &observing_B.bzTemp_j);
  observing_B.bxBase += observing_B.bxTemp_j;
  observing_B.byBase += observing_B.c_o;
  observing_B.bzBase += observing_B.bzTemp_j;
  computeFieldCircularWireCarte_o(0.024448737341529164, 0.024748737341529166,
    -0.0042, 0.01, 7321.1273822271851, &observing_B.bxTemp_j, &observing_B.c_o,
    &observing_B.bzTemp_j);
  computeFieldCircularWireCarte_o(-0.020300000000000002, 0.0, -0.006,
    0.0052206519382138588, u[0], &observing_B.b_bxTemp, &observing_B.b_byTemp,
    &observing_B.k2_n);
  observing_B.bxBase = (observing_B.bxBase + observing_B.bxTemp_j) +
    observing_B.b_bxTemp * 480.0;
  observing_B.byBase = (observing_B.byBase + observing_B.c_o) +
    observing_B.b_byTemp * 480.0;
  observing_B.bzBase = (observing_B.bzBase + observing_B.bzTemp_j) +
    observing_B.k2_n * 480.0;
  computeFieldCircularWireCarte_o(-0.0003, -0.02, -0.006, 0.0052206519382138588,
    u[1], &observing_B.b_bxTemp, &observing_B.b_byTemp, &observing_B.k2_n);
  observing_B.bxBase += observing_B.b_bxTemp * 480.0;
  observing_B.byBase += observing_B.b_byTemp * 480.0;
  observing_B.bzBase += observing_B.k2_n * 480.0;
  computeFieldCircularWireCarte_o(0.0197, 0.0, -0.006, 0.0052206519382138588, u
    [2], &observing_B.b_bxTemp, &observing_B.b_byTemp, &observing_B.k2_n);
  observing_B.bxBase += observing_B.b_bxTemp * 480.0;
  observing_B.byBase += observing_B.b_byTemp * 480.0;
  observing_B.bzBase += observing_B.k2_n * 480.0;
  computeFieldCircularWireCarte_o(-0.0003, 0.02, -0.006, 0.0052206519382138588,
    u[3], &observing_B.b_bxTemp, &observing_B.b_byTemp, &observing_B.k2_n);
  observing_B.bxBase += observing_B.b_bxTemp * 480.0;
  observing_B.byBase += observing_B.b_byTemp * 480.0;
  observing_B.bzBase += observing_B.k2_n * 480.0;
  observing_B.f_idx_0 = rt_atan2d_snf(x[1], x[0] - -0.0003);
  observing_B.bxTemp_j = observing_rt_hypotd_snf(x[0] - -0.0003, x[1]);
  observing_B.c_o = 0.0022 / (sqrt(0.025 * observing_B.bxTemp_j) *
    12.566370614359172);
  observing_B.bzTemp_j = x[2] * x[2];
  observing_B.k2_n = fmin(0.1 * observing_B.bxTemp_j / ((observing_B.bxTemp_j +
    0.025) * (observing_B.bxTemp_j + 0.025) + observing_B.bzTemp_j), 1.0);
  observing_ellipke(observing_B.k2_n, &observing_B.b_bxTemp,
                    &observing_B.b_byTemp);
  observing_B.k2_n = sqrt(observing_B.k2_n);
  observing_B.d_tmp = observing_B.bxTemp_j * observing_B.bxTemp_j;
  observing_B.d_tmp_i = (observing_B.bxTemp_j - 0.025) * (observing_B.bxTemp_j -
    0.025) + observing_B.bzTemp_j;
  observing_B.d_idx_0 = (observing_B.b_bxTemp - ((observing_B.d_tmp +
    0.00062500000000000012) + observing_B.bzTemp_j) / observing_B.d_tmp_i *
    observing_B.b_byTemp) * (-(x[2] / observing_B.bxTemp_j) * observing_B.c_o *
    observing_B.k2_n);
  observing_B.c_o = (observing_B.b_bxTemp - ((observing_B.d_tmp -
    0.00062500000000000012) + observing_B.bzTemp_j) / observing_B.d_tmp_i *
                     observing_B.b_byTemp) * (observing_B.c_o * observing_B.k2_n);
  trueCount = 0;
  if (observing_B.bxTemp_j < 1.0E-6) {
    for (i = 0; i < 1; i++) {
      trueCount++;
    }

    observing_B.c_o = 1.3750000000000004E-6 / (observing_rt_powd_snf
      (observing_B.bzTemp_j + 0.00062500000000000012, 1.5) * 2.0);
  }

  if (trueCount - 1 >= 0) {
    observing_B.f_idx_0 = 0.0;
    observing_B.d_idx_0 = 0.0;
  }

  y[0] = observing_B.d_idx_0 * cos(observing_B.f_idx_0) + observing_B.bxBase;
  y[1] = observing_B.d_idx_0 * sin(observing_B.f_idx_0) + observing_B.byBase;
  y[2] = observing_B.bzBase + observing_B.c_o;
}

static void observing_binary_expand_op_7(real_T in1_data[], const int32_T
  in1_size[2], int32_T in2, const struct_dZd4pOWAPoKYGFDw6kFnYF *in3, const
  real_T in4_data[], const int32_T *in4_size, real_T in5)
{
  int32_T i;
  int32_T stride_0_0;

  /* MATLAB Function: '<S5>/MATLAB Function1' */
  stride_0_0 = (*in4_size != 1);
  for (i = 0; i < 6; i++) {
    observing_B.in3_l[i] = in4_data[i * stride_0_0] + in3->xLp[i];
  }

  o_maglevSystemMeasurements_xred(observing_B.in3_l, in3->uLp, observing_B.dv3);
  for (i = 0; i < 6; i++) {
    observing_B.in3_l[i] = in3->xLp[i] - in4_data[i * stride_0_0];
  }

  o_maglevSystemMeasurements_xred(observing_B.in3_l, in3->uLp, observing_B.dv4);
  observing_B.d5 = 2.0 * in5;
  stride_0_0 = in1_size[0];
  for (i = 0; i < stride_0_0; i++) {
    in1_data[i + in1_size[0] * in2] = (observing_B.dv3[i] - observing_B.dv4[i]) /
      observing_B.d5;
  }

  /* End of MATLAB Function: '<S5>/MATLAB Function1' */
}

static void observing_binary_expand_op_9(real_T in1[9], const real_T in2_data[],
  const int32_T in2_size[2], const real_T in3[36], const
  struct_dZd4pOWAPoKYGFDw6kFnYF *in4)
{
  real_T tmp;
  int32_T aux_0_1;
  int32_T i;
  int32_T i_0;
  int32_T in2_data_tmp;
  int32_T in2_size_idx_0;
  int32_T stride_0_0;
  int32_T tmp_0;

  /* MATLAB Function: '<S5>/MATLAB Function1' */
  i = in2_size[0];
  in2_size_idx_0 = in2_size[0];
  for (i_0 = 0; i_0 < 6; i_0++) {
    for (stride_0_0 = 0; stride_0_0 < i; stride_0_0++) {
      observing_B.in2_data_l[stride_0_0 + in2_size_idx_0 * i_0] = 0.0;
    }

    for (stride_0_0 = 0; stride_0_0 < 6; stride_0_0++) {
      tmp = in3[6 * i_0 + stride_0_0];
      for (aux_0_1 = 0; aux_0_1 < i; aux_0_1++) {
        in2_data_tmp = in2_size_idx_0 * i_0 + aux_0_1;
        observing_B.in2_data_l[in2_data_tmp] += in2_data[in2_size[0] *
          stride_0_0 + aux_0_1] * tmp;
      }
    }
  }

  in2_size_idx_0 = in2_size[0];
  for (i_0 = 0; i_0 < i; i_0++) {
    for (stride_0_0 = 0; stride_0_0 < i; stride_0_0++) {
      observing_B.in2_data_o[stride_0_0 + in2_size_idx_0 * i_0] = 0.0;
    }

    for (stride_0_0 = 0; stride_0_0 < 6; stride_0_0++) {
      tmp_0 = in2_size[0] * stride_0_0;
      tmp = in2_data[i_0 + tmp_0];
      for (aux_0_1 = 0; aux_0_1 < i; aux_0_1++) {
        in2_data_tmp = in2_size_idx_0 * i_0 + aux_0_1;
        observing_B.in2_data_o[in2_data_tmp] += observing_B.in2_data_l[aux_0_1 +
          tmp_0] * tmp;
      }
    }
  }

  stride_0_0 = (in2_size[0] != 1);
  aux_0_1 = 0;
  for (i = 0; i < 3; i++) {
    tmp_0 = in2_size_idx_0 * aux_0_1;
    in1[3 * i] = in4->R[3 * i] + observing_B.in2_data_o[tmp_0];
    i_0 = 3 * i + 1;
    in1[i_0] = observing_B.in2_data_o[stride_0_0 + tmp_0] + in4->R[i_0];
    i_0 = 3 * i + 2;
    in1[i_0] = observing_B.in2_data_o[(stride_0_0 << 1) + tmp_0] + in4->R[i_0];
    aux_0_1 += stride_0_0;
  }

  /* End of MATLAB Function: '<S5>/MATLAB Function1' */
}

static void observing_binary_expand_op_8(real_T in1_data[], int32_T in1_size[2],
  const int8_T in2_data[], const int32_T in2_size[2], const real_T in3[18],
  const real_T in4_data[], const int32_T in4_size[2], const real_T in5[36],
  const struct_dZd4pOWAPoKYGFDw6kFnYF *in6)
{
  int32_T aux_0_1;
  int32_T i;
  int32_T i_0;
  int32_T i_1;
  int32_T in3_data_tmp;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_0_1;
  int32_T stride_1_1;

  /* MATLAB Function: '<S5>/MATLAB Function1' */
  i = in4_size[1];
  for (i_0 = 0; i_0 < i; i_0++) {
    for (i_1 = 0; i_1 < 6; i_1++) {
      observing_B.in3_data_p[i_1 + 6 * i_0] = 0.0;
    }

    for (i_1 = 0; i_1 < 3; i_1++) {
      observing_B.d4 = in4_data[in4_size[0] * i_0 + i_1];
      for (aux_0_1 = 0; aux_0_1 < 6; aux_0_1++) {
        in3_data_tmp = 6 * i_0 + aux_0_1;
        observing_B.in3_data_p[in3_data_tmp] += in3[6 * i_1 + aux_0_1] *
          observing_B.d4;
      }
    }
  }

  loop_ub = in4_size[1] == 1 ? in2_size[1] : in4_size[1];
  stride_0_0 = (in2_size[0] != 1);
  stride_0_1 = (in2_size[1] != 1);
  stride_1_1 = (in4_size[1] != 1);
  aux_0_1 = 0;
  in3_data_tmp = 0;
  for (i_0 = 0; i_0 < loop_ub; i_0++) {
    for (i_1 = 0; i_1 < 6; i_1++) {
      observing_B.in2_data_c[i_1 + 6 * i_0] = (real_T)in2_data[i_1 * stride_0_0
        + in2_size[0] * aux_0_1] - observing_B.in3_data_p[6 * in3_data_tmp + i_1];
    }

    in3_data_tmp += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  for (i_0 = 0; i_0 < i; i_0++) {
    for (i_1 = 0; i_1 < 6; i_1++) {
      observing_B.in3_data_p[i_1 + 6 * i_0] = 0.0;
    }

    for (i_1 = 0; i_1 < 3; i_1++) {
      observing_B.d4 = in4_data[in4_size[0] * i_0 + i_1];
      for (aux_0_1 = 0; aux_0_1 < 6; aux_0_1++) {
        in3_data_tmp = 6 * i_0 + aux_0_1;
        observing_B.in3_data_p[in3_data_tmp] += in3[6 * i_1 + aux_0_1] *
          observing_B.d4;
      }
    }
  }

  aux_0_1 = 0;
  for (i = 0; i < 6; i++) {
    for (i_0 = 0; i_0 < loop_ub; i_0++) {
      observing_B.in2_data_f[i_0 + loop_ub * i] = (real_T)in2_data[i_0 *
        stride_0_1 * in2_size[0] + aux_0_1] - observing_B.in3_data_p[i_0 *
        stride_1_1 * 6 + i];
    }

    aux_0_1 += stride_0_0;
    for (i_0 = 0; i_0 < 6; i_0++) {
      observing_B.in2[i_0 + 6 * i] = 0.0;
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      observing_B.d4 = in5[6 * i + i_0];
      for (i_1 = 0; i_1 < 6; i_1++) {
        in3_data_tmp = 6 * i + i_1;
        observing_B.in2[in3_data_tmp] += observing_B.in2_data_c[6 * i_0 + i_1] *
          observing_B.d4;
      }
    }

    observing_B.in3_d[i] = 0.0;
    observing_B.in3_d[i + 6] = 0.0;
    observing_B.in3_d[i + 12] = 0.0;
  }

  for (i = 0; i < 3; i++) {
    for (i_0 = 0; i_0 < 3; i_0++) {
      observing_B.d4 = in6->R[3 * i + i_0];
      for (i_1 = 0; i_1 < 6; i_1++) {
        aux_0_1 = 6 * i + i_1;
        observing_B.in3_d[aux_0_1] += in3[6 * i_0 + i_1] * observing_B.d4;
      }
    }
  }

  in1_size[0] = 6;
  in1_size[1] = 6;
  for (i = 0; i < 6; i++) {
    for (i_0 = 0; i_0 < 6; i_0++) {
      observing_B.in2_n[i_0 + 6 * i] = 0.0;
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      i_1 = 6 * i + i_0;
      observing_B.d4 = observing_B.in2_data_f[i_1];
      for (aux_0_1 = 0; aux_0_1 < 6; aux_0_1++) {
        in3_data_tmp = 6 * i + aux_0_1;
        observing_B.in2_n[in3_data_tmp] += observing_B.in2[6 * i_0 + aux_0_1] *
          observing_B.d4;
      }

      observing_B.in3[i_1] = 0.0;
    }

    for (i_0 = 0; i_0 < 3; i_0++) {
      observing_B.d4 = in3[6 * i_0 + i];
      for (i_1 = 0; i_1 < 6; i_1++) {
        aux_0_1 = 6 * i + i_1;
        observing_B.in3[aux_0_1] += observing_B.in3_d[6 * i_0 + i_1] *
          observing_B.d4;
      }
    }

    for (i_0 = 0; i_0 < 6; i_0++) {
      i_1 = 6 * i + i_0;
      in1_data[i_1] = observing_B.in2_n[i_1] + observing_B.in3[i_1];
    }
  }

  /* End of MATLAB Function: '<S5>/MATLAB Function1' */
}

/* Function for MATLAB Function: '<S5>/MATLAB Function1' */
static void observing_chol(real_T A[36])
{
  int32_T b_j;
  int32_T b_k;
  int32_T c_tmp;
  int32_T idxAjj;
  int32_T jmax;
  boolean_T exitg1;
  jmax = 0;
  b_j = 0;
  exitg1 = false;
  while ((!exitg1) && (b_j < 6)) {
    real_T c;
    idxAjj = b_j * 6 + b_j;
    observing_B.ssq_a = 0.0;
    if (b_j >= 1) {
      for (b_k = 0; b_k < b_j; b_k++) {
        c = A[b_k * 6 + b_j];
        observing_B.ssq_a += c * c;
      }
    }

    observing_B.ssq_a = A[idxAjj] - observing_B.ssq_a;
    if (observing_B.ssq_a > 0.0) {
      observing_B.ssq_a = sqrt(observing_B.ssq_a);
      A[idxAjj] = observing_B.ssq_a;
      if (b_j + 1 < 6) {
        int32_T d;
        if (b_j != 0) {
          d = ((b_j - 1) * 6 + b_j) + 2;
          for (b_k = b_j + 2; b_k <= d; b_k += 6) {
            int32_T e;
            c_tmp = b_k - b_j;
            c = -A[div_nde_s32_floor(c_tmp - 2, 6) * 6 + b_j];
            e = c_tmp + 4;
            for (c_tmp = b_k; c_tmp <= e; c_tmp++) {
              int32_T tmp;
              tmp = ((idxAjj + c_tmp) - b_k) + 1;
              A[tmp] += A[c_tmp - 1] * c;
            }
          }
        }

        observing_B.ssq_a = 1.0 / observing_B.ssq_a;
        d = (idxAjj - b_j) + 6;
        for (b_k = idxAjj + 2; b_k <= d; b_k++) {
          A[b_k - 1] *= observing_B.ssq_a;
        }
      }

      b_j++;
    } else {
      A[idxAjj] = observing_B.ssq_a;
      jmax = b_j + 1;
      exitg1 = true;
    }
  }

  if (jmax == 0) {
    jmax = 7;
  }

  for (b_j = 2; b_j < jmax; b_j++) {
    for (idxAjj = 0; idxAjj <= b_j - 2; idxAjj++) {
      A[idxAjj + 6 * (b_j - 1)] = 0.0;
    }
  }
}

static void observing_binary_expand_op_3(real_T in1[9], const
  struct_dZd4pOWAPoKYGFDw6kFnYF *in2, int32_T in3, const real_T in4_data[],
  const int32_T *in4_size)
{
  real_T in2_0;
  int32_T aux_0_1;
  int32_T i;
  int32_T stride_0_0;

  /* MATLAB Function: '<S5>/MATLAB Function1' */
  in2_0 = in2->Wc[in3];
  i = *in4_size;
  for (stride_0_0 = 0; stride_0_0 < i; stride_0_0++) {
    for (aux_0_1 = 0; aux_0_1 < i; aux_0_1++) {
      observing_B.in4_data_b[aux_0_1 + *in4_size * stride_0_0] =
        in4_data[aux_0_1] * in4_data[stride_0_0];
    }
  }

  stride_0_0 = (*in4_size != 1);
  aux_0_1 = 0;
  for (i = 0; i < 3; i++) {
    int32_T in1_tmp;
    int32_T in1_tmp_0;
    in1_tmp_0 = *in4_size * aux_0_1;
    observing_B.in1[3 * i] = in1[3 * i] + in2_0 *
      observing_B.in4_data_b[in1_tmp_0];
    in1_tmp = 3 * i + 1;
    observing_B.in1[in1_tmp] = observing_B.in4_data_b[stride_0_0 + in1_tmp_0] *
      in2_0 + in1[in1_tmp];
    in1_tmp = 3 * i + 2;
    observing_B.in1[in1_tmp] = observing_B.in4_data_b[(stride_0_0 << 1) +
      in1_tmp_0] * in2_0 + in1[in1_tmp];
    aux_0_1 += stride_0_0;
  }

  memcpy(&in1[0], &observing_B.in1[0], 9U * sizeof(real_T));

  /* End of MATLAB Function: '<S5>/MATLAB Function1' */
}

static void observing_binary_expand_op_2(real_T in1_data[], int32_T in1_size[2],
  const struct_dZd4pOWAPoKYGFDw6kFnYF *in2, int32_T in3, const real_T in4[78],
  const real_T in5[6], const real_T in6_data[], const int32_T *in6_size)
{
  real_T in2_0;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T i;
  int32_T i_0;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_0_1;
  int32_T stride_1_1;

  /* MATLAB Function: '<S5>/MATLAB Function1' */
  in2_0 = in2->Wc[in3];
  for (i_0 = 0; i_0 < 6; i_0++) {
    observing_B.in4[i_0] = in4[6 * in3 + i_0] - in5[i_0];
  }

  i_0 = *in6_size;
  for (i = 0; i < i_0; i++) {
    for (loop_ub = 0; loop_ub < 6; loop_ub++) {
      observing_B.in4_data_g[loop_ub + 6 * i] = observing_B.in4[loop_ub] *
        in6_data[i];
    }
  }

  loop_ub = *in6_size == 1 ? in1_size[1] : *in6_size;
  stride_0_0 = (in1_size[0] != 1);
  stride_0_1 = (in1_size[1] != 1);
  stride_1_1 = (*in6_size != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i_0 = 0; i_0 < loop_ub; i_0++) {
    for (i = 0; i < 6; i++) {
      observing_B.in1_data_j[i + 6 * i_0] = observing_B.in4_data_g[6 * aux_1_1 +
        i] * in2_0 + in1_data[i * stride_0_0 + in1_size[0] * aux_0_1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  in1_size[0] = 6;
  in1_size[1] = loop_ub;
  for (i_0 = 0; i_0 < loop_ub; i_0++) {
    for (i = 0; i < 6; i++) {
      stride_0_0 = 6 * i_0;
      in1_data[i + stride_0_0] = observing_B.in1_data_j[i + stride_0_0];
    }
  }

  /* End of MATLAB Function: '<S5>/MATLAB Function1' */
}

static void observing_binary_expand_op_6(real_T in1_data[], int32_T *in1_size,
  const real_T in2[6], const real_T in3_data[], const int32_T in3_size[2], const
  real_T in4[3], const real_T in5_data[], const int32_T *in5_size)
{
  int32_T i;
  int32_T i_0;
  int32_T stride_0_0;

  /* MATLAB Function: '<S5>/MATLAB Function1' */
  stride_0_0 = (*in5_size != 1);
  observing_B.in4_h[0] = in4[0] - in5_data[0];
  observing_B.in4_h[1] = in4[1] - in5_data[stride_0_0];
  observing_B.in4_h[2] = in4[2] - in5_data[stride_0_0 << 1];
  i = in3_size[0];
  if (i - 1 >= 0) {
    memset(&observing_B.in3_data_lx[0], 0, (uint32_T)i * sizeof(real_T));
  }

  for (stride_0_0 = 0; stride_0_0 < 3; stride_0_0++) {
    real_T tmp;
    tmp = observing_B.in4_h[stride_0_0];
    for (i_0 = 0; i_0 < i; i_0++) {
      observing_B.in3_data_lx[i_0] += in3_data[in3_size[0] * stride_0_0 + i_0] *
        tmp;
    }
  }

  *in1_size = 6;
  stride_0_0 = (in3_size[0] != 1);
  for (i = 0; i < 6; i++) {
    in1_data[i] = observing_B.in3_data_lx[i * stride_0_0] + in2[i];
  }

  /* End of MATLAB Function: '<S5>/MATLAB Function1' */
}

static void observing_binary_expand_op_5(real_T in1_data[], int32_T in1_size[2],
  const real_T in2[36], const real_T in3_data[], const int32_T in3_size[2],
  const real_T in4[9])
{
  real_T tmp;
  int32_T aux_0_1;
  int32_T i;
  int32_T i_0;
  int32_T in3_data_tmp;
  int32_T in3_size_idx_0;
  int32_T stride_0_0;
  int32_T tmp_0;

  /* MATLAB Function: '<S5>/MATLAB Function1' */
  i = in3_size[0];
  in3_size_idx_0 = in3_size[0];
  for (i_0 = 0; i_0 < 3; i_0++) {
    for (stride_0_0 = 0; stride_0_0 < i; stride_0_0++) {
      observing_B.in3_data_l[stride_0_0 + in3_size_idx_0 * i_0] = 0.0;
    }

    for (stride_0_0 = 0; stride_0_0 < 3; stride_0_0++) {
      tmp = in4[3 * i_0 + stride_0_0];
      for (aux_0_1 = 0; aux_0_1 < i; aux_0_1++) {
        in3_data_tmp = in3_size_idx_0 * i_0 + aux_0_1;
        observing_B.in3_data_l[in3_data_tmp] += in3_data[in3_size[0] *
          stride_0_0 + aux_0_1] * tmp;
      }
    }
  }

  in3_size_idx_0 = in3_size[0];
  for (i_0 = 0; i_0 < i; i_0++) {
    for (stride_0_0 = 0; stride_0_0 < i; stride_0_0++) {
      observing_B.in3_data[stride_0_0 + in3_size_idx_0 * i_0] = 0.0;
    }

    for (stride_0_0 = 0; stride_0_0 < 3; stride_0_0++) {
      tmp_0 = in3_size[0] * stride_0_0;
      tmp = in3_data[i_0 + tmp_0];
      for (aux_0_1 = 0; aux_0_1 < i; aux_0_1++) {
        in3_data_tmp = in3_size_idx_0 * i_0 + aux_0_1;
        observing_B.in3_data[in3_data_tmp] += observing_B.in3_data_l[aux_0_1 +
          tmp_0] * tmp;
      }
    }
  }

  in1_size[0] = 6;
  in1_size[1] = 6;
  stride_0_0 = (in3_size[0] != 1);
  aux_0_1 = 0;
  for (i = 0; i < 6; i++) {
    for (i_0 = 0; i_0 < 6; i_0++) {
      tmp_0 = 6 * i + i_0;
      in1_data[tmp_0] = in2[tmp_0] - observing_B.in3_data[i_0 * stride_0_0 +
        in3_size_idx_0 * aux_0_1];
    }

    aux_0_1 += stride_0_0;
  }

  /* End of MATLAB Function: '<S5>/MATLAB Function1' */
}

/* Model step function */
void observing_step(void)
{
  int32_T b_z_pred_size;
  int32_T e_size;
  boolean_T exitg1;

  /* S-Function (sfun_MagLevTbx_MagSens_v43): '<S10>/S-Function' */
  /* S-Function Block: <S10>/S-Function */

  /*  Get measurement of mag sensor  */
  sfun_MagLevTbx_MagSens_WrappedOutput(&observing_B.SFunction[0]);

  /* S-Function (sfun_MagLevTbx_CurrSens): '<S1>/S-Function' */

  /* S-Function Block: <S1>/S-Function */

  /*  Get measurement of current sensor 0U  */
  sfun_MagLevTbx_CurrSens_WrappedOutput(0U, &observing_B.SFunction_o);

  /* S-Function (sfun_MagLevTbx_CurrSens): '<S2>/S-Function' */

  /* S-Function Block: <S2>/S-Function */

  /*  Get measurement of current sensor 1U  */
  sfun_MagLevTbx_CurrSens_WrappedOutput(1U, &observing_B.SFunction_g);

  /* S-Function (sfun_MagLevTbx_CurrSens): '<S3>/S-Function' */

  /* S-Function Block: <S3>/S-Function */

  /*  Get measurement of current sensor 2U  */
  sfun_MagLevTbx_CurrSens_WrappedOutput(2U, &observing_B.SFunction_b);

  /* S-Function (sfun_MagLevTbx_CurrSens): '<S4>/S-Function' */

  /* S-Function Block: <S4>/S-Function */

  /*  Get measurement of current sensor 3U  */
  sfun_MagLevTbx_CurrSens_WrappedOutput(3U, &observing_B.SFunction_e);

  /* DataTypeConversion: '<S5>/Cast To Double1' */
  observing_B.CastToDouble1[0] = observing_B.SFunction_o;
  observing_B.CastToDouble1[1] = observing_B.SFunction_g;
  observing_B.CastToDouble1[2] = observing_B.SFunction_b;
  observing_B.CastToDouble1[3] = observing_B.SFunction_e;

  /* MATLAB Function: '<S5>/MATLAB Function' incorporates:
   *  Constant: '<Root>/Constant'
   *  UnitDelay: '<S5>/Unit Delay1'
   */
  switch ((int32_T)observing_P.Constant_Value) {
   case 1:
    for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
      observing_B.cols_n[observing_B.jmax] = 0.0;
    }

    for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
      observing_B.ssq = observing_DW.UnitDelay_DSTATE[observing_B.jmax];
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp
           ++) {
        observing_B.cols_n[observing_B.b_rtemp] += observing_P.obs.Ad[6 *
          observing_B.jmax + observing_B.b_rtemp] * observing_B.ssq;
      }

      observing_B.x_pred[observing_B.jmax] = 0.0;
    }

    for (observing_B.jmax = 0; observing_B.jmax < 4; observing_B.jmax++) {
      observing_B.ssq = observing_B.CastToDouble1[observing_B.jmax];
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp
           ++) {
        observing_B.x_pred[observing_B.b_rtemp] += observing_P.obs.Bd[6 *
          observing_B.jmax + observing_B.b_rtemp] * observing_B.ssq;
      }
    }

    for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
      observing_B.x_pred_temp_data[observing_B.jmax] =
        observing_B.cols_n[observing_B.jmax] +
        observing_B.x_pred[observing_B.jmax];
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp
           ++) {
        observing_B.P_pred[observing_B.b_rtemp + 6 * observing_B.jmax] = 0.0;
      }

      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp
           ++) {
        observing_B.ssq = observing_DW.UnitDelay1_DSTATE[6 * observing_B.jmax +
          observing_B.b_rtemp];
        for (observing_B.idxAjj = 0; observing_B.idxAjj < 6; observing_B.idxAjj
             ++) {
          observing_B.b_r2 = 6 * observing_B.jmax + observing_B.idxAjj;
          observing_B.P_pred[observing_B.b_r2] += observing_P.obs.Ad[6 *
            observing_B.b_rtemp + observing_B.idxAjj] * observing_B.ssq;
        }
      }
    }

    for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp
           ++) {
        observing_B.ssq = 0.0;
        for (observing_B.idxAjj = 0; observing_B.idxAjj < 6; observing_B.idxAjj
             ++) {
          observing_B.ssq += observing_B.P_pred[6 * observing_B.idxAjj +
            observing_B.jmax] * observing_P.obs.Ad[6 * observing_B.idxAjj +
            observing_B.b_rtemp];
        }

        observing_B.idxAjj = 6 * observing_B.b_rtemp + observing_B.jmax;
        observing_B.S_chol_g[observing_B.idxAjj] =
          observing_P.obs.Qd[observing_B.idxAjj] + observing_B.ssq;
      }
    }

    observing_B.P_pred_temp_size[0] = 6;
    memcpy(&observing_B.P_pred_temp_data[0], &observing_B.S_chol_g[0], 36U *
           sizeof(real_T));
    break;

   case 2:
    observing_stateTrans(observing_DW.UnitDelay_DSTATE,
                         observing_B.CastToDouble1, observing_P.obs.dt,
                         observing_B.cols_n);
    for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
      observing_B.x_pred_temp_data[observing_B.jmax] =
        observing_B.cols_n[observing_B.jmax];
    }

    observing_B.a21 = observing_P.obs.delta;
    observing_B.dt = observing_P.obs.dt;
    observing_B.b_rtemp = (int32_T)observing_P.obs.nx;
    observing_B.Ad_size[0] = (int32_T)observing_P.obs.nx;
    observing_B.Ad_size[1] = (int32_T)observing_P.obs.nx;
    for (observing_B.b_r2 = 0; observing_B.b_r2 < observing_B.b_rtemp;
         observing_B.b_r2++) {
      e_size = observing_B.b_rtemp;
      if (observing_B.b_rtemp - 1 >= 0) {
        memset(&observing_B.e_data[0], 0, (uint32_T)observing_B.b_rtemp * sizeof
               (real_T));
      }

      observing_B.e_data[observing_B.b_r2] = observing_B.a21;
      if (observing_B.b_rtemp == 6) {
        for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
          observing_B.cols_n[observing_B.jmax] =
            observing_DW.UnitDelay_DSTATE[observing_B.jmax] +
            observing_B.e_data[observing_B.jmax];
        }

        observing_stateTrans(observing_B.cols_n, observing_B.CastToDouble1,
                             observing_B.dt, observing_B.x_pred);
        for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
          observing_B.cols_n[observing_B.jmax] =
            observing_DW.UnitDelay_DSTATE[observing_B.jmax] -
            observing_B.e_data[observing_B.jmax];
        }

        observing_stateTrans(observing_B.cols_n, observing_B.CastToDouble1,
                             observing_B.dt, observing_B.dv);
        observing_B.ssq = 2.0 * observing_B.a21;
        observing_B.idxAjj = observing_B.Ad_size[0];
        for (observing_B.jmax = 0; observing_B.jmax < observing_B.idxAjj;
             observing_B.jmax++) {
          observing_B.Ad_data[observing_B.jmax + observing_B.Ad_size[0] *
            observing_B.b_r2] = (observing_B.x_pred[observing_B.jmax] -
            observing_B.dv[observing_B.jmax]) / observing_B.ssq;
        }
      } else {
        observing_binary_expand_op_11(observing_B.Ad_data, observing_B.Ad_size,
          observing_B.b_r2, observing_DW.UnitDelay_DSTATE, observing_B.e_data,
          &e_size, observing_B.CastToDouble1, observing_B.dt, observing_B.a21);
      }
    }

    if (observing_B.Ad_size[0] == 6) {
      for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6;
             observing_B.b_rtemp++) {
          observing_B.Ad_data_c[observing_B.b_rtemp + 6 * observing_B.jmax] =
            0.0;
        }

        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6;
             observing_B.b_rtemp++) {
          observing_B.ssq = observing_DW.UnitDelay1_DSTATE[6 * observing_B.jmax
            + observing_B.b_rtemp];
          for (observing_B.idxAjj = 0; observing_B.idxAjj < 6;
               observing_B.idxAjj++) {
            observing_B.b_r2 = 6 * observing_B.jmax + observing_B.idxAjj;
            observing_B.Ad_data_c[observing_B.b_r2] +=
              observing_B.Ad_data[observing_B.Ad_size[0] * observing_B.b_rtemp +
              observing_B.idxAjj] * observing_B.ssq;
          }
        }
      }

      for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6;
             observing_B.b_rtemp++) {
          observing_B.ssq = 0.0;
          for (observing_B.idxAjj = 0; observing_B.idxAjj < 6;
               observing_B.idxAjj++) {
            observing_B.ssq += observing_B.Ad_data_c[6 * observing_B.idxAjj +
              observing_B.jmax] * observing_B.Ad_data[observing_B.Ad_size[0] *
              observing_B.idxAjj + observing_B.b_rtemp];
          }

          observing_B.b_r2 = 6 * observing_B.b_rtemp + observing_B.jmax;
          observing_B.P_pred[observing_B.b_r2] =
            observing_P.obs.Qd[observing_B.b_r2] + observing_B.ssq;
        }
      }

      observing_B.P_pred_temp_size[0] = 6;
      memcpy(&observing_B.P_pred_temp_data[0], &observing_B.P_pred[0], 36U *
             sizeof(real_T));
    } else {
      observing_binary_expand_op_12(observing_B.P_pred_temp_data,
        observing_B.P_pred_temp_size, observing_B.Ad_data, observing_B.Ad_size,
        observing_DW.UnitDelay1_DSTATE, &observing_P.obs);
    }
    break;

   case 3:
    observing_B.ssq = observing_P.obs.nx + observing_P.obs.lambda;
    for (observing_B.jmax = 0; observing_B.jmax < 36; observing_B.jmax++) {
      observing_DW.UnitDelay1_DSTATE[observing_B.jmax] *= observing_B.ssq;
    }

    observing_B.jmax = 0;
    observing_B.b_r2 = 0;
    exitg1 = false;
    while ((!exitg1) && (observing_B.b_r2 < 6)) {
      observing_B.idxAjj = observing_B.b_r2 * 6 + observing_B.b_r2;
      observing_B.ssq = 0.0;
      if (observing_B.b_r2 >= 1) {
        for (observing_B.r1 = 0; observing_B.r1 < observing_B.b_r2;
             observing_B.r1++) {
          observing_B.a21 = observing_DW.UnitDelay1_DSTATE[observing_B.r1 * 6 +
            observing_B.b_r2];
          observing_B.ssq += observing_B.a21 * observing_B.a21;
        }
      }

      observing_B.ssq = observing_DW.UnitDelay1_DSTATE[observing_B.idxAjj] -
        observing_B.ssq;
      if (observing_B.ssq > 0.0) {
        observing_B.ssq = sqrt(observing_B.ssq);
        observing_DW.UnitDelay1_DSTATE[observing_B.idxAjj] = observing_B.ssq;
        if (observing_B.b_r2 + 1 < 6) {
          if (observing_B.b_r2 != 0) {
            observing_B.b_rtemp = ((observing_B.b_r2 - 1) * 6 + observing_B.b_r2)
              + 2;
            for (observing_B.r1 = observing_B.b_r2 + 2; observing_B.r1 <=
                 observing_B.b_rtemp; observing_B.r1 += 6) {
              observing_B.r2 = observing_B.r1 - observing_B.b_r2;
              observing_B.a21 =
                -observing_DW.UnitDelay1_DSTATE[div_nde_s32_floor(observing_B.r2
                - 2, 6) * 6 + observing_B.b_r2];
              e_size = observing_B.r2 + 4;
              for (observing_B.r3 = observing_B.r1; observing_B.r3 <= e_size;
                   observing_B.r3++) {
                observing_B.r2 = ((observing_B.idxAjj + observing_B.r3) -
                                  observing_B.r1) + 1;
                observing_DW.UnitDelay1_DSTATE[observing_B.r2] +=
                  observing_DW.UnitDelay1_DSTATE[observing_B.r3 - 1] *
                  observing_B.a21;
              }
            }
          }

          observing_B.ssq = 1.0 / observing_B.ssq;
          observing_B.r1 = (observing_B.idxAjj - observing_B.b_r2) + 6;
          for (observing_B.b_rtemp = observing_B.idxAjj + 2; observing_B.b_rtemp
               <= observing_B.r1; observing_B.b_rtemp++) {
            observing_DW.UnitDelay1_DSTATE[observing_B.b_rtemp - 1] *=
              observing_B.ssq;
          }
        }

        observing_B.b_r2++;
      } else {
        observing_DW.UnitDelay1_DSTATE[observing_B.idxAjj] = observing_B.ssq;
        observing_B.jmax = observing_B.b_r2 + 1;
        exitg1 = true;
      }
    }

    if (observing_B.jmax == 0) {
      observing_B.jmax = 7;
    }

    for (observing_B.b_rtemp = 2; observing_B.b_rtemp < observing_B.jmax;
         observing_B.b_rtemp++) {
      for (observing_B.b_r2 = 0; observing_B.b_r2 <= observing_B.b_rtemp - 2;
           observing_B.b_r2++) {
        observing_DW.UnitDelay1_DSTATE[observing_B.b_r2 + 6 *
          (observing_B.b_rtemp - 1)] = 0.0;
      }
    }

    for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp
           ++) {
        observing_B.ssq = observing_DW.UnitDelay_DSTATE[observing_B.b_rtemp];
        observing_B.r2 = 6 * observing_B.jmax + observing_B.b_rtemp;
        observing_B.a21 = observing_DW.UnitDelay1_DSTATE[observing_B.r2];
        observing_B.P_pred[observing_B.r2] = observing_B.ssq + observing_B.a21;
        observing_B.S_chol_g[observing_B.r2] = observing_B.ssq - observing_B.a21;
      }

      observing_B.cols[observing_B.jmax] =
        observing_DW.UnitDelay_DSTATE[observing_B.jmax];
    }

    for (observing_B.jmax = 0; observing_B.jmax < 36; observing_B.jmax++) {
      observing_B.cols[observing_B.jmax + 6] =
        observing_B.P_pred[observing_B.jmax];
      observing_B.cols[observing_B.jmax + 42] =
        observing_B.S_chol_g[observing_B.jmax];
    }

    observing_B.idxAjj = (int32_T)observing_P.obs.nx;
    observing_B.sigma_pred_size[0] = (int32_T)observing_P.obs.nx;
    observing_B.b_r2 = (int32_T)(2.0 * observing_P.obs.nx + 1.0);
    observing_B.sigma_pred_size[1] = observing_B.b_r2;
    observing_B.b_rtemp = observing_B.b_r2 * (int32_T)observing_P.obs.nx;
    if (observing_B.b_rtemp - 1 >= 0) {
      memset(&observing_B.sigma_pred_data[0], 0, (uint32_T)observing_B.b_rtemp *
             sizeof(real_T));
    }

    for (observing_B.b_rtemp = 0; observing_B.b_rtemp < observing_B.b_r2;
         observing_B.b_rtemp++) {
      observing_stateTrans(&observing_B.cols[6 * observing_B.b_rtemp],
                           observing_B.CastToDouble1, observing_P.obs.dt,
                           observing_B.cols_n);
      for (observing_B.jmax = 0; observing_B.jmax < observing_B.idxAjj;
           observing_B.jmax++) {
        observing_B.sigma_pred_data[observing_B.jmax + observing_B.idxAjj *
          observing_B.b_rtemp] = observing_B.cols_n[observing_B.jmax];
      }
    }

    observing_B.x_pred_temp_size = (int32_T)observing_P.obs.nx;
    if (observing_B.idxAjj - 1 >= 0) {
      memset(&observing_B.x_pred_temp_data[0], 0, (uint32_T)observing_B.idxAjj *
             sizeof(real_T));
    }

    for (observing_B.jmax = 0; observing_B.jmax < 13; observing_B.jmax++) {
      observing_B.ssq = observing_P.obs.Wm[observing_B.jmax];
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < observing_B.idxAjj;
           observing_B.b_rtemp++) {
        observing_B.x_pred_temp_data[observing_B.b_rtemp] +=
          observing_B.sigma_pred_data[observing_B.idxAjj * observing_B.jmax +
          observing_B.b_rtemp] * observing_B.ssq;
      }
    }

    observing_B.P_pred_temp_size[0] = 6;
    observing_B.P_pred_temp_size[1] = 6;
    memcpy(&observing_B.P_pred_temp_data[0], &observing_P.obs.Qd[0], 36U *
           sizeof(real_T));
    for (observing_B.r1 = 0; observing_B.r1 < observing_B.b_r2; observing_B.r1++)
    {
      observing_B.b_rtemp = observing_B.sigma_pred_size[0];
      if (observing_B.sigma_pred_size[0] == observing_B.x_pred_temp_size) {
        e_size = observing_B.sigma_pred_size[0];
        for (observing_B.jmax = 0; observing_B.jmax < observing_B.b_rtemp;
             observing_B.jmax++) {
          observing_B.e_data[observing_B.jmax] =
            observing_B.sigma_pred_data[observing_B.sigma_pred_size[0] *
            observing_B.r1 + observing_B.jmax] -
            observing_B.x_pred_temp_data[observing_B.jmax];
        }
      } else {
        observing_binary_expand_op_1(observing_B.e_data, &e_size,
          observing_B.sigma_pred_data, observing_B.sigma_pred_size,
          observing_B.r1, observing_B.x_pred_temp_data,
          &observing_B.x_pred_temp_size);
      }

      if ((observing_B.P_pred_temp_size[0] == e_size) && (e_size ==
           observing_B.P_pred_temp_size[1])) {
        observing_B.ssq = observing_P.obs.Wc[observing_B.r1];
        observing_B.jmax = e_size;
        observing_B.Ad_size[0] = e_size;
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < observing_B.jmax;
             observing_B.b_rtemp++) {
          for (observing_B.idxAjj = 0; observing_B.idxAjj < observing_B.jmax;
               observing_B.idxAjj++) {
            observing_B.Ad_data[observing_B.idxAjj + observing_B.Ad_size[0] *
              observing_B.b_rtemp] = observing_B.e_data[observing_B.idxAjj] *
              observing_B.e_data[observing_B.b_rtemp];
          }
        }

        observing_B.idxAjj = observing_B.P_pred_temp_size[0] *
          observing_B.P_pred_temp_size[1];
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < observing_B.idxAjj;
             observing_B.b_rtemp++) {
          observing_B.P_pred_temp_data[observing_B.b_rtemp] +=
            observing_B.Ad_data[observing_B.b_rtemp] * observing_B.ssq;
        }
      } else {
        observing_binary_expand_op(observing_B.P_pred_temp_data,
          observing_B.P_pred_temp_size, &observing_P.obs, observing_B.r1,
          observing_B.e_data, &e_size);
      }
    }
    break;
  }

  for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
    observing_B.x_pred[observing_B.jmax] =
      observing_B.x_pred_temp_data[observing_B.jmax];
    for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp++)
    {
      observing_B.P_pred[observing_B.b_rtemp + 6 * observing_B.jmax] =
        observing_B.P_pred_temp_data[observing_B.P_pred_temp_size[0] *
        observing_B.jmax + observing_B.b_rtemp];
    }
  }

  /* End of MATLAB Function: '<S5>/MATLAB Function' */

  /* DataTypeConversion: '<S10>/Data Type Conversion' */
  observing_B.DataTypeConversion = observing_B.SFunction[0];

  /* DataTypeConversion: '<S10>/Data Type Conversion1' */
  observing_B.DataTypeConversion1 = observing_B.SFunction[1];

  /* DataTypeConversion: '<S10>/Data Type Conversion2' */
  observing_B.DataTypeConversion2 = observing_B.SFunction[2];

  /* MATLAB Function: '<S5>/MATLAB Function1' incorporates:
   *  Constant: '<Root>/Constant'
   *  SignalConversion generated from: '<S8>/ SFunction '
   */
  observing_B.meas[0] = 0.001 * observing_B.DataTypeConversion;
  observing_B.meas[1] = 0.001 * observing_B.DataTypeConversion1;
  observing_B.meas[2] = -0.001 * observing_B.DataTypeConversion2;
  e_size = (int32_T)observing_P.obs.nx;
  observing_B.In_size[0] = (int32_T)observing_P.obs.nx;
  observing_B.In_size[1] = (int32_T)observing_P.obs.nx;
  observing_B.idxAjj = (int32_T)observing_P.obs.nx * (int32_T)observing_P.obs.nx;
  if (observing_B.idxAjj - 1 >= 0) {
    memset(&observing_B.In_data_d[0], 0, (uint32_T)observing_B.idxAjj * sizeof
           (int8_T));
  }

  if ((int32_T)observing_P.obs.nx > 0) {
    observing_B.jmax = (uint8_T)(int32_T)observing_P.obs.nx;
    for (observing_B.r1 = 0; observing_B.r1 < observing_B.jmax; observing_B.r1++)
    {
      observing_B.In_data_d[observing_B.r1 + e_size * observing_B.r1] = 1;
    }
  }

  switch ((int32_T)observing_P.Constant_Value) {
   case 1:
    for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
      observing_B.L[observing_B.jmax] = observing_P.obs.Cd[3 * observing_B.jmax];
      observing_B.b_r2 = 3 * observing_B.jmax + 1;
      observing_B.L[observing_B.jmax + 6] = observing_P.obs.Cd[observing_B.b_r2];
      observing_B.idxAjj = 3 * observing_B.jmax + 2;
      observing_B.L[observing_B.jmax + 12] =
        observing_P.obs.Cd[observing_B.idxAjj];
      observing_B.ssq = 0.0;
      observing_B.dt = 0.0;
      observing_B.a21 = 0.0;
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp
           ++) {
        observing_B.d = observing_B.P_pred[6 * observing_B.jmax +
          observing_B.b_rtemp];
        observing_B.ssq += observing_P.obs.Cd[3 * observing_B.b_rtemp] *
          observing_B.d;
        observing_B.dt += observing_P.obs.Cd[3 * observing_B.b_rtemp + 1] *
          observing_B.d;
        observing_B.a21 += observing_P.obs.Cd[3 * observing_B.b_rtemp + 2] *
          observing_B.d;
      }

      observing_B.B[observing_B.idxAjj] = observing_B.a21;
      observing_B.B[observing_B.b_r2] = observing_B.dt;
      observing_B.B[3 * observing_B.jmax] = observing_B.ssq;
    }

    for (observing_B.jmax = 0; observing_B.jmax < 3; observing_B.jmax++) {
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 3; observing_B.b_rtemp
           ++) {
        observing_B.ssq = 0.0;
        for (observing_B.idxAjj = 0; observing_B.idxAjj < 6; observing_B.idxAjj
             ++) {
          observing_B.ssq += observing_B.B[3 * observing_B.idxAjj +
            observing_B.jmax] * observing_B.L[6 * observing_B.b_rtemp +
            observing_B.idxAjj];
        }

        observing_B.b_r2 = 3 * observing_B.b_rtemp + observing_B.jmax;
        observing_B.b_A[observing_B.b_r2] = observing_P.obs.R[observing_B.b_r2]
          + observing_B.ssq;
      }
    }

    for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
      observing_B.B[observing_B.jmax] = 0.0;
      observing_B.B[observing_B.jmax + 6] = 0.0;
      observing_B.B[observing_B.jmax + 12] = 0.0;
    }

    for (observing_B.jmax = 0; observing_B.jmax < 3; observing_B.jmax++) {
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp
           ++) {
        observing_B.ssq = observing_B.L[6 * observing_B.jmax +
          observing_B.b_rtemp];
        for (observing_B.idxAjj = 0; observing_B.idxAjj < 6; observing_B.idxAjj
             ++) {
          observing_B.b_r2 = 6 * observing_B.jmax + observing_B.idxAjj;
          observing_B.B[observing_B.b_r2] += observing_B.P_pred[6 *
            observing_B.b_rtemp + observing_B.idxAjj] * observing_B.ssq;
        }
      }
    }

    observing_B.r1 = 0;
    observing_B.r2 = 1;
    observing_B.r3 = 2;
    observing_B.ssq = fabs(observing_B.b_A[0]);
    observing_B.a21 = fabs(observing_B.b_A[1]);
    if (observing_B.a21 > observing_B.ssq) {
      observing_B.ssq = observing_B.a21;
      observing_B.r1 = 1;
      observing_B.r2 = 0;
    }

    if (fabs(observing_B.b_A[2]) > observing_B.ssq) {
      observing_B.r1 = 2;
      observing_B.r2 = 1;
      observing_B.r3 = 0;
    }

    observing_B.b_A[observing_B.r2] /= observing_B.b_A[observing_B.r1];
    observing_B.b_A[observing_B.r3] /= observing_B.b_A[observing_B.r1];
    observing_B.b_A[observing_B.r2 + 3] -= observing_B.b_A[observing_B.r1 + 3] *
      observing_B.b_A[observing_B.r2];
    observing_B.b_A[observing_B.r3 + 3] -= observing_B.b_A[observing_B.r1 + 3] *
      observing_B.b_A[observing_B.r3];
    observing_B.b_A[observing_B.r2 + 6] -= observing_B.b_A[observing_B.r1 + 6] *
      observing_B.b_A[observing_B.r2];
    observing_B.b_A[observing_B.r3 + 6] -= observing_B.b_A[observing_B.r1 + 6] *
      observing_B.b_A[observing_B.r3];
    if (fabs(observing_B.b_A[observing_B.r3 + 3]) > fabs
        (observing_B.b_A[observing_B.r2 + 3])) {
      observing_B.jmax = observing_B.r2;
      observing_B.r2 = observing_B.r3;
      observing_B.r3 = observing_B.jmax;
    }

    observing_B.b_A[observing_B.r3 + 3] /= observing_B.b_A[observing_B.r2 + 3];
    observing_B.b_A[observing_B.r3 + 6] -= observing_B.b_A[observing_B.r3 + 3] *
      observing_B.b_A[observing_B.r2 + 6];
    for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp++)
    {
      observing_B.b_r2 = 6 * observing_B.r1 + observing_B.b_rtemp;
      observing_B.L[observing_B.b_r2] = observing_B.B[observing_B.b_rtemp] /
        observing_B.b_A[observing_B.r1];
      observing_B.idxAjj = 6 * observing_B.r2 + observing_B.b_rtemp;
      observing_B.L[observing_B.idxAjj] = observing_B.B[observing_B.b_rtemp + 6]
        - observing_B.b_A[observing_B.r1 + 3] * observing_B.L[observing_B.b_r2];
      observing_B.jmax = 6 * observing_B.r3 + observing_B.b_rtemp;
      observing_B.L[observing_B.jmax] = observing_B.B[observing_B.b_rtemp + 12]
        - observing_B.b_A[observing_B.r1 + 6] * observing_B.L[observing_B.b_r2];
      observing_B.L[observing_B.idxAjj] /= observing_B.b_A[observing_B.r2 + 3];
      observing_B.L[observing_B.jmax] -= observing_B.b_A[observing_B.r2 + 6] *
        observing_B.L[observing_B.idxAjj];
      observing_B.L[observing_B.jmax] /= observing_B.b_A[observing_B.r3 + 6];
      observing_B.L[observing_B.idxAjj] -= observing_B.b_A[observing_B.r3 + 3] *
        observing_B.L[observing_B.jmax];
      observing_B.L[observing_B.b_r2] -= observing_B.L[observing_B.jmax] *
        observing_B.b_A[observing_B.r3];
      observing_B.L[observing_B.b_r2] -= observing_B.L[observing_B.idxAjj] *
        observing_B.b_A[observing_B.r2];
    }

    for (observing_B.jmax = 0; observing_B.jmax < 3; observing_B.jmax++) {
      observing_B.ssq = 0.0;
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp
           ++) {
        observing_B.ssq += observing_P.obs.Cd[3 * observing_B.b_rtemp +
          observing_B.jmax] * observing_B.x_pred[observing_B.b_rtemp];
      }

      observing_B.b_z_pred_data[observing_B.jmax] =
        observing_B.meas[observing_B.jmax] - observing_B.ssq;
    }

    observing_B.ssq = 0.0;
    observing_B.dt = 0.0;
    observing_B.a21 = 0.0;
    for (observing_B.jmax = 0; observing_B.jmax < 4; observing_B.jmax++) {
      observing_B.d = observing_B.CastToDouble1[observing_B.jmax];
      observing_B.ssq += observing_P.obs.Dd[3 * observing_B.jmax] *
        observing_B.d;
      observing_B.dt += observing_P.obs.Dd[3 * observing_B.jmax + 1] *
        observing_B.d;
      observing_B.a21 += observing_P.obs.Dd[3 * observing_B.jmax + 2] *
        observing_B.d;
    }

    observing_B.ssq = observing_B.b_z_pred_data[0] - observing_B.ssq;
    observing_B.dt = observing_B.b_z_pred_data[1] - observing_B.dt;
    observing_B.a21 = observing_B.b_z_pred_data[2] - observing_B.a21;
    for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
      observing_B.e_data_d[observing_B.jmax] = ((observing_B.L[observing_B.jmax
        + 6] * observing_B.dt + observing_B.L[observing_B.jmax] *
        observing_B.ssq) + observing_B.L[observing_B.jmax + 12] *
        observing_B.a21) + observing_B.x_pred[observing_B.jmax];
    }

    if ((int32_T)observing_P.obs.nx == 6) {
      for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
        observing_B.ssq = observing_P.obs.Cd[3 * observing_B.jmax + 1];
        observing_B.a21 = observing_P.obs.Cd[3 * observing_B.jmax];
        observing_B.dt = observing_P.obs.Cd[3 * observing_B.jmax + 2];
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6;
             observing_B.b_rtemp++) {
          observing_B.S_chol[observing_B.b_rtemp + 6 * observing_B.jmax] =
            (real_T)observing_B.In_data_d[e_size * observing_B.jmax +
            observing_B.b_rtemp] - ((observing_B.L[observing_B.b_rtemp + 6] *
            observing_B.ssq + observing_B.a21 *
            observing_B.L[observing_B.b_rtemp]) +
            observing_B.L[observing_B.b_rtemp + 12] * observing_B.dt);
        }
      }
    } else {
      observing_binary_expand_op_10(observing_B.S_chol, observing_B.In_data_d,
        observing_B.In_size, observing_B.L, &observing_P.obs);
    }

    for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp
           ++) {
        observing_B.S_chol_g[observing_B.b_rtemp + 6 * observing_B.jmax] = 0.0;
      }

      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp
           ++) {
        observing_B.ssq = observing_B.P_pred[6 * observing_B.jmax +
          observing_B.b_rtemp];
        for (observing_B.idxAjj = 0; observing_B.idxAjj < 6; observing_B.idxAjj
             ++) {
          observing_B.b_r2 = 6 * observing_B.jmax + observing_B.idxAjj;
          observing_B.S_chol_g[observing_B.b_r2] += observing_B.S_chol[6 *
            observing_B.b_rtemp + observing_B.idxAjj] * observing_B.ssq;
        }
      }

      observing_B.B[observing_B.jmax] = 0.0;
      observing_B.B[observing_B.jmax + 6] = 0.0;
      observing_B.B[observing_B.jmax + 12] = 0.0;
    }

    for (observing_B.jmax = 0; observing_B.jmax < 3; observing_B.jmax++) {
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 3; observing_B.b_rtemp
           ++) {
        observing_B.ssq = observing_P.obs.R[3 * observing_B.jmax +
          observing_B.b_rtemp];
        for (observing_B.idxAjj = 0; observing_B.idxAjj < 6; observing_B.idxAjj
             ++) {
          observing_B.b_r2 = 6 * observing_B.jmax + observing_B.idxAjj;
          observing_B.B[observing_B.b_r2] += observing_B.L[6 *
            observing_B.b_rtemp + observing_B.idxAjj] * observing_B.ssq;
        }
      }
    }

    for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp
           ++) {
        observing_B.S_chol_m[observing_B.b_rtemp + 6 * observing_B.jmax] = 0.0;
      }

      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp
           ++) {
        observing_B.ssq = observing_B.S_chol[6 * observing_B.b_rtemp +
          observing_B.jmax];
        for (observing_B.idxAjj = 0; observing_B.idxAjj < 6; observing_B.idxAjj
             ++) {
          observing_B.b_r2 = 6 * observing_B.jmax + observing_B.idxAjj;
          observing_B.S_chol_m[observing_B.b_r2] += observing_B.S_chol_g[6 *
            observing_B.b_rtemp + observing_B.idxAjj] * observing_B.ssq;
        }

        observing_B.P_pred[observing_B.b_rtemp + 6 * observing_B.jmax] = 0.0;
      }

      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 3; observing_B.b_rtemp
           ++) {
        observing_B.ssq = observing_B.L[6 * observing_B.b_rtemp +
          observing_B.jmax];
        for (observing_B.idxAjj = 0; observing_B.idxAjj < 6; observing_B.idxAjj
             ++) {
          observing_B.b_r2 = 6 * observing_B.jmax + observing_B.idxAjj;
          observing_B.P_pred[observing_B.b_r2] += observing_B.B[6 *
            observing_B.b_rtemp + observing_B.idxAjj] * observing_B.ssq;
        }
      }
    }

    observing_B.P_est_temp_size[0] = 6;
    for (observing_B.jmax = 0; observing_B.jmax < 36; observing_B.jmax++) {
      observing_B.P_est_temp_data[observing_B.jmax] =
        observing_B.S_chol_m[observing_B.jmax] +
        observing_B.P_pred[observing_B.jmax];
    }
    break;

   case 2:
    observing_B.a21 = observing_P.obs.delta;
    observing_B.Cd_size[0] = (int32_T)observing_P.obs.nz;
    observing_B.Cd_size[1] = (int32_T)observing_P.obs.nx;
    for (observing_B.b_r2 = 0; observing_B.b_r2 < e_size; observing_B.b_r2++) {
      if (e_size - 1 >= 0) {
        memset(&observing_B.e_data_d[0], 0, (uint32_T)e_size * sizeof(real_T));
      }

      observing_B.e_data_d[observing_B.b_r2] = observing_B.a21;
      if (e_size == 6) {
        for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
          observing_B.cols_n[observing_B.jmax] =
            observing_P.obs.xLp[observing_B.jmax] +
            observing_B.e_data_d[observing_B.jmax];
        }

        o_maglevSystemMeasurements_xred(observing_B.cols_n, observing_P.obs.uLp,
          observing_B.b_z_pred_data);
        for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
          observing_B.cols_n[observing_B.jmax] =
            observing_P.obs.xLp[observing_B.jmax] -
            observing_B.e_data_d[observing_B.jmax];
        }

        o_maglevSystemMeasurements_xred(observing_B.cols_n, observing_P.obs.uLp,
          observing_B.dy_data);
        observing_B.ssq = 2.0 * observing_B.a21;
        observing_B.idxAjj = observing_B.Cd_size[0];
        for (observing_B.jmax = 0; observing_B.jmax < observing_B.idxAjj;
             observing_B.jmax++) {
          observing_B.Cd_data[observing_B.jmax + observing_B.Cd_size[0] *
            observing_B.b_r2] = (observing_B.b_z_pred_data[observing_B.jmax] -
            observing_B.dy_data[observing_B.jmax]) / observing_B.ssq;
        }
      } else {
        observing_binary_expand_op_7(observing_B.Cd_data, observing_B.Cd_size,
          observing_B.b_r2, &observing_P.obs, observing_B.e_data_d, &e_size,
          observing_B.a21);
      }
    }

    if (observing_B.Cd_size[0] == 3) {
      for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 3;
             observing_B.b_rtemp++) {
          observing_B.L[observing_B.b_rtemp + 3 * observing_B.jmax] = 0.0;
        }

        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6;
             observing_B.b_rtemp++) {
          observing_B.ssq = observing_B.P_pred[6 * observing_B.jmax +
            observing_B.b_rtemp];
          for (observing_B.idxAjj = 0; observing_B.idxAjj < 3;
               observing_B.idxAjj++) {
            observing_B.b_r2 = 3 * observing_B.jmax + observing_B.idxAjj;
            observing_B.L[observing_B.b_r2] +=
              observing_B.Cd_data[observing_B.Cd_size[0] * observing_B.b_rtemp +
              observing_B.idxAjj] * observing_B.ssq;
          }
        }
      }

      for (observing_B.jmax = 0; observing_B.jmax < 3; observing_B.jmax++) {
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 3;
             observing_B.b_rtemp++) {
          observing_B.ssq = 0.0;
          for (observing_B.idxAjj = 0; observing_B.idxAjj < 6;
               observing_B.idxAjj++) {
            observing_B.ssq += observing_B.L[3 * observing_B.idxAjj +
              observing_B.jmax] * observing_B.Cd_data[observing_B.Cd_size[0] *
              observing_B.idxAjj + observing_B.b_rtemp];
          }

          observing_B.b_r2 = 3 * observing_B.b_rtemp + observing_B.jmax;
          observing_B.b_A[observing_B.b_r2] = observing_P.obs.R[observing_B.b_r2]
            + observing_B.ssq;
        }
      }
    } else {
      observing_binary_expand_op_9(observing_B.b_A, observing_B.Cd_data,
        observing_B.Cd_size, observing_B.P_pred, &observing_P.obs);
    }

    observing_B.jmax = observing_B.Cd_size[0];
    for (observing_B.b_rtemp = 0; observing_B.b_rtemp < observing_B.jmax;
         observing_B.b_rtemp++) {
      for (observing_B.idxAjj = 0; observing_B.idxAjj < 6; observing_B.idxAjj++)
      {
        observing_B.B[observing_B.idxAjj + 6 * observing_B.b_rtemp] = 0.0;
      }

      for (observing_B.idxAjj = 0; observing_B.idxAjj < 6; observing_B.idxAjj++)
      {
        observing_B.ssq = observing_B.Cd_data[observing_B.Cd_size[0] *
          observing_B.idxAjj + observing_B.b_rtemp];
        for (observing_B.b_r2 = 0; observing_B.b_r2 < 6; observing_B.b_r2++) {
          observing_B.r1 = 6 * observing_B.b_rtemp + observing_B.b_r2;
          observing_B.B[observing_B.r1] += observing_B.P_pred[6 *
            observing_B.idxAjj + observing_B.b_r2] * observing_B.ssq;
        }
      }
    }

    observing_B.r1 = 0;
    observing_B.r2 = 1;
    observing_B.r3 = 2;
    observing_B.ssq = fabs(observing_B.b_A[0]);
    observing_B.a21 = fabs(observing_B.b_A[1]);
    if (observing_B.a21 > observing_B.ssq) {
      observing_B.ssq = observing_B.a21;
      observing_B.r1 = 1;
      observing_B.r2 = 0;
    }

    if (fabs(observing_B.b_A[2]) > observing_B.ssq) {
      observing_B.r1 = 2;
      observing_B.r2 = 1;
      observing_B.r3 = 0;
    }

    observing_B.b_A[observing_B.r2] /= observing_B.b_A[observing_B.r1];
    observing_B.b_A[observing_B.r3] /= observing_B.b_A[observing_B.r1];
    observing_B.b_A[observing_B.r2 + 3] -= observing_B.b_A[observing_B.r1 + 3] *
      observing_B.b_A[observing_B.r2];
    observing_B.b_A[observing_B.r3 + 3] -= observing_B.b_A[observing_B.r1 + 3] *
      observing_B.b_A[observing_B.r3];
    observing_B.b_A[observing_B.r2 + 6] -= observing_B.b_A[observing_B.r1 + 6] *
      observing_B.b_A[observing_B.r2];
    observing_B.b_A[observing_B.r3 + 6] -= observing_B.b_A[observing_B.r1 + 6] *
      observing_B.b_A[observing_B.r3];
    if (fabs(observing_B.b_A[observing_B.r3 + 3]) > fabs
        (observing_B.b_A[observing_B.r2 + 3])) {
      observing_B.jmax = observing_B.r2;
      observing_B.r2 = observing_B.r3;
      observing_B.r3 = observing_B.jmax;
    }

    observing_B.b_A[observing_B.r3 + 3] /= observing_B.b_A[observing_B.r2 + 3];
    observing_B.b_A[observing_B.r3 + 6] -= observing_B.b_A[observing_B.r3 + 3] *
      observing_B.b_A[observing_B.r2 + 6];
    for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp++)
    {
      observing_B.b_r2 = 6 * observing_B.r1 + observing_B.b_rtemp;
      observing_B.L[observing_B.b_r2] = observing_B.B[observing_B.b_rtemp] /
        observing_B.b_A[observing_B.r1];
      observing_B.idxAjj = 6 * observing_B.r2 + observing_B.b_rtemp;
      observing_B.L[observing_B.idxAjj] = observing_B.B[observing_B.b_rtemp + 6]
        - observing_B.b_A[observing_B.r1 + 3] * observing_B.L[observing_B.b_r2];
      observing_B.jmax = 6 * observing_B.r3 + observing_B.b_rtemp;
      observing_B.L[observing_B.jmax] = observing_B.B[observing_B.b_rtemp + 12]
        - observing_B.b_A[observing_B.r1 + 6] * observing_B.L[observing_B.b_r2];
      observing_B.L[observing_B.idxAjj] /= observing_B.b_A[observing_B.r2 + 3];
      observing_B.L[observing_B.jmax] -= observing_B.b_A[observing_B.r2 + 6] *
        observing_B.L[observing_B.idxAjj];
      observing_B.L[observing_B.jmax] /= observing_B.b_A[observing_B.r3 + 6];
      observing_B.L[observing_B.idxAjj] -= observing_B.b_A[observing_B.r3 + 3] *
        observing_B.L[observing_B.jmax];
      observing_B.L[observing_B.b_r2] -= observing_B.L[observing_B.jmax] *
        observing_B.b_A[observing_B.r3];
      observing_B.L[observing_B.b_r2] -= observing_B.L[observing_B.idxAjj] *
        observing_B.b_A[observing_B.r2];
    }

    o_maglevSystemMeasurements_xred(observing_B.x_pred,
      observing_B.CastToDouble1, observing_B.b_z_pred_data);
    observing_B.ssq = observing_B.meas[0] - observing_B.b_z_pred_data[0];
    observing_B.dt = observing_B.meas[1] - observing_B.b_z_pred_data[1];
    observing_B.a21 = observing_B.meas[2] - observing_B.b_z_pred_data[2];
    for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
      observing_B.e_data_d[observing_B.jmax] = ((observing_B.L[observing_B.jmax
        + 6] * observing_B.dt + observing_B.L[observing_B.jmax] *
        observing_B.ssq) + observing_B.L[observing_B.jmax + 12] *
        observing_B.a21) + observing_B.x_pred[observing_B.jmax];
    }

    if (((int32_T)observing_P.obs.nx == 6) && ((int32_T)observing_P.obs.nx ==
         observing_B.Cd_size[1]) && ((int32_T)observing_P.obs.nx == 6) &&
        ((int32_T)observing_P.obs.nx == observing_B.Cd_size[1])) {
      observing_B.b_r2 = observing_B.Cd_size[1];
      for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < observing_B.b_r2;
             observing_B.b_rtemp++) {
          observing_B.idxAjj = observing_B.Cd_size[0] * observing_B.b_rtemp;
          observing_B.ssq = (real_T)observing_B.In_data_d[e_size *
            observing_B.b_rtemp + observing_B.jmax] -
            ((observing_B.L[observing_B.jmax + 6] * observing_B.Cd_data[1 +
              observing_B.idxAjj] + observing_B.L[observing_B.jmax] *
              observing_B.Cd_data[observing_B.idxAjj]) +
             observing_B.L[observing_B.jmax + 12] * observing_B.Cd_data[2 +
             observing_B.idxAjj]);
          observing_B.idxAjj = 6 * observing_B.b_rtemp + observing_B.jmax;
          observing_B.In_data[observing_B.idxAjj] = observing_B.ssq;
          observing_B.In_data_b[observing_B.idxAjj] = observing_B.ssq;
        }
      }

      e_size = observing_B.Cd_size[1];
      for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < observing_B.b_r2;
             observing_B.b_rtemp++) {
          observing_B.Ad_data_c[observing_B.b_rtemp + e_size * observing_B.jmax]
            = observing_B.In_data_b[6 * observing_B.b_rtemp + observing_B.jmax];
        }

        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6;
             observing_B.b_rtemp++) {
          observing_B.S_chol[observing_B.b_rtemp + 6 * observing_B.jmax] = 0.0;
        }

        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6;
             observing_B.b_rtemp++) {
          observing_B.ssq = observing_B.P_pred[6 * observing_B.jmax +
            observing_B.b_rtemp];
          for (observing_B.idxAjj = 0; observing_B.idxAjj < 6;
               observing_B.idxAjj++) {
            observing_B.r1 = 6 * observing_B.jmax + observing_B.idxAjj;
            observing_B.S_chol[observing_B.r1] += observing_B.In_data[6 *
              observing_B.b_rtemp + observing_B.idxAjj] * observing_B.ssq;
          }
        }

        observing_B.B[observing_B.jmax] = 0.0;
        observing_B.B[observing_B.jmax + 6] = 0.0;
        observing_B.B[observing_B.jmax + 12] = 0.0;
      }

      for (observing_B.jmax = 0; observing_B.jmax < 3; observing_B.jmax++) {
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 3;
             observing_B.b_rtemp++) {
          observing_B.ssq = observing_P.obs.R[3 * observing_B.jmax +
            observing_B.b_rtemp];
          for (observing_B.idxAjj = 0; observing_B.idxAjj < 6;
               observing_B.idxAjj++) {
            observing_B.b_r2 = 6 * observing_B.jmax + observing_B.idxAjj;
            observing_B.B[observing_B.b_r2] += observing_B.L[6 *
              observing_B.b_rtemp + observing_B.idxAjj] * observing_B.ssq;
          }
        }
      }

      for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6;
             observing_B.b_rtemp++) {
          observing_B.S_chol_g[observing_B.b_rtemp + 6 * observing_B.jmax] = 0.0;
        }

        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6;
             observing_B.b_rtemp++) {
          observing_B.idxAjj = 6 * observing_B.jmax + observing_B.b_rtemp;
          observing_B.ssq = observing_B.Ad_data_c[observing_B.idxAjj];
          for (observing_B.b_r2 = 0; observing_B.b_r2 < 6; observing_B.b_r2++) {
            observing_B.r1 = 6 * observing_B.jmax + observing_B.b_r2;
            observing_B.S_chol_g[observing_B.r1] += observing_B.S_chol[6 *
              observing_B.b_rtemp + observing_B.b_r2] * observing_B.ssq;
          }

          observing_B.P_pred[observing_B.idxAjj] = 0.0;
        }

        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 3;
             observing_B.b_rtemp++) {
          observing_B.ssq = observing_B.L[6 * observing_B.b_rtemp +
            observing_B.jmax];
          for (observing_B.idxAjj = 0; observing_B.idxAjj < 6;
               observing_B.idxAjj++) {
            observing_B.b_r2 = 6 * observing_B.jmax + observing_B.idxAjj;
            observing_B.P_pred[observing_B.b_r2] += observing_B.B[6 *
              observing_B.b_rtemp + observing_B.idxAjj] * observing_B.ssq;
          }
        }
      }

      observing_B.P_est_temp_size[0] = 6;
      for (observing_B.jmax = 0; observing_B.jmax < 36; observing_B.jmax++) {
        observing_B.P_est_temp_data[observing_B.jmax] =
          observing_B.S_chol_g[observing_B.jmax] +
          observing_B.P_pred[observing_B.jmax];
      }
    } else {
      observing_binary_expand_op_8(observing_B.P_est_temp_data,
        observing_B.P_est_temp_size, observing_B.In_data_d, observing_B.In_size,
        observing_B.L, observing_B.Cd_data, observing_B.Cd_size,
        observing_B.P_pred, &observing_P.obs);
    }
    break;

   case 3:
    observing_B.ssq = observing_P.obs.nx + observing_P.obs.lambda;
    for (observing_B.jmax = 0; observing_B.jmax < 36; observing_B.jmax++) {
      observing_B.S_chol[observing_B.jmax] = observing_B.ssq *
        observing_B.P_pred[observing_B.jmax];
    }

    observing_chol(observing_B.S_chol);
    for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp
           ++) {
        observing_B.ssq = observing_B.x_pred[observing_B.b_rtemp];
        observing_B.b_r2 = 6 * observing_B.jmax + observing_B.b_rtemp;
        observing_B.a21 = observing_B.S_chol[observing_B.b_r2];
        observing_B.S_chol_g[observing_B.b_r2] = observing_B.ssq +
          observing_B.a21;
        observing_B.S_chol_m[observing_B.b_r2] = observing_B.ssq -
          observing_B.a21;
      }

      observing_B.cols[observing_B.jmax] = observing_B.x_pred[observing_B.jmax];
    }

    for (observing_B.jmax = 0; observing_B.jmax < 36; observing_B.jmax++) {
      observing_B.cols[observing_B.jmax + 6] =
        observing_B.S_chol_g[observing_B.jmax];
      observing_B.cols[observing_B.jmax + 42] =
        observing_B.S_chol_m[observing_B.jmax];
    }

    observing_B.idxAjj = (int32_T)observing_P.obs.nz;
    observing_B.Cd_size[0] = (int32_T)observing_P.obs.nz;
    observing_B.b_r2 = (int32_T)(2.0 * observing_P.obs.nx + 1.0);
    observing_B.Cd_size[1] = observing_B.b_r2;
    observing_B.b_rtemp = observing_B.b_r2 * (int32_T)observing_P.obs.nz;
    if (observing_B.b_rtemp - 1 >= 0) {
      memset(&observing_B.b_gamma_data[0], 0, (uint32_T)observing_B.b_rtemp *
             sizeof(real_T));
    }

    for (observing_B.b_rtemp = 0; observing_B.b_rtemp < observing_B.b_r2;
         observing_B.b_rtemp++) {
      o_maglevSystemMeasurements_xred(&observing_B.cols[6 * observing_B.b_rtemp],
        observing_B.CastToDouble1, observing_B.b_z_pred_data);
      for (observing_B.jmax = 0; observing_B.jmax < observing_B.idxAjj;
           observing_B.jmax++) {
        observing_B.b_gamma_data[observing_B.jmax + observing_B.idxAjj *
          observing_B.b_rtemp] = observing_B.b_z_pred_data[observing_B.jmax];
      }
    }

    b_z_pred_size = (int32_T)observing_P.obs.nz;
    if (observing_B.idxAjj - 1 >= 0) {
      memset(&observing_B.b_z_pred_data[0], 0, (uint32_T)observing_B.idxAjj *
             sizeof(real_T));
    }

    for (observing_B.jmax = 0; observing_B.jmax < 13; observing_B.jmax++) {
      observing_B.ssq = observing_P.obs.Wm[observing_B.jmax];
      for (observing_B.b_rtemp = 0; observing_B.b_rtemp < observing_B.idxAjj;
           observing_B.b_rtemp++) {
        observing_B.b_z_pred_data[observing_B.b_rtemp] +=
          observing_B.b_gamma_data[observing_B.idxAjj * observing_B.jmax +
          observing_B.b_rtemp] * observing_B.ssq;
      }
    }

    memcpy(&observing_B.Pyy[0], &observing_P.obs.R[0], 9U * sizeof(real_T));
    observing_B.Pxy_size[0] = (int32_T)observing_P.obs.nx;
    observing_B.Pxy_size[1] = (int32_T)observing_P.obs.nz;
    observing_B.idxAjj = (int32_T)observing_P.obs.nx * (int32_T)
      observing_P.obs.nz;
    if (observing_B.idxAjj - 1 >= 0) {
      memset(&observing_B.Pxy_data[0], 0, (uint32_T)observing_B.idxAjj * sizeof
             (real_T));
    }

    for (observing_B.r1 = 0; observing_B.r1 < observing_B.b_r2; observing_B.r1++)
    {
      observing_B.b_rtemp = observing_B.Cd_size[0];
      if (observing_B.Cd_size[0] == b_z_pred_size) {
        e_size = observing_B.Cd_size[0];
        for (observing_B.jmax = 0; observing_B.jmax < observing_B.b_rtemp;
             observing_B.jmax++) {
          observing_B.dy_data[observing_B.jmax] =
            observing_B.b_gamma_data[observing_B.Cd_size[0] * observing_B.r1 +
            observing_B.jmax] - observing_B.b_z_pred_data[observing_B.jmax];
        }
      } else {
        observing_binary_expand_op_1(observing_B.dy_data, &e_size,
          observing_B.b_gamma_data, observing_B.Cd_size, observing_B.r1,
          observing_B.b_z_pred_data, &b_z_pred_size);
      }

      if (e_size == 3) {
        observing_B.ssq = observing_P.obs.Wc[observing_B.r1];
        for (observing_B.jmax = 0; observing_B.jmax < 3; observing_B.jmax++) {
          for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 3;
               observing_B.b_rtemp++) {
            observing_B.b_A[observing_B.b_rtemp + 3 * observing_B.jmax] =
              observing_B.dy_data[observing_B.b_rtemp] *
              observing_B.dy_data[observing_B.jmax];
          }
        }

        for (observing_B.jmax = 0; observing_B.jmax < 9; observing_B.jmax++) {
          observing_B.Pyy[observing_B.jmax] += observing_B.b_A[observing_B.jmax]
            * observing_B.ssq;
        }
      } else {
        observing_binary_expand_op_3(observing_B.Pyy, &observing_P.obs,
          observing_B.r1, observing_B.dy_data, &e_size);
      }

      if ((observing_B.Pxy_size[0] == 6) && (e_size == observing_B.Pxy_size[1]))
      {
        observing_B.ssq = observing_P.obs.Wc[observing_B.r1];
        for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
          observing_B.cols_n[observing_B.jmax] = observing_B.cols[6 *
            observing_B.r1 + observing_B.jmax] -
            observing_B.x_pred[observing_B.jmax];
        }

        observing_B.jmax = e_size;
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < observing_B.jmax;
             observing_B.b_rtemp++) {
          for (observing_B.idxAjj = 0; observing_B.idxAjj < 6;
               observing_B.idxAjj++) {
            observing_B.B[observing_B.idxAjj + 6 * observing_B.b_rtemp] =
              observing_B.cols_n[observing_B.idxAjj] *
              observing_B.dy_data[observing_B.b_rtemp];
          }
        }

        observing_B.idxAjj = 6 * observing_B.Pxy_size[1];
        observing_B.Pxy_size[0] = 6;
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < observing_B.idxAjj;
             observing_B.b_rtemp++) {
          observing_B.Pxy_data[observing_B.b_rtemp] +=
            observing_B.B[observing_B.b_rtemp] * observing_B.ssq;
        }
      } else {
        observing_binary_expand_op_2(observing_B.Pxy_data, observing_B.Pxy_size,
          &observing_P.obs, observing_B.r1, observing_B.cols, observing_B.x_pred,
          observing_B.dy_data, &e_size);
      }
    }

    if (observing_B.Pxy_size[0] == 0) {
      observing_B.b_L_size[0] = 0;
      observing_B.b_L_size[1] = 3;
    } else {
      memcpy(&observing_B.b_A[0], &observing_B.Pyy[0], 9U * sizeof(real_T));
      observing_B.jmax = 0;
      observing_B.b_r2 = 1;
      observing_B.idxAjj = 2;
      observing_B.ssq = fabs(observing_B.Pyy[0]);
      observing_B.a21 = fabs(observing_B.Pyy[1]);
      if (observing_B.a21 > observing_B.ssq) {
        observing_B.ssq = observing_B.a21;
        observing_B.jmax = 1;
        observing_B.b_r2 = 0;
      }

      if (fabs(observing_B.Pyy[2]) > observing_B.ssq) {
        observing_B.jmax = 2;
        observing_B.b_r2 = 1;
        observing_B.idxAjj = 0;
      }

      observing_B.b_A[observing_B.b_r2] = observing_B.Pyy[observing_B.b_r2] /
        observing_B.Pyy[observing_B.jmax];
      observing_B.b_A[observing_B.idxAjj] /= observing_B.b_A[observing_B.jmax];
      observing_B.b_A[observing_B.b_r2 + 3] -= observing_B.b_A[observing_B.jmax
        + 3] * observing_B.b_A[observing_B.b_r2];
      observing_B.b_A[observing_B.idxAjj + 3] -=
        observing_B.b_A[observing_B.jmax + 3] *
        observing_B.b_A[observing_B.idxAjj];
      observing_B.b_A[observing_B.b_r2 + 6] -= observing_B.b_A[observing_B.jmax
        + 6] * observing_B.b_A[observing_B.b_r2];
      observing_B.b_A[observing_B.idxAjj + 6] -=
        observing_B.b_A[observing_B.jmax + 6] *
        observing_B.b_A[observing_B.idxAjj];
      if (fabs(observing_B.b_A[observing_B.idxAjj + 3]) > fabs
          (observing_B.b_A[observing_B.b_r2 + 3])) {
        observing_B.b_rtemp = observing_B.b_r2;
        observing_B.b_r2 = observing_B.idxAjj;
        observing_B.idxAjj = observing_B.b_rtemp;
      }

      observing_B.b_A[observing_B.idxAjj + 3] /=
        observing_B.b_A[observing_B.b_r2 + 3];
      observing_B.b_A[observing_B.idxAjj + 6] -=
        observing_B.b_A[observing_B.idxAjj + 3] *
        observing_B.b_A[observing_B.b_r2 + 6];
      observing_B.b_rtemp = observing_B.Pxy_size[0];
      observing_B.b_L_size[0] = observing_B.Pxy_size[0];
      observing_B.b_L_size[1] = 3;
      for (observing_B.r1 = 0; observing_B.r1 < observing_B.b_rtemp;
           observing_B.r1++) {
        e_size = observing_B.b_L_size[0] * observing_B.jmax + observing_B.r1;
        observing_B.Cd_data[e_size] = observing_B.Pxy_data[observing_B.r1] /
          observing_B.b_A[observing_B.jmax];
        observing_B.r3 = observing_B.b_L_size[0] * observing_B.b_r2 +
          observing_B.r1;
        observing_B.Cd_data[observing_B.r3] =
          observing_B.Pxy_data[observing_B.r1 + observing_B.Pxy_size[0]] -
          observing_B.b_A[observing_B.jmax + 3] * observing_B.Cd_data[e_size];
        observing_B.r2 = observing_B.b_L_size[0] * observing_B.idxAjj +
          observing_B.r1;
        observing_B.Cd_data[observing_B.r2] = observing_B.Pxy_data
          [(observing_B.Pxy_size[0] << 1) + observing_B.r1] -
          observing_B.b_A[observing_B.jmax + 6] * observing_B.Cd_data[e_size];
        observing_B.Cd_data[observing_B.r3] /= observing_B.b_A[observing_B.b_r2
          + 3];
        observing_B.Cd_data[observing_B.r2] -= observing_B.b_A[observing_B.b_r2
          + 6] * observing_B.Cd_data[observing_B.r3];
        observing_B.Cd_data[observing_B.r2] /=
          observing_B.b_A[observing_B.idxAjj + 6];
        observing_B.Cd_data[observing_B.r3] -=
          observing_B.b_A[observing_B.idxAjj + 3] *
          observing_B.Cd_data[observing_B.r2];
        observing_B.Cd_data[e_size] -= observing_B.Cd_data[observing_B.r2] *
          observing_B.b_A[observing_B.idxAjj];
        observing_B.Cd_data[e_size] -= observing_B.Cd_data[observing_B.r3] *
          observing_B.b_A[observing_B.b_r2];
      }
    }

    if (((int32_T)observing_P.obs.nz == 3) && (observing_B.b_L_size[0] == 6)) {
      observing_B.ssq = observing_B.meas[0] - observing_B.b_z_pred_data[0];
      observing_B.dt = observing_B.meas[1] - observing_B.b_z_pred_data[1];
      observing_B.a21 = observing_B.meas[2] - observing_B.b_z_pred_data[2];
      for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
        observing_B.e_data_d[observing_B.jmax] =
          ((observing_B.Cd_data[observing_B.jmax + observing_B.b_L_size[0]] *
            observing_B.dt + observing_B.Cd_data[observing_B.jmax] *
            observing_B.ssq) + observing_B.Cd_data[(observing_B.b_L_size[0] << 1)
           + observing_B.jmax] * observing_B.a21) +
          observing_B.x_pred[observing_B.jmax];
      }
    } else {
      observing_binary_expand_op_6(observing_B.e_data_d, &e_size,
        observing_B.x_pred, observing_B.Cd_data, observing_B.b_L_size,
        observing_B.meas, observing_B.b_z_pred_data, &b_z_pred_size);
    }

    if (observing_B.b_L_size[0] == 6) {
      for (observing_B.jmax = 0; observing_B.jmax < 3; observing_B.jmax++) {
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6;
             observing_B.b_rtemp++) {
          observing_B.Pxy_data[observing_B.b_rtemp + 6 * observing_B.jmax] = 0.0;
        }

        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 3;
             observing_B.b_rtemp++) {
          observing_B.ssq = observing_B.Pyy[3 * observing_B.jmax +
            observing_B.b_rtemp];
          for (observing_B.idxAjj = 0; observing_B.idxAjj < 6;
               observing_B.idxAjj++) {
            e_size = 6 * observing_B.jmax + observing_B.idxAjj;
            observing_B.Pxy_data[e_size] +=
              observing_B.Cd_data[observing_B.b_L_size[0] * observing_B.b_rtemp
              + observing_B.idxAjj] * observing_B.ssq;
          }
        }
      }

      for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
        observing_B.ssq = observing_B.Cd_data[observing_B.jmax];
        observing_B.a21 = observing_B.Cd_data[observing_B.jmax +
          observing_B.b_L_size[0]];
        observing_B.dt = observing_B.Cd_data[(observing_B.b_L_size[0] << 1) +
          observing_B.jmax];
        for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6;
             observing_B.b_rtemp++) {
          observing_B.b_r2 = 6 * observing_B.jmax + observing_B.b_rtemp;
          observing_B.S_chol[observing_B.b_r2] =
            observing_B.P_pred[observing_B.b_r2] -
            ((observing_B.Pxy_data[observing_B.b_rtemp + 6] * observing_B.a21 +
              observing_B.Pxy_data[observing_B.b_rtemp] * observing_B.ssq) +
             observing_B.Pxy_data[observing_B.b_rtemp + 12] * observing_B.dt);
        }
      }

      observing_B.P_est_temp_size[0] = 6;
      memcpy(&observing_B.P_est_temp_data[0], &observing_B.S_chol[0], 36U *
             sizeof(real_T));
    } else {
      observing_binary_expand_op_5(observing_B.P_est_temp_data,
        observing_B.P_est_temp_size, observing_B.P_pred, observing_B.Cd_data,
        observing_B.b_L_size, observing_B.Pyy);
    }
    break;
  }

  /* Gain: '<Root>/Gain' incorporates:
   *  MATLAB Function: '<S5>/MATLAB Function1'
   */
  observing_B.Gain[0] = observing_P.Gain_Gain * observing_B.e_data_d[0];
  observing_B.Gain[1] = observing_P.Gain_Gain * observing_B.e_data_d[1];
  observing_B.Gain[2] = observing_P.Gain_Gain * observing_B.e_data_d[2];

  /* Gain: '<Root>/Gain1' incorporates:
   *  MATLAB Function: '<S5>/MATLAB Function1'
   */
  observing_B.Gain1[0] = observing_P.Gain1_Gain * observing_B.e_data_d[3];
  observing_B.Gain1[1] = observing_P.Gain1_Gain * observing_B.e_data_d[4];
  observing_B.Gain1[2] = observing_P.Gain1_Gain * observing_B.e_data_d[5];
  for (observing_B.jmax = 0; observing_B.jmax < 6; observing_B.jmax++) {
    /* Update for UnitDelay: '<S5>/Unit Delay1' incorporates:
     *  MATLAB Function: '<S5>/MATLAB Function1'
     */
    for (observing_B.b_rtemp = 0; observing_B.b_rtemp < 6; observing_B.b_rtemp++)
    {
      observing_DW.UnitDelay1_DSTATE[observing_B.b_rtemp + 6 * observing_B.jmax]
        = observing_B.P_est_temp_data[observing_B.P_est_temp_size[0] *
        observing_B.jmax + observing_B.b_rtemp];
    }

    /* End of Update for UnitDelay: '<S5>/Unit Delay1' */

    /* Update for UnitDelay: '<S5>/Unit Delay' incorporates:
     *  MATLAB Function: '<S5>/MATLAB Function1'
     */
    observing_DW.UnitDelay_DSTATE[observing_B.jmax] =
      observing_B.e_data_d[observing_B.jmax];
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  observing_M->Timing.taskTime0 =
    ((time_T)(++observing_M->Timing.clockTick0)) * observing_M->Timing.stepSize0;
}

/* Model initialize function */
void observing_initialize(void)
{
  /* Registration code */
  rtmSetTFinal(observing_M, -1);
  observing_M->Timing.stepSize0 = 0.01;

  /* External mode info */
  observing_M->Sizes.checksums[0] = (2966908483U);
  observing_M->Sizes.checksums[1] = (2390295856U);
  observing_M->Sizes.checksums[2] = (2259109302U);
  observing_M->Sizes.checksums[3] = (3422247059U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[3];
    observing_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(observing_M->extModeInfo,
      &observing_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(observing_M->extModeInfo, observing_M->Sizes.checksums);
    rteiSetTPtr(observing_M->extModeInfo, rtmGetTPtr(observing_M));
  }

  {
    int32_T i;

    /* Start for S-Function (sfun_MagLevTbx_MagSens_v43): '<S10>/S-Function' */

    /* S-Function Block: <S10>/S-Function */

    /*  Initialize mag sensor  */
    sfun_MagLevTbx_MagSens_WrappedStart();

    /* Start for S-Function (sfun_MagLevTbx_CurrSens): '<S1>/S-Function' */

    /* S-Function Block: <S1>/S-Function */

    /*  Initialize current sensors 0U  */
    sfun_MagLevTbx_CurrSens_WrappedStart(0U);

    /* Start for S-Function (sfun_MagLevTbx_CurrSens): '<S2>/S-Function' */

    /* S-Function Block: <S2>/S-Function */

    /*  Initialize current sensors 1U  */
    sfun_MagLevTbx_CurrSens_WrappedStart(1U);

    /* Start for S-Function (sfun_MagLevTbx_CurrSens): '<S3>/S-Function' */

    /* S-Function Block: <S3>/S-Function */

    /*  Initialize current sensors 2U  */
    sfun_MagLevTbx_CurrSens_WrappedStart(2U);

    /* Start for S-Function (sfun_MagLevTbx_CurrSens): '<S4>/S-Function' */

    /* S-Function Block: <S4>/S-Function */

    /*  Initialize current sensors 3U  */
    sfun_MagLevTbx_CurrSens_WrappedStart(3U);

    /* InitializeConditions for UnitDelay: '<S5>/Unit Delay1' */
    memcpy(&observing_DW.UnitDelay1_DSTATE[0],
           &observing_P.UnitDelay1_InitialCondition[0], 36U * sizeof(real_T));

    /* InitializeConditions for UnitDelay: '<S5>/Unit Delay' */
    for (i = 0; i < 6; i++) {
      observing_DW.UnitDelay_DSTATE[i] =
        observing_P.UnitDelay_InitialCondition[i];
    }

    /* End of InitializeConditions for UnitDelay: '<S5>/Unit Delay' */

    /* Enable for S-Function (sfun_MagLevTbx_MagSens_v43): '<S10>/S-Function' */
    /* Level2 S-Function Block: '<S10>/S-Function' (sfun_MagLevTbx_MagSens_v43) */

    /* S-Function Block: <S10>/S-Function */

    /* Enable for S-Function (sfun_MagLevTbx_CurrSens): '<S1>/S-Function' */
    /* Level2 S-Function Block: '<S1>/S-Function' (sfun_MagLevTbx_CurrSens) */

    /* S-Function Block: <S1>/S-Function */

    /* Enable for S-Function (sfun_MagLevTbx_CurrSens): '<S2>/S-Function' */
    /* Level2 S-Function Block: '<S2>/S-Function' (sfun_MagLevTbx_CurrSens) */

    /* S-Function Block: <S2>/S-Function */

    /* Enable for S-Function (sfun_MagLevTbx_CurrSens): '<S3>/S-Function' */
    /* Level2 S-Function Block: '<S3>/S-Function' (sfun_MagLevTbx_CurrSens) */

    /* S-Function Block: <S3>/S-Function */

    /* Enable for S-Function (sfun_MagLevTbx_CurrSens): '<S4>/S-Function' */
    /* Level2 S-Function Block: '<S4>/S-Function' (sfun_MagLevTbx_CurrSens) */

    /* S-Function Block: <S4>/S-Function */
  }
}

/* Model terminate function */
void observing_terminate(void)
{
  /* Terminate for S-Function (sfun_MagLevTbx_MagSens_v43): '<S10>/S-Function' */

  /* S-Function Block: <S10>/S-Function */
  sfun_MagLevTbx_MagSens_WrappedTerminate();

  /* Terminate for S-Function (sfun_MagLevTbx_CurrSens): '<S1>/S-Function' */

  /* S-Function Block: <S1>/S-Function */
  sfun_MagLevTbx_CurrSens_WrappedTerminate();

  /* Terminate for S-Function (sfun_MagLevTbx_CurrSens): '<S2>/S-Function' */

  /* S-Function Block: <S2>/S-Function */
  sfun_MagLevTbx_CurrSens_WrappedTerminate();

  /* Terminate for S-Function (sfun_MagLevTbx_CurrSens): '<S3>/S-Function' */

  /* S-Function Block: <S3>/S-Function */
  sfun_MagLevTbx_CurrSens_WrappedTerminate();

  /* Terminate for S-Function (sfun_MagLevTbx_CurrSens): '<S4>/S-Function' */

  /* S-Function Block: <S4>/S-Function */
  sfun_MagLevTbx_CurrSens_WrappedTerminate();
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
