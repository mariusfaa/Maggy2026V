/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: observing_private.h
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

#ifndef observing_private_h_
#define observing_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "observing_types.h"
#include "observing.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetTFinal
#define rtmSetTFinal(rtm, val)         ((rtm)->Timing.tFinal = (val))
#endif

extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void sfun_MagLevTbx_MagSens_WrappedStart(void);
extern void sfun_MagLevTbx_MagSens_WrappedOutput(double *y0);
extern void sfun_MagLevTbx_MagSens_WrappedTerminate(void);
extern void sfun_MagLevTbx_CurrSens_WrappedStart(uint8_T sensorId);
extern void sfun_MagLevTbx_CurrSens_WrappedOutput(uint8_T sensorId, float *y0);
extern void sfun_MagLevTbx_CurrSens_WrappedTerminate(void);
extern int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator);

#endif                                 /* observing_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
