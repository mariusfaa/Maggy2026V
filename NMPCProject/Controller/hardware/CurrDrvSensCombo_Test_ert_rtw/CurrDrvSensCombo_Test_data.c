/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: CurrDrvSensCombo_Test_data.c
 *
 * Code generated for Simulink model 'CurrDrvSensCombo_Test'.
 *
 * Model version                  : 1.52
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Mon Apr  6 10:38:59 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "CurrDrvSensCombo_Test.h"

/* Block parameters (default storage) */
P_CurrDrvSensCombo_Test_T CurrDrvSensCombo_Test_P = {
  /* Mask Parameter: RepeatingSequenceStair_OutValue
   * Referenced by: '<S9>/Vector'
   */
  { 32.0, 32.0, 32.0, 32.0, 0.0, 0.0, 0.0, 0.0, -32.0, -32.0, -32.0, -32.0, 0.0,
    0.0, 0.0, 0.0 },

  /* Mask Parameter: LimitedCounter_uplimit
   * Referenced by: '<S13>/FixPt Switch'
   */
  15U,

  /* Expression: 0
   * Referenced by: '<Root>/Constant'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Current  Profile  Selector'
   */
  1.0,

  /* Expression: 0
   * Referenced by:
   */
  0.0,

  /* Expression: 32
   * Referenced by: '<Root>/Sine Wave'
   */
  32.0,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave'
   */
  0.0,

  /* Expression: 2*pi*0.25
   * Referenced by: '<Root>/Sine Wave'
   */
  1.5707963267948966,

  /* Expression: 0
   * Referenced by: '<Root>/Sine Wave'
   */
  0.0,

  /* Computed Parameter: SineWave_Hsin
   * Referenced by: '<Root>/Sine Wave'
   */
  0.015707317311820675,

  /* Computed Parameter: SineWave_HCos
   * Referenced by: '<Root>/Sine Wave'
   */
  0.99987663248166059,

  /* Computed Parameter: SineWave_PSin
   * Referenced by: '<Root>/Sine Wave'
   */
  -0.015707317311820675,

  /* Computed Parameter: SineWave_PCos
   * Referenced by: '<Root>/Sine Wave'
   */
  0.99987663248166059,

  /* Computed Parameter: Constant_Value_d
   * Referenced by: '<S13>/Constant'
   */
  0U,

  /* Computed Parameter: Output_InitialCondition
   * Referenced by: '<S11>/Output'
   */
  0U,

  /* Computed Parameter: FixPtConstant_Value
   * Referenced by: '<S12>/FixPt Constant'
   */
  1U
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
