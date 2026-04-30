/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: CurrDrvSensCombo_Test.c
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
#include <math.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "CurrDrvSensCombo_Test_private.h"

const int16_T CurrDrvSensCombo_Test_I16GND = 0;/* int16_T ground */

/* Block signals (default storage) */
B_CurrDrvSensCombo_Test_T CurrDrvSensCombo_Test_B;

/* Block states (default storage) */
DW_CurrDrvSensCombo_Test_T CurrDrvSensCombo_Test_DW;

/* Real-time model */
static RT_MODEL_CurrDrvSensCombo_Tes_T CurrDrvSensCombo_Test_M_;
RT_MODEL_CurrDrvSensCombo_Tes_T *const CurrDrvSensCombo_Test_M =
  &CurrDrvSensCombo_Test_M_;
static void rate_monotonic_scheduler(void);

/*
 * Set which subrates need to run this base step (base rate always runs).
 * This function must be called prior to calling the model step function
 * in order to remember which rates need to run this base step.  The
 * buffering of events allows for overlapping preemption.
 */
void CurrDrvSensCombo_Test_SetEventsForThisBaseStep(boolean_T *eventFlags)
{
  /* Task runs when its counter is zero, computed via rtmStepTask macro */
  eventFlags[1] = ((boolean_T)rtmStepTask(CurrDrvSensCombo_Test_M, 1));
}

/*
 *         This function updates active task flag for each subrate
 *         and rate transition flags for tasks that exchange data.
 *         The function assumes rate-monotonic multitasking scheduler.
 *         The function must be called at model base rate so that
 *         the generated code self-manages all its subrates and rate
 *         transition flags.
 */
static void rate_monotonic_scheduler(void)
{
  /* To ensure a deterministic data transfer between two rates,
   * data is transferred at the priority of a fast task and the frequency
   * of the slow task.  The following flags indicate when the data transfer
   * happens.  That is, a rate interaction flag is set true when both rates
   * will run, and false otherwise.
   */

  /* tid 0 shares data with slower tid rate: 1 */
  CurrDrvSensCombo_Test_M->Timing.RateInteraction.TID0_1 =
    (CurrDrvSensCombo_Test_M->Timing.TaskCounters.TID[1] == 0);

  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (CurrDrvSensCombo_Test_M->Timing.TaskCounters.TID[1])++;
  if ((CurrDrvSensCombo_Test_M->Timing.TaskCounters.TID[1]) > 9) {/* Sample time: [0.1s, 0.0s] */
    CurrDrvSensCombo_Test_M->Timing.TaskCounters.TID[1] = 0;
  }
}

/* Model step function for TID0 */
void CurrDrvSensCombo_Test_step0(void) /* Sample time: [0.01s, 0.0s] */
{
  real_T rtb_MultiportSwitch;

  {                                    /* Sample time: [0.01s, 0.0s] */
    rate_monotonic_scheduler();
  }

  /* RateTransition generated from: '<Root>/Multiport Switch' */
  if (CurrDrvSensCombo_Test_M->Timing.RateInteraction.TID0_1) {
    /* RateTransition generated from: '<Root>/Multiport Switch' */
    CurrDrvSensCombo_Test_B.TmpRTBAtMultiportSwitchInport2 =
      CurrDrvSensCombo_Test_DW.TmpRTBAtMultiportSwitchInport2_;
  }

  /* End of RateTransition generated from: '<Root>/Multiport Switch' */

  /* Sin: '<Root>/Sine Wave' */
  if (CurrDrvSensCombo_Test_DW.systemEnable != 0) {
    rtb_MultiportSwitch = CurrDrvSensCombo_Test_P.SineWave_Freq *
      ((CurrDrvSensCombo_Test_M->Timing.clockTick0) * 0.01);
    CurrDrvSensCombo_Test_DW.lastSin = sin(rtb_MultiportSwitch);
    CurrDrvSensCombo_Test_DW.lastCos = cos(rtb_MultiportSwitch);
    CurrDrvSensCombo_Test_DW.systemEnable = 0;
  }

  /* MultiPortSwitch: '<Root>/Multiport Switch' incorporates:
   *  Constant: '<Root>/Constant'
   *  Constant: '<Root>/Current  Profile  Selector'
   *  Sin: '<Root>/Sine Wave'
   */
  switch ((int32_T)CurrDrvSensCombo_Test_P.CurrentProfileSelector_Value) {
   case 1:
    rtb_MultiportSwitch = CurrDrvSensCombo_Test_B.TmpRTBAtMultiportSwitchInport2;
    break;

   case 2:
    rtb_MultiportSwitch = ((CurrDrvSensCombo_Test_DW.lastSin *
      CurrDrvSensCombo_Test_P.SineWave_PCos + CurrDrvSensCombo_Test_DW.lastCos *
      CurrDrvSensCombo_Test_P.SineWave_PSin) *
      CurrDrvSensCombo_Test_P.SineWave_HCos + (CurrDrvSensCombo_Test_DW.lastCos *
      CurrDrvSensCombo_Test_P.SineWave_PCos - CurrDrvSensCombo_Test_DW.lastSin *
      CurrDrvSensCombo_Test_P.SineWave_PSin) *
      CurrDrvSensCombo_Test_P.SineWave_Hsin) *
      CurrDrvSensCombo_Test_P.SineWave_Amp +
      CurrDrvSensCombo_Test_P.SineWave_Bias;
    break;

   default:
    rtb_MultiportSwitch = CurrDrvSensCombo_Test_P.Constant_Value;
    break;
  }

  /* End of MultiPortSwitch: '<Root>/Multiport Switch' */

  /* DataTypeConversion: '<Root>/Data Type Conversion' */
  rtb_MultiportSwitch = floor(rtb_MultiportSwitch);
  if (rtIsNaN(rtb_MultiportSwitch) || rtIsInf(rtb_MultiportSwitch)) {
    rtb_MultiportSwitch = 0.0;
  } else {
    rtb_MultiportSwitch = fmod(rtb_MultiportSwitch, 65536.0);
  }

  /* DataTypeConversion: '<Root>/Data Type Conversion' */
  CurrDrvSensCombo_Test_B.DataTypeConversion = (int16_T)(rtb_MultiportSwitch <
    0.0 ? (int32_T)(int16_T)-(int16_T)(uint16_T)-rtb_MultiportSwitch : (int32_T)
    (int16_T)(uint16_T)rtb_MultiportSwitch);

  /* S-Function (sfun_MagLevTbx_CurrDrv): '<S3>/S-Function' */

  /* S-Function Block: <S3>/S-Function */

  /*  Set PWM of current driver 1U  */
  sfun_MagLevTbx_CurrDrv_WrappedOutput(1U,
    &CurrDrvSensCombo_Test_B.DataTypeConversion);

  /* S-Function (sfun_MagLevTbx_CurrDrv): '<S1>/S-Function' */

  /* S-Function Block: <S1>/S-Function */

  /*  Set PWM of current driver 0U  */
  sfun_MagLevTbx_CurrDrv_WrappedOutput(0U, ((const int16_T*)
    &CurrDrvSensCombo_Test_I16GND));

  /* S-Function (sfun_MagLevTbx_CurrDrv): '<S2>/S-Function' */

  /* S-Function Block: <S2>/S-Function */

  /*  Set PWM of current driver 2U  */
  sfun_MagLevTbx_CurrDrv_WrappedOutput(2U, ((const int16_T*)
    &CurrDrvSensCombo_Test_I16GND));

  /* S-Function (sfun_MagLevTbx_CurrDrv): '<S4>/S-Function' */

  /* S-Function Block: <S4>/S-Function */

  /*  Set PWM of current driver 3U  */
  sfun_MagLevTbx_CurrDrv_WrappedOutput(3U, ((const int16_T*)
    &CurrDrvSensCombo_Test_I16GND));

  /* S-Function (sfun_MagLevTbx_CurrSens): '<S5>/S-Function' */

  /* S-Function Block: <S5>/S-Function */

  /*  Get measurement of current sensor 0U  */
  sfun_MagLevTbx_CurrSens_WrappedOutput(0U, &CurrDrvSensCombo_Test_B.SFunction);

  /* S-Function (sfun_MagLevTbx_CurrSens): '<S7>/S-Function' */

  /* S-Function Block: <S7>/S-Function */

  /*  Get measurement of current sensor 1U  */
  sfun_MagLevTbx_CurrSens_WrappedOutput(1U, &CurrDrvSensCombo_Test_B.SFunction_b);

  /* S-Function (sfun_MagLevTbx_CurrSens): '<S6>/S-Function' */

  /* S-Function Block: <S6>/S-Function */

  /*  Get measurement of current sensor 2U  */
  sfun_MagLevTbx_CurrSens_WrappedOutput(2U, &CurrDrvSensCombo_Test_B.SFunction_e);

  /* S-Function (sfun_MagLevTbx_CurrSens): '<S8>/S-Function' */

  /* S-Function Block: <S8>/S-Function */

  /*  Get measurement of current sensor 3U  */
  sfun_MagLevTbx_CurrSens_WrappedOutput(3U, &CurrDrvSensCombo_Test_B.SFunction_l);

  /* S-Function (sfun_MagLevTbx_UsbSerialPacketSend): '<S10>/S-Function' */
  /* S-Function Block: <S10>/S-Function */

  /*  Pack input signals to payload buffer  */
  {
    uint8_T *pAux = (uint8_T *)CurrDrvSensCombo_Test_DW.SFunction_DWORK1[0];

    /*  copy input signals to payload buffer  */
    memcpy(pAux, &CurrDrvSensCombo_Test_B.SFunction, 4);
    pAux += 4;

    /*  copy input signals to payload buffer  */
    memcpy(pAux, &CurrDrvSensCombo_Test_B.SFunction_b, 4);
    pAux += 4;

    /*  copy input signals to payload buffer  */
    memcpy(pAux, &CurrDrvSensCombo_Test_B.SFunction_e, 4);
    pAux += 4;

    /*  copy input signals to payload buffer  */
    memcpy(pAux, &CurrDrvSensCombo_Test_B.SFunction_l, 4);
  }

  /*  Perform COBS encoding  */
  cobsEncode(CurrDrvSensCombo_Test_DW.SFunction_DWORK1[0], 16U,
             CurrDrvSensCombo_Test_DW.SFunction_DWORK1[1]);

  /*  Add null terminator  */
  ((uint8_T *)CurrDrvSensCombo_Test_DW.SFunction_DWORK1[1])[17U] = 0x00;

  /*  Send packet  */
  sfun_MagLevTbx_UsbSerialPacketSend_WrappedOutput( (uint8_T*)
    CurrDrvSensCombo_Test_DW.SFunction_DWORK1[1], 18U, 1);

  /* Update for Sin: '<Root>/Sine Wave' */
  rtb_MultiportSwitch = CurrDrvSensCombo_Test_DW.lastSin;
  CurrDrvSensCombo_Test_DW.lastSin = CurrDrvSensCombo_Test_DW.lastSin *
    CurrDrvSensCombo_Test_P.SineWave_HCos + CurrDrvSensCombo_Test_DW.lastCos *
    CurrDrvSensCombo_Test_P.SineWave_Hsin;
  CurrDrvSensCombo_Test_DW.lastCos = CurrDrvSensCombo_Test_DW.lastCos *
    CurrDrvSensCombo_Test_P.SineWave_HCos - rtb_MultiportSwitch *
    CurrDrvSensCombo_Test_P.SineWave_Hsin;

  /* Update absolute time */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  CurrDrvSensCombo_Test_M->Timing.clockTick0++;
}

/* Model step function for TID1 */
void CurrDrvSensCombo_Test_step1(void) /* Sample time: [0.1s, 0.0s] */
{
  real_T rtb_Output;

  /* MultiPortSwitch: '<S9>/Output' incorporates:
   *  Constant: '<S9>/Vector'
   *  UnitDelay: '<S11>/Output'
   */
  rtb_Output =
    CurrDrvSensCombo_Test_P.RepeatingSequenceStair_OutValue[CurrDrvSensCombo_Test_DW.Output_DSTATE];

  /* RateTransition generated from: '<Root>/Multiport Switch' */
  CurrDrvSensCombo_Test_DW.TmpRTBAtMultiportSwitchInport2_ = rtb_Output;

  /* Sum: '<S12>/FixPt Sum1' incorporates:
   *  Constant: '<S12>/FixPt Constant'
   *  UnitDelay: '<S11>/Output'
   */
  CurrDrvSensCombo_Test_DW.Output_DSTATE +=
    CurrDrvSensCombo_Test_P.FixPtConstant_Value;

  /* Switch: '<S13>/FixPt Switch' */
  if (CurrDrvSensCombo_Test_DW.Output_DSTATE >
      CurrDrvSensCombo_Test_P.LimitedCounter_uplimit) {
    /* Sum: '<S12>/FixPt Sum1' incorporates:
     *  Constant: '<S13>/Constant'
     */
    CurrDrvSensCombo_Test_DW.Output_DSTATE =
      CurrDrvSensCombo_Test_P.Constant_Value_d;
  }

  /* End of Switch: '<S13>/FixPt Switch' */
}

/* Model initialize function */
void CurrDrvSensCombo_Test_initialize(void)
{
  /* Start for RateTransition generated from: '<Root>/Multiport Switch' */
  CurrDrvSensCombo_Test_B.TmpRTBAtMultiportSwitchInport2 =
    CurrDrvSensCombo_Test_P.TmpRTBAtMultiportSwitchInport2_;

  /* Start for S-Function (sfun_MagLevTbx_CurrDrv): '<S3>/S-Function' */

  /* S-Function Block: <S3>/S-Function */

  /* Init current driver 1U (first user) */
  sfun_MagLevTbx_CurrDrv_WrappedStart(1U);

  /* Start for S-Function (sfun_MagLevTbx_CurrDrv): '<S1>/S-Function' */

  /* S-Function Block: <S1>/S-Function */

  /* Init current driver 0U (first user) */
  sfun_MagLevTbx_CurrDrv_WrappedStart(0U);

  /* Start for S-Function (sfun_MagLevTbx_CurrDrv): '<S2>/S-Function' */

  /* S-Function Block: <S2>/S-Function */

  /* Init current driver 2U (first user) */
  sfun_MagLevTbx_CurrDrv_WrappedStart(2U);

  /* Start for S-Function (sfun_MagLevTbx_CurrDrv): '<S4>/S-Function' */

  /* S-Function Block: <S4>/S-Function */

  /* Init current driver 3U (first user) */
  sfun_MagLevTbx_CurrDrv_WrappedStart(3U);

  /* Start for S-Function (sfun_MagLevTbx_CurrSens): '<S5>/S-Function' */

  /* S-Function Block: <S5>/S-Function */

  /*  Initialize current sensors 0U  */
  sfun_MagLevTbx_CurrSens_WrappedStart(0U);

  /* Start for S-Function (sfun_MagLevTbx_CurrSens): '<S7>/S-Function' */

  /* S-Function Block: <S7>/S-Function */

  /*  Initialize current sensors 1U  */
  sfun_MagLevTbx_CurrSens_WrappedStart(1U);

  /* Start for S-Function (sfun_MagLevTbx_CurrSens): '<S6>/S-Function' */

  /* S-Function Block: <S6>/S-Function */

  /*  Initialize current sensors 2U  */
  sfun_MagLevTbx_CurrSens_WrappedStart(2U);

  /* Start for S-Function (sfun_MagLevTbx_CurrSens): '<S8>/S-Function' */

  /* S-Function Block: <S8>/S-Function */

  /*  Initialize current sensors 3U  */
  sfun_MagLevTbx_CurrSens_WrappedStart(3U);

  /* Start for S-Function (sfun_MagLevTbx_UsbSerialPacketSend): '<S10>/S-Function' */

  /* S-Function Block: <S10>/S-Function */

  /*  Allocate payload buffer  */
  CurrDrvSensCombo_Test_DW.SFunction_DWORK1[0] = calloc(16U, sizeof(uint8_T));

  /*  Allocate packet buffer  */
  CurrDrvSensCombo_Test_DW.SFunction_DWORK1[1] = calloc(18U, sizeof(uint8_T));

  /*  Initialize USB Serial  */
  sfun_MagLevTbx_UsbSerialPacketSend_WrappedStart();

  /* InitializeConditions for RateTransition generated from: '<Root>/Multiport Switch' */
  CurrDrvSensCombo_Test_DW.TmpRTBAtMultiportSwitchInport2_ =
    CurrDrvSensCombo_Test_P.TmpRTBAtMultiportSwitchInport2_;

  /* InitializeConditions for Sum: '<S12>/FixPt Sum1' incorporates:
   *  UnitDelay: '<S11>/Output'
   */
  CurrDrvSensCombo_Test_DW.Output_DSTATE =
    CurrDrvSensCombo_Test_P.Output_InitialCondition;

  /* Enable for Sin: '<Root>/Sine Wave' */
  CurrDrvSensCombo_Test_DW.systemEnable = 1;

  /* Enable for S-Function (sfun_MagLevTbx_CurrDrv): '<S3>/S-Function' */
  /* Level2 S-Function Block: '<S3>/S-Function' (sfun_MagLevTbx_CurrDrv) */

  /* S-Function Block: <S3>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_CurrDrv): '<S1>/S-Function' */
  /* Level2 S-Function Block: '<S1>/S-Function' (sfun_MagLevTbx_CurrDrv) */

  /* S-Function Block: <S1>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_CurrDrv): '<S2>/S-Function' */
  /* Level2 S-Function Block: '<S2>/S-Function' (sfun_MagLevTbx_CurrDrv) */

  /* S-Function Block: <S2>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_CurrDrv): '<S4>/S-Function' */
  /* Level2 S-Function Block: '<S4>/S-Function' (sfun_MagLevTbx_CurrDrv) */

  /* S-Function Block: <S4>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_CurrSens): '<S5>/S-Function' */
  /* Level2 S-Function Block: '<S5>/S-Function' (sfun_MagLevTbx_CurrSens) */

  /* S-Function Block: <S5>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_CurrSens): '<S7>/S-Function' */
  /* Level2 S-Function Block: '<S7>/S-Function' (sfun_MagLevTbx_CurrSens) */

  /* S-Function Block: <S7>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_CurrSens): '<S6>/S-Function' */
  /* Level2 S-Function Block: '<S6>/S-Function' (sfun_MagLevTbx_CurrSens) */

  /* S-Function Block: <S6>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_CurrSens): '<S8>/S-Function' */
  /* Level2 S-Function Block: '<S8>/S-Function' (sfun_MagLevTbx_CurrSens) */

  /* S-Function Block: <S8>/S-Function */
}

/* Model terminate function */
void CurrDrvSensCombo_Test_terminate(void)
{
  /* Terminate for S-Function (sfun_MagLevTbx_CurrDrv): '<S3>/S-Function' */

  /* S-Function Block: <S3>/S-Function */

  /* De-init current driver 1U (last user) */
  sfun_MagLevTbx_CurrDrv_WrappedTerminate(1U);

  /* Terminate for S-Function (sfun_MagLevTbx_CurrDrv): '<S1>/S-Function' */

  /* S-Function Block: <S1>/S-Function */

  /* De-init current driver 0U (last user) */
  sfun_MagLevTbx_CurrDrv_WrappedTerminate(0U);

  /* Terminate for S-Function (sfun_MagLevTbx_CurrDrv): '<S2>/S-Function' */

  /* S-Function Block: <S2>/S-Function */

  /* De-init current driver 2U (last user) */
  sfun_MagLevTbx_CurrDrv_WrappedTerminate(2U);

  /* Terminate for S-Function (sfun_MagLevTbx_CurrDrv): '<S4>/S-Function' */

  /* S-Function Block: <S4>/S-Function */

  /* De-init current driver 3U (last user) */
  sfun_MagLevTbx_CurrDrv_WrappedTerminate(3U);

  /* Terminate for S-Function (sfun_MagLevTbx_CurrSens): '<S5>/S-Function' */

  /* S-Function Block: <S5>/S-Function */
  sfun_MagLevTbx_CurrSens_WrappedTerminate();

  /* Terminate for S-Function (sfun_MagLevTbx_CurrSens): '<S7>/S-Function' */

  /* S-Function Block: <S7>/S-Function */
  sfun_MagLevTbx_CurrSens_WrappedTerminate();

  /* Terminate for S-Function (sfun_MagLevTbx_CurrSens): '<S6>/S-Function' */

  /* S-Function Block: <S6>/S-Function */
  sfun_MagLevTbx_CurrSens_WrappedTerminate();

  /* Terminate for S-Function (sfun_MagLevTbx_CurrSens): '<S8>/S-Function' */

  /* S-Function Block: <S8>/S-Function */
  sfun_MagLevTbx_CurrSens_WrappedTerminate();

  /* Terminate for S-Function (sfun_MagLevTbx_UsbSerialPacketSend): '<S10>/S-Function' */

  /* S-Function Block: <S10>/S-Function */

  /*  Release heap space  */
  free(CurrDrvSensCombo_Test_DW.SFunction_DWORK1[0]);
  free(CurrDrvSensCombo_Test_DW.SFunction_DWORK1[1]);

  /* De-init USB Serial (last user) */
  sfun_MagLevTbx_UsbSerialPacketSend_WrappedTerminate();
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
