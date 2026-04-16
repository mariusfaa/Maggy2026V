/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: CurrDrvSensCombo_Test.h
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

#ifndef CurrDrvSensCombo_Test_h_
#define CurrDrvSensCombo_Test_h_
#ifndef CurrDrvSensCombo_Test_COMMON_INCLUDES_
#define CurrDrvSensCombo_Test_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                              /* CurrDrvSensCombo_Test_COMMON_INCLUDES_ */

#include "CurrDrvSensCombo_Test_types.h"
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmStepTask
#define rtmStepTask(rtm, idx)          ((rtm)->Timing.TaskCounters.TID[(idx)] == 0)
#endif

#ifndef rtmTaskCounter
#define rtmTaskCounter(rtm, idx)       ((rtm)->Timing.TaskCounters.TID[(idx)])
#endif

/* Block signals (default storage) */
typedef struct {
  real_T TmpRTBAtMultiportSwitchInport2;/* '<Root>/Repeating Sequence Stair' */
  real32_T SFunction;                  /* '<S5>/S-Function' */
  real32_T SFunction_b;                /* '<S7>/S-Function' */
  real32_T SFunction_e;                /* '<S6>/S-Function' */
  real32_T SFunction_l;                /* '<S8>/S-Function' */
  int16_T DataTypeConversion;          /* '<Root>/Data Type Conversion' */
} B_CurrDrvSensCombo_Test_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T TmpRTBAtMultiportSwitchInport2_;/* synthesized block */
  real_T lastSin;                      /* '<Root>/Sine Wave' */
  real_T lastCos;                      /* '<Root>/Sine Wave' */
  void* SFunction_DWORK1[2];           /* '<S10>/S-Function' */
  int32_T systemEnable;                /* '<Root>/Sine Wave' */
  uint8_T Output_DSTATE;               /* '<S11>/Output' */
} DW_CurrDrvSensCombo_Test_T;

/* Parameters (default storage) */
struct P_CurrDrvSensCombo_Test_T_ {
  real_T RepeatingSequenceStair_OutValue[16];
                              /* Mask Parameter: RepeatingSequenceStair_OutValue
                               * Referenced by: '<S9>/Vector'
                               */
  uint8_T LimitedCounter_uplimit;      /* Mask Parameter: LimitedCounter_uplimit
                                        * Referenced by: '<S13>/FixPt Switch'
                                        */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T CurrentProfileSelector_Value; /* Expression: 1
                                        * Referenced by: '<Root>/Current  Profile  Selector'
                                        */
  real_T TmpRTBAtMultiportSwitchInport2_;/* Expression: 0
                                          * Referenced by:
                                          */
  real_T SineWave_Amp;                 /* Expression: 32
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_Bias;                /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_Freq;                /* Expression: 2*pi*0.25
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_Phase;               /* Expression: 0
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_Hsin;                /* Computed Parameter: SineWave_Hsin
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_HCos;                /* Computed Parameter: SineWave_HCos
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_PSin;                /* Computed Parameter: SineWave_PSin
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  real_T SineWave_PCos;                /* Computed Parameter: SineWave_PCos
                                        * Referenced by: '<Root>/Sine Wave'
                                        */
  uint8_T Constant_Value_d;            /* Computed Parameter: Constant_Value_d
                                        * Referenced by: '<S13>/Constant'
                                        */
  uint8_T Output_InitialCondition;/* Computed Parameter: Output_InitialCondition
                                   * Referenced by: '<S11>/Output'
                                   */
  uint8_T FixPtConstant_Value;        /* Computed Parameter: FixPtConstant_Value
                                       * Referenced by: '<S12>/FixPt Constant'
                                       */
};

/* Real-time Model Data Structure */
struct tag_RTM_CurrDrvSensCombo_Test_T {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    struct {
      uint8_T TID[2];
    } TaskCounters;

    struct {
      boolean_T TID0_1;
    } RateInteraction;
  } Timing;
};

/* Block parameters (default storage) */
extern P_CurrDrvSensCombo_Test_T CurrDrvSensCombo_Test_P;

/* Block signals (default storage) */
extern B_CurrDrvSensCombo_Test_T CurrDrvSensCombo_Test_B;

/* Block states (default storage) */
extern DW_CurrDrvSensCombo_Test_T CurrDrvSensCombo_Test_DW;

/* External data declarations for dependent source files */
extern const int16_T CurrDrvSensCombo_Test_I16GND;/* int16_T ground */

/* External function called from main */
extern void CurrDrvSensCombo_Test_SetEventsForThisBaseStep(boolean_T *eventFlags);

/* Model entry point functions */
extern void CurrDrvSensCombo_Test_initialize(void);
extern void CurrDrvSensCombo_Test_step0(void);/* Sample time: [0.01s, 0.0s] */
extern void CurrDrvSensCombo_Test_step1(void);/* Sample time: [0.1s, 0.0s] */
extern void CurrDrvSensCombo_Test_terminate(void);

/* Real-time Model object */
extern RT_MODEL_CurrDrvSensCombo_Tes_T *const CurrDrvSensCombo_Test_M;
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S11>/Data Type Propagation' : Unused code path elimination
 * Block '<S12>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S13>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S9>/Out' : Eliminate redundant signal conversion block
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'CurrDrvSensCombo_Test'
 * '<S1>'   : 'CurrDrvSensCombo_Test/Current Driver (+X)'
 * '<S2>'   : 'CurrDrvSensCombo_Test/Current Driver (+Y)'
 * '<S3>'   : 'CurrDrvSensCombo_Test/Current Driver (-X)'
 * '<S4>'   : 'CurrDrvSensCombo_Test/Current Driver (-Y)'
 * '<S5>'   : 'CurrDrvSensCombo_Test/Current Sensor (+X)'
 * '<S6>'   : 'CurrDrvSensCombo_Test/Current Sensor (+Y)'
 * '<S7>'   : 'CurrDrvSensCombo_Test/Current Sensor (-X)'
 * '<S8>'   : 'CurrDrvSensCombo_Test/Current Sensor (-Y)'
 * '<S9>'   : 'CurrDrvSensCombo_Test/Repeating Sequence Stair'
 * '<S10>'  : 'CurrDrvSensCombo_Test/USB Serial Packet Send'
 * '<S11>'  : 'CurrDrvSensCombo_Test/Repeating Sequence Stair/LimitedCounter'
 * '<S12>'  : 'CurrDrvSensCombo_Test/Repeating Sequence Stair/LimitedCounter/Increment Real World'
 * '<S13>'  : 'CurrDrvSensCombo_Test/Repeating Sequence Stair/LimitedCounter/Wrap To Zero'
 */
#endif                                 /* CurrDrvSensCombo_Test_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
