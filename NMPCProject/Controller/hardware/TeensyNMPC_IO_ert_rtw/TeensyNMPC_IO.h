/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: TeensyNMPC_IO.h
 *
 * Code generated for Simulink model 'TeensyNMPC_IO'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Sun Apr  5 22:10:45 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef TeensyNMPC_IO_h_
#define TeensyNMPC_IO_h_
#ifndef TeensyNMPC_IO_COMMON_INCLUDES_
#define TeensyNMPC_IO_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* TeensyNMPC_IO_COMMON_INCLUDES_ */

#include "TeensyNMPC_IO_types.h"
#include "rt_nonfinite.h"
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real_T SFunction[3];                 /* '<S13>/S-Function' */
  real_T VectorConcatenate[3];         /* '<Root>/Vector Concatenate' */
  real32_T SFunction_f;                /* '<S5>/S-Function' */
  real32_T SFunction_c;                /* '<S6>/S-Function' */
  real32_T SFunction_i;                /* '<S7>/S-Function' */
  real32_T SFunction_a;                /* '<S8>/S-Function' */
  int16_T SFunction_o1[4];             /* '<S10>/S-Function' */
  int16_T Switch1[4];                  /* '<Root>/Switch1' */
  uint8_T SFunction_o2;                /* '<S10>/S-Function' */
} B_TeensyNMPC_IO_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay_DSTATE;             /* '<Root>/Unit Delay' */
  void* SFunction_DWORK1[2];           /* '<S10>/S-Function' */
  void* SFunction_DWORK1_f[2];         /* '<S11>/S-Function' */
} DW_TeensyNMPC_IO_T;

/* Parameters (default storage) */
struct P_TeensyNMPC_IO_T_ {
  real_T Constant2_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant2'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T Constant_Value;               /* Expression: 1
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T UnitDelay_InitialCondition;   /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay'
                                        */
  real_T Switch1_Threshold;            /* Expression: 0
                                        * Referenced by: '<Root>/Switch1'
                                        */
  uint8_T Switch_Threshold;            /* Computed Parameter: Switch_Threshold
                                        * Referenced by: '<Root>/Switch'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_TeensyNMPC_IO_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (default storage) */
extern P_TeensyNMPC_IO_T TeensyNMPC_IO_P;

/* Block signals (default storage) */
extern B_TeensyNMPC_IO_T TeensyNMPC_IO_B;

/* Block states (default storage) */
extern DW_TeensyNMPC_IO_T TeensyNMPC_IO_DW;

/* Model entry point functions */
extern void TeensyNMPC_IO_initialize(void);
extern void TeensyNMPC_IO_step(void);
extern void TeensyNMPC_IO_terminate(void);

/* Real-time Model object */
extern RT_MODEL_TeensyNMPC_IO_T *const TeensyNMPC_IO_M;
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

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
 * '<Root>' : 'TeensyNMPC_IO'
 * '<S1>'   : 'TeensyNMPC_IO/Current Driver'
 * '<S2>'   : 'TeensyNMPC_IO/Current Driver1'
 * '<S3>'   : 'TeensyNMPC_IO/Current Driver2'
 * '<S4>'   : 'TeensyNMPC_IO/Current Driver3'
 * '<S5>'   : 'TeensyNMPC_IO/Current Sensor'
 * '<S6>'   : 'TeensyNMPC_IO/Current Sensor1'
 * '<S7>'   : 'TeensyNMPC_IO/Current Sensor2'
 * '<S8>'   : 'TeensyNMPC_IO/Current Sensor3'
 * '<S9>'   : 'TeensyNMPC_IO/Magnetic Sensor'
 * '<S10>'  : 'TeensyNMPC_IO/USB Serial Packet Receive'
 * '<S11>'  : 'TeensyNMPC_IO/USB Serial Packet Send'
 * '<S12>'  : 'TeensyNMPC_IO/Magnetic Sensor/MagSens'
 * '<S13>'  : 'TeensyNMPC_IO/Magnetic Sensor/MagSens/MagSens PCB 4.3'
 */
#endif                                 /* TeensyNMPC_IO_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
