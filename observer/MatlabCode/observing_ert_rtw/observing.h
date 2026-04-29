/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: observing.h
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

#ifndef observing_h_
#define observing_h_
#ifndef observing_COMMON_INCLUDES_
#define observing_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* observing_COMMON_INCLUDES_ */

#include "observing_types.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "MW_target_hardware_resources.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
#define rtmGetRTWExtModeInfo(rtm)      ((rtm)->extModeInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T p[300];
  real_T F[300];
  real_T sigma_pred_data[210];
  real_T A[101];
  real_T b_x[101];
  real_T c_x[101];
  real_T d_x[101];
  real_T P_pred_temp_data[100];
  real_T Ad_data[100];
  real_T P_est_temp_data[100];
  real_T bx[100];
  real_T by[100];
  real_T bz[100];
  real_T bxTemp[100];
  real_T byTemp[100];
  real_T bzTemp[100];
  real_T p_m[100];
  real_T p_c[100];
  real_T p_k[100];
  real_T phi[100];
  real_T rho[100];
  real_T k2[100];
  real_T in4_data[100];
  real_T in1_data[100];
  real_T in2_data[100];
  real_T in3_data[100];
  real_T cols[78];
  real_T b_gamma_data[63];
  real_T Ad_data_c[60];
  real_T In_data[60];
  real_T In_data_b[60];
  real_T in3_data_p[60];
  real_T in2_data_c[60];
  real_T in2_data_f[60];
  real_T in2_data_g[60];
  real_T S_chol[36];
  real_T P_pred[36];
  real_T S_chol_g[36];
  real_T S_chol_m[36];
  real_T in2[36];
  real_T in2_n[36];
  real_T in3[36];
  real_T in3_p[36];
  real_T Cd_data[30];
  real_T Pxy_data[30];
  real_T in3_data_l[30];
  real_T in1_data_j[30];
  real_T L[18];
  real_T B[18];
  real_T in3_d[18];
  real_T in4_data_g[18];
  real_T in2_data_l[18];
  int8_T In_data_d[100];
  boolean_T indices[100];
  int8_T d_data[100];
  real_T x_pred_temp_data[10];
  real_T e_data[10];
  real_T e_data_d[10];
  real_T in3_data_lx[10];
  real_T Pyy[9];
  real_T b_A[9];
  real_T in2_data_o[9];
  real_T in4_data_b[9];
  real_T in1[9];
  real_T x_pred[6];
  real_T dv[6];
  real_T cols_n[6];
  real_T in3_b[6];
  real_T dv1[6];
  real_T dv2[6];
  real_T a[6];
  real_T b_a[6];
  real_T in4[6];
  real_T in3_l[6];
  real_T CastToDouble1[4];             /* '<S5>/Cast To Double1' */
  real_T SFunction[3];                 /* '<S10>/S-Function' */
  real_T meas[3];
  real_T dy_data[3];
  real_T b_z_pred_data[3];
  real_T Y[3];
  real_T dv3[3];
  real_T dv4[3];
  real_T in4_h[3];
  real_T DataTypeConversion;           /* '<S10>/Data Type Conversion' */
  real_T DataTypeConversion1;          /* '<S10>/Data Type Conversion1' */
  real_T DataTypeConversion2;          /* '<S10>/Data Type Conversion2' */
  real_T Gain[3];                      /* '<Root>/Gain' */
  real_T Gain1[3];                     /* '<Root>/Gain1' */
  real_T dt;
  real_T ssq;
  real_T a21;
  real_T d;
  real_T d1;
  real_T g;
  real_T p_b;
  real_T p_d;
  real_T x;
  real_T x_e;
  real_T x_b;
  real_T a_j;
  real_T c;
  real_T b_x_f;
  real_T a0;
  real_T b0;
  real_T s0;
  real_T i1;
  real_T w1;
  real_T a1;
  real_T b1;
  real_T y;
  real_T d2;
  real_T d3;
  real_T d4;
  real_T in2_a;
  real_T d5;
  real_T bxBase;
  real_T byBase;
  real_T bzBase;
  real_T bxTemp_j;
  real_T bzTemp_j;
  real_T b_bxTemp;
  real_T b_byTemp;
  real_T c_o;
  real_T k2_n;
  real_T d_tmp;
  real_T d_tmp_i;
  real_T f_idx_0;
  real_T d_idx_0;
  real_T rho_o;
  real_T c_n;
  real_T k2_m;
  real_T E;
  real_T a_c;
  real_T b_a_m;
  real_T k2_tmp;
  real_T d_tmp_m;
  real_T d_tmp_j;
  real_T d_idx_0_h;
  real_T f_idx_0_c;
  real_T a0_c;
  real_T b0_p;
  real_T s0_p;
  real_T i1_a;
  real_T w1_e;
  real_T a_a;
  real_T b;
  real_T ssq_a;
  int32_T P_pred_temp_size[2];
  int32_T Ad_size[2];
  int32_T sigma_pred_size[2];
  int32_T P_est_temp_size[2];
  int32_T In_size[2];
  int32_T Cd_size[2];
  int32_T Pxy_size[2];
  int32_T b_L_size[2];
  real32_T SFunction_o;                /* '<S1>/S-Function' */
  real32_T SFunction_g;                /* '<S2>/S-Function' */
  real32_T SFunction_b;                /* '<S3>/S-Function' */
  real32_T SFunction_e;                /* '<S4>/S-Function' */
  int32_T jmax;
  int32_T idxAjj;
  int32_T r1;
  int32_T r2;
  int32_T r3;
  int32_T b_r2;
  int32_T b_rtemp;
  int32_T x_pred_temp_size;
} B_observing_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay1_DSTATE[36];        /* '<S5>/Unit Delay1' */
  real_T UnitDelay_DSTATE[6];          /* '<S5>/Unit Delay' */
  struct {
    void *LoggedData;
  } Scope4_PWORK;                      /* '<Root>/Scope4' */

  struct {
    void *LoggedData;
  } Scope5_PWORK;                      /* '<Root>/Scope5' */

  struct {
    void *LoggedData[3];
  } Scope_PWORK;                       /* '<Root>/Scope' */

  boolean_T doneDoubleBufferReInit;    /* '<S5>/MATLAB Function1' */
  boolean_T doneDoubleBufferReInit_c;  /* '<S5>/MATLAB Function' */
} DW_observing_T;

/* Parameters (default storage) */
struct P_observing_T_ {
  struct_dZd4pOWAPoKYGFDw6kFnYF obs;   /* Variable: obs
                                        * Referenced by:
                                        *   '<S5>/MATLAB Function'
                                        *   '<S5>/MATLAB Function1'
                                        */
  real_T UnitDelay1_InitialCondition[36];/* Expression: eye(6,6)
                                          * Referenced by: '<S5>/Unit Delay1'
                                          */
  real_T UnitDelay_InitialCondition[6];/* Expression: zeros(6,1)
                                        * Referenced by: '<S5>/Unit Delay'
                                        */
  real_T Constant_Value;               /* Expression: 1
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T Gain_Gain;                    /* Expression: 1e+3
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T Gain1_Gain;                   /* Expression: 1e+3
                                        * Referenced by: '<Root>/Gain1'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_observing_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T checksums[4];
  } Sizes;

  /*
   * SpecialInfo:
   * The following substructure contains special information
   * related to other components that are dependent on RTW.
   */
  struct {
    const void *mappingInfo;
  } SpecialInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (default storage) */
extern P_observing_T observing_P;

/* Block signals (default storage) */
extern B_observing_T observing_B;

/* Block states (default storage) */
extern DW_observing_T observing_DW;

/* Model entry point functions */
extern void observing_initialize(void);
extern void observing_step(void);
extern void observing_terminate(void);

/* Real-time Model object */
extern RT_MODEL_observing_T *const observing_M;
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S5>/Cast To Double' : Eliminate redundant data type conversion
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
 * '<Root>' : 'observing'
 * '<S1>'   : 'observing/Current Sensor'
 * '<S2>'   : 'observing/Current Sensor1'
 * '<S3>'   : 'observing/Current Sensor2'
 * '<S4>'   : 'observing/Current Sensor3'
 * '<S5>'   : 'observing/Kalman'
 * '<S6>'   : 'observing/Magnetic Sensor'
 * '<S7>'   : 'observing/Kalman/MATLAB Function'
 * '<S8>'   : 'observing/Kalman/MATLAB Function1'
 * '<S9>'   : 'observing/Magnetic Sensor/MagSens'
 * '<S10>'  : 'observing/Magnetic Sensor/MagSens/MagSens PCB 4.3'
 */
#endif                                 /* observing_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
