/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: TeensyNMPC_IO.c
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

#include "TeensyNMPC_IO.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "TeensyNMPC_IO_private.h"

/* Block signals (default storage) */
B_TeensyNMPC_IO_T TeensyNMPC_IO_B;

/* Block states (default storage) */
DW_TeensyNMPC_IO_T TeensyNMPC_IO_DW;

/* Real-time model */
static RT_MODEL_TeensyNMPC_IO_T TeensyNMPC_IO_M_;
RT_MODEL_TeensyNMPC_IO_T *const TeensyNMPC_IO_M = &TeensyNMPC_IO_M_;

/* Model step function */
void TeensyNMPC_IO_step(void)
{
  real_T tmp;
  real_T tmp_0;
  int16_T Switch1_tmp;
  boolean_T tmp_1;

  {
    /* user code (Output function Header) */

    /*  number of bytes placed in receiving buffer  */
    size_t receivedBytesNum;

    /* S-Function (sfun_MagLevTbx_UsbSerialPacketReceive): '<S10>/S-Function' */
    /* S-Function Block: <S10>/S-Function */

    /*  Receive packet  */
    receivedBytesNum = sfun_MagLevTbx_UsbSerialPacketReceive_WrappedOutput
      ( (uint8_T *)TeensyNMPC_IO_DW.SFunction_DWORK1[1], 10U, (boolean_T *)1);
    if (receivedBytesNum == 10U) {
      /*  Valid packet: proceed with decoding and unpacking  */

      /*  Perform COBS decoding  */
      cobsDecode((uint8_T *) TeensyNMPC_IO_DW.SFunction_DWORK1[1], 9U, (uint8_T *)
                 TeensyNMPC_IO_DW.SFunction_DWORK1[0]);

      /*  Unpack payload buffer to output data ports  */
      {
        uint8_T *pAux = (uint8_T *)TeensyNMPC_IO_DW.SFunction_DWORK1[0];
        memcpy(&TeensyNMPC_IO_B.SFunction_o1[0], pAux, 8);
      }

      /*  Set Status port to 1 (packet received)  */
      TeensyNMPC_IO_B.SFunction_o2 = 1;
    } else {
      /*  Invalid packet: do nothing (hold previous output values)  */

      /*  Set Status port to 0 (packet not received)  */
      TeensyNMPC_IO_B.SFunction_o2 = 0;
    }

    /* Switch: '<Root>/Switch' incorporates:
     *  Constant: '<Root>/Constant'
     *  Constant: '<Root>/Constant1'
     *  Sum: '<Root>/Add'
     *  UnitDelay: '<Root>/Unit Delay'
     */
    if (TeensyNMPC_IO_B.SFunction_o2 > TeensyNMPC_IO_P.Switch_Threshold) {
      TeensyNMPC_IO_DW.UnitDelay_DSTATE = TeensyNMPC_IO_P.Constant1_Value;
    } else {
      TeensyNMPC_IO_DW.UnitDelay_DSTATE += TeensyNMPC_IO_P.Constant_Value;
    }

    /* End of Switch: '<Root>/Switch' */

    /* Switch: '<Root>/Switch1' incorporates:
     *  Constant: '<Root>/Constant2'
     *  UnitDelay: '<Root>/Unit Delay'
     */
    if (TeensyNMPC_IO_DW.UnitDelay_DSTATE > TeensyNMPC_IO_P.Switch1_Threshold) {
      /* Switch: '<Root>/Switch1' */
      TeensyNMPC_IO_B.Switch1[0] = TeensyNMPC_IO_B.SFunction_o1[0];
      TeensyNMPC_IO_B.Switch1[1] = TeensyNMPC_IO_B.SFunction_o1[1];
      TeensyNMPC_IO_B.Switch1[2] = TeensyNMPC_IO_B.SFunction_o1[2];
      TeensyNMPC_IO_B.Switch1[3] = TeensyNMPC_IO_B.SFunction_o1[3];
    } else {
      tmp_0 = floor(TeensyNMPC_IO_P.Constant2_Value);
      tmp_1 = (rtIsNaN(tmp_0) || rtIsInf(tmp_0));
      if (tmp_1) {
        tmp = 0.0;
      } else {
        tmp = fmod(tmp_0, 65536.0);
      }

      /* Switch: '<Root>/Switch1' incorporates:
       *  Constant: '<Root>/Constant2'
       */
      TeensyNMPC_IO_B.Switch1[0] = (int16_T)(tmp < 0.0 ? (int32_T)(int16_T)
        -(int16_T)(uint16_T)-tmp : (int32_T)(int16_T)(uint16_T)tmp);
      if (tmp_1) {
        tmp = 0.0;
      } else {
        tmp = fmod(tmp_0, 65536.0);
      }

      /* Switch: '<Root>/Switch1' */
      TeensyNMPC_IO_B.Switch1[1] = (int16_T)(tmp < 0.0 ? (int32_T)(int16_T)
        -(int16_T)(uint16_T)-tmp : (int32_T)(int16_T)(uint16_T)tmp);
      if (tmp_1) {
        tmp = 0.0;
      } else {
        tmp = fmod(tmp_0, 65536.0);
      }

      /* Switch: '<Root>/Switch1' */
      Switch1_tmp = (int16_T)(tmp < 0.0 ? (int32_T)(int16_T)-(int16_T)(uint16_T)
        -tmp : (int32_T)(int16_T)(uint16_T)tmp);
      TeensyNMPC_IO_B.Switch1[2] = Switch1_tmp;
      TeensyNMPC_IO_B.Switch1[3] = Switch1_tmp;
    }

    /* End of Switch: '<Root>/Switch1' */

    /* S-Function (sfun_MagLevTbx_CurrDrv): '<S1>/S-Function' */

    /* S-Function Block: <S1>/S-Function */

    /*  Set PWM of current driver 0U  */
    sfun_MagLevTbx_CurrDrv_WrappedOutput(0U, &TeensyNMPC_IO_B.Switch1[0]);

    /* S-Function (sfun_MagLevTbx_CurrDrv): '<S2>/S-Function' */

    /* S-Function Block: <S2>/S-Function */

    /*  Set PWM of current driver 1U  */
    sfun_MagLevTbx_CurrDrv_WrappedOutput(1U, &TeensyNMPC_IO_B.Switch1[1]);

    /* S-Function (sfun_MagLevTbx_CurrDrv): '<S3>/S-Function' */

    /* S-Function Block: <S3>/S-Function */

    /*  Set PWM of current driver 2U  */
    sfun_MagLevTbx_CurrDrv_WrappedOutput(2U, &TeensyNMPC_IO_B.Switch1[2]);

    /* S-Function (sfun_MagLevTbx_CurrDrv): '<S4>/S-Function' */

    /* S-Function Block: <S4>/S-Function */

    /*  Set PWM of current driver 3U  */
    sfun_MagLevTbx_CurrDrv_WrappedOutput(3U, &TeensyNMPC_IO_B.Switch1[3]);

    /* S-Function (sfun_MagLevTbx_MagSens_v43): '<S13>/S-Function' */
    /* S-Function Block: <S13>/S-Function */

    /*  Get measurement of mag sensor  */
    sfun_MagLevTbx_MagSens_WrappedOutput(&TeensyNMPC_IO_B.SFunction[0]);

    /* DataTypeConversion: '<S13>/Data Type Conversion' incorporates:
     *  Concatenate: '<Root>/Vector Concatenate'
     */
    TeensyNMPC_IO_B.VectorConcatenate[0] = TeensyNMPC_IO_B.SFunction[0];

    /* DataTypeConversion: '<S13>/Data Type Conversion1' incorporates:
     *  Concatenate: '<Root>/Vector Concatenate'
     */
    TeensyNMPC_IO_B.VectorConcatenate[1] = TeensyNMPC_IO_B.SFunction[1];

    /* DataTypeConversion: '<S13>/Data Type Conversion2' incorporates:
     *  Concatenate: '<Root>/Vector Concatenate'
     */
    TeensyNMPC_IO_B.VectorConcatenate[2] = TeensyNMPC_IO_B.SFunction[2];

    /* S-Function (sfun_MagLevTbx_CurrSens): '<S5>/S-Function' */

    /* S-Function Block: <S5>/S-Function */

    /*  Get measurement of current sensor 0U  */
    sfun_MagLevTbx_CurrSens_WrappedOutput(0U, &TeensyNMPC_IO_B.SFunction_f);

    /* S-Function (sfun_MagLevTbx_CurrSens): '<S6>/S-Function' */

    /* S-Function Block: <S6>/S-Function */

    /*  Get measurement of current sensor 1U  */
    sfun_MagLevTbx_CurrSens_WrappedOutput(1U, &TeensyNMPC_IO_B.SFunction_c);

    /* S-Function (sfun_MagLevTbx_CurrSens): '<S7>/S-Function' */

    /* S-Function Block: <S7>/S-Function */

    /*  Get measurement of current sensor 2U  */
    sfun_MagLevTbx_CurrSens_WrappedOutput(2U, &TeensyNMPC_IO_B.SFunction_i);

    /* S-Function (sfun_MagLevTbx_CurrSens): '<S8>/S-Function' */

    /* S-Function Block: <S8>/S-Function */

    /*  Get measurement of current sensor 3U  */
    sfun_MagLevTbx_CurrSens_WrappedOutput(3U, &TeensyNMPC_IO_B.SFunction_a);

    /* S-Function (sfun_MagLevTbx_UsbSerialPacketSend): '<S11>/S-Function' */
    /* S-Function Block: <S11>/S-Function */

    /*  Pack input signals to payload buffer  */
    {
      uint8_T *pAux = (uint8_T *)TeensyNMPC_IO_DW.SFunction_DWORK1_f[0];

      /*  copy input signals to payload buffer  */
      memcpy(pAux, &TeensyNMPC_IO_B.VectorConcatenate[0], 24);
      pAux += 24;

      /*  copy input signals to payload buffer  */
      memcpy(pAux, &TeensyNMPC_IO_B.SFunction_f, 4);
      pAux += 4;

      /*  copy input signals to payload buffer  */
      memcpy(pAux, &TeensyNMPC_IO_B.SFunction_c, 4);
      pAux += 4;

      /*  copy input signals to payload buffer  */
      memcpy(pAux, &TeensyNMPC_IO_B.SFunction_i, 4);
      pAux += 4;

      /*  copy input signals to payload buffer  */
      memcpy(pAux, &TeensyNMPC_IO_B.SFunction_a, 4);
    }

    /*  Perform COBS encoding  */
    cobsEncode(TeensyNMPC_IO_DW.SFunction_DWORK1_f[0], 40U,
               TeensyNMPC_IO_DW.SFunction_DWORK1_f[1]);

    /*  Add null terminator  */
    ((uint8_T *)TeensyNMPC_IO_DW.SFunction_DWORK1_f[1])[41U] = 0x00;

    /*  Send packet  */
    sfun_MagLevTbx_UsbSerialPacketSend_WrappedOutput( (uint8_T*)
      TeensyNMPC_IO_DW.SFunction_DWORK1_f[1], 42U, 1);
  }
}

/* Model initialize function */
void TeensyNMPC_IO_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* Start for S-Function (sfun_MagLevTbx_UsbSerialPacketReceive): '<S10>/S-Function' */

  /* S-Function Block: <S10>/S-Function */

  /*  Allocate payload buffer  */
  TeensyNMPC_IO_DW.SFunction_DWORK1[0] = calloc(9U, sizeof(uint8_T));

  /*  Allocate packet buffer  */
  TeensyNMPC_IO_DW.SFunction_DWORK1[1] = calloc(10U, sizeof(uint8_T));

  /*  Initialize USB Serial  */
  sfun_MagLevTbx_UsbSerialPacketReceive_WrappedStart();

  /* Start for S-Function (sfun_MagLevTbx_CurrDrv): '<S1>/S-Function' */

  /* S-Function Block: <S1>/S-Function */

  /* Init current driver 0U (first user) */
  sfun_MagLevTbx_CurrDrv_WrappedStart(0U);

  /* Start for S-Function (sfun_MagLevTbx_CurrDrv): '<S2>/S-Function' */

  /* S-Function Block: <S2>/S-Function */

  /* Init current driver 1U (first user) */
  sfun_MagLevTbx_CurrDrv_WrappedStart(1U);

  /* Start for S-Function (sfun_MagLevTbx_CurrDrv): '<S3>/S-Function' */

  /* S-Function Block: <S3>/S-Function */

  /* Init current driver 2U (first user) */
  sfun_MagLevTbx_CurrDrv_WrappedStart(2U);

  /* Start for S-Function (sfun_MagLevTbx_CurrDrv): '<S4>/S-Function' */

  /* S-Function Block: <S4>/S-Function */

  /* Init current driver 3U (first user) */
  sfun_MagLevTbx_CurrDrv_WrappedStart(3U);

  /* Start for S-Function (sfun_MagLevTbx_MagSens_v43): '<S13>/S-Function' */

  /* S-Function Block: <S13>/S-Function */

  /*  Initialize mag sensor  */
  sfun_MagLevTbx_MagSens_WrappedStart();

  /* Start for S-Function (sfun_MagLevTbx_CurrSens): '<S5>/S-Function' */

  /* S-Function Block: <S5>/S-Function */

  /*  Initialize current sensors 0U  */
  sfun_MagLevTbx_CurrSens_WrappedStart(0U);

  /* Start for S-Function (sfun_MagLevTbx_CurrSens): '<S6>/S-Function' */

  /* S-Function Block: <S6>/S-Function */

  /*  Initialize current sensors 1U  */
  sfun_MagLevTbx_CurrSens_WrappedStart(1U);

  /* Start for S-Function (sfun_MagLevTbx_CurrSens): '<S7>/S-Function' */

  /* S-Function Block: <S7>/S-Function */

  /*  Initialize current sensors 2U  */
  sfun_MagLevTbx_CurrSens_WrappedStart(2U);

  /* Start for S-Function (sfun_MagLevTbx_CurrSens): '<S8>/S-Function' */

  /* S-Function Block: <S8>/S-Function */

  /*  Initialize current sensors 3U  */
  sfun_MagLevTbx_CurrSens_WrappedStart(3U);

  /* Start for S-Function (sfun_MagLevTbx_UsbSerialPacketSend): '<S11>/S-Function' */

  /* S-Function Block: <S11>/S-Function */

  /*  Allocate payload buffer  */
  TeensyNMPC_IO_DW.SFunction_DWORK1_f[0] = calloc(40U, sizeof(uint8_T));

  /*  Allocate packet buffer  */
  TeensyNMPC_IO_DW.SFunction_DWORK1_f[1] = calloc(42U, sizeof(uint8_T));

  /*  Initialize USB Serial  */
  sfun_MagLevTbx_UsbSerialPacketSend_WrappedStart();

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay' */
  TeensyNMPC_IO_DW.UnitDelay_DSTATE = TeensyNMPC_IO_P.UnitDelay_InitialCondition;

  /* Enable for S-Function (sfun_MagLevTbx_CurrDrv): '<S1>/S-Function' */
  /* Level2 S-Function Block: '<S1>/S-Function' (sfun_MagLevTbx_CurrDrv) */

  /* S-Function Block: <S1>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_CurrDrv): '<S2>/S-Function' */
  /* Level2 S-Function Block: '<S2>/S-Function' (sfun_MagLevTbx_CurrDrv) */

  /* S-Function Block: <S2>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_CurrDrv): '<S3>/S-Function' */
  /* Level2 S-Function Block: '<S3>/S-Function' (sfun_MagLevTbx_CurrDrv) */

  /* S-Function Block: <S3>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_CurrDrv): '<S4>/S-Function' */
  /* Level2 S-Function Block: '<S4>/S-Function' (sfun_MagLevTbx_CurrDrv) */

  /* S-Function Block: <S4>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_MagSens_v43): '<S13>/S-Function' */
  /* Level2 S-Function Block: '<S13>/S-Function' (sfun_MagLevTbx_MagSens_v43) */

  /* S-Function Block: <S13>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_CurrSens): '<S5>/S-Function' */
  /* Level2 S-Function Block: '<S5>/S-Function' (sfun_MagLevTbx_CurrSens) */

  /* S-Function Block: <S5>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_CurrSens): '<S6>/S-Function' */
  /* Level2 S-Function Block: '<S6>/S-Function' (sfun_MagLevTbx_CurrSens) */

  /* S-Function Block: <S6>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_CurrSens): '<S7>/S-Function' */
  /* Level2 S-Function Block: '<S7>/S-Function' (sfun_MagLevTbx_CurrSens) */

  /* S-Function Block: <S7>/S-Function */

  /* Enable for S-Function (sfun_MagLevTbx_CurrSens): '<S8>/S-Function' */
  /* Level2 S-Function Block: '<S8>/S-Function' (sfun_MagLevTbx_CurrSens) */

  /* S-Function Block: <S8>/S-Function */
}

/* Model terminate function */
void TeensyNMPC_IO_terminate(void)
{
  /* Terminate for S-Function (sfun_MagLevTbx_UsbSerialPacketReceive): '<S10>/S-Function' */

  /* S-Function Block: <S10>/S-Function */

  /*  Release heap space  */
  free(TeensyNMPC_IO_DW.SFunction_DWORK1[0]);
  free(TeensyNMPC_IO_DW.SFunction_DWORK1[1]);

  /* De-init USB Serial (last user) */
  sfun_MagLevTbx_UsbSerialPacketReceive_WrappedTerminate();

  /* Terminate for S-Function (sfun_MagLevTbx_CurrDrv): '<S1>/S-Function' */

  /* S-Function Block: <S1>/S-Function */

  /* De-init current driver 0U (last user) */
  sfun_MagLevTbx_CurrDrv_WrappedTerminate(0U);

  /* Terminate for S-Function (sfun_MagLevTbx_CurrDrv): '<S2>/S-Function' */

  /* S-Function Block: <S2>/S-Function */

  /* De-init current driver 1U (last user) */
  sfun_MagLevTbx_CurrDrv_WrappedTerminate(1U);

  /* Terminate for S-Function (sfun_MagLevTbx_CurrDrv): '<S3>/S-Function' */

  /* S-Function Block: <S3>/S-Function */

  /* De-init current driver 2U (last user) */
  sfun_MagLevTbx_CurrDrv_WrappedTerminate(2U);

  /* Terminate for S-Function (sfun_MagLevTbx_CurrDrv): '<S4>/S-Function' */

  /* S-Function Block: <S4>/S-Function */

  /* De-init current driver 3U (last user) */
  sfun_MagLevTbx_CurrDrv_WrappedTerminate(3U);

  /* Terminate for S-Function (sfun_MagLevTbx_MagSens_v43): '<S13>/S-Function' */

  /* S-Function Block: <S13>/S-Function */
  sfun_MagLevTbx_MagSens_WrappedTerminate();

  /* Terminate for S-Function (sfun_MagLevTbx_CurrSens): '<S5>/S-Function' */

  /* S-Function Block: <S5>/S-Function */
  sfun_MagLevTbx_CurrSens_WrappedTerminate();

  /* Terminate for S-Function (sfun_MagLevTbx_CurrSens): '<S6>/S-Function' */

  /* S-Function Block: <S6>/S-Function */
  sfun_MagLevTbx_CurrSens_WrappedTerminate();

  /* Terminate for S-Function (sfun_MagLevTbx_CurrSens): '<S7>/S-Function' */

  /* S-Function Block: <S7>/S-Function */
  sfun_MagLevTbx_CurrSens_WrappedTerminate();

  /* Terminate for S-Function (sfun_MagLevTbx_CurrSens): '<S8>/S-Function' */

  /* S-Function Block: <S8>/S-Function */
  sfun_MagLevTbx_CurrSens_WrappedTerminate();

  /* Terminate for S-Function (sfun_MagLevTbx_UsbSerialPacketSend): '<S11>/S-Function' */

  /* S-Function Block: <S11>/S-Function */

  /*  Release heap space  */
  free(TeensyNMPC_IO_DW.SFunction_DWORK1_f[0]);
  free(TeensyNMPC_IO_DW.SFunction_DWORK1_f[1]);

  /* De-init USB Serial (last user) */
  sfun_MagLevTbx_UsbSerialPacketSend_WrappedTerminate();
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
