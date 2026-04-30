/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: TeensyNMPC_IO_private.h
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

#ifndef TeensyNMPC_IO_private_h_
#define TeensyNMPC_IO_private_h_
#include "rtwtypes.h"
#include "TeensyNMPC_IO_types.h"
#include "TeensyNMPC_IO.h"

extern void sfun_MagLevTbx_UsbSerialPacketReceive_WrappedStart();
extern size_t sfun_MagLevTbx_UsbSerialPacketReceive_WrappedOutput( uint8_T
  *pPacketBuffer, uint32_T packetSize, boolean_T nullTerminated);
extern void sfun_MagLevTbx_UsbSerialPacketReceive_WrappedTerminate();
extern void sfun_MagLevTbx_CurrDrv_WrappedStart(uint8_T driverId);
extern void sfun_MagLevTbx_CurrDrv_WrappedOutput(uint8_T driverId, int16_T *u0);
extern void sfun_MagLevTbx_CurrDrv_WrappedTerminate(uint8_T driverId);
extern void sfun_MagLevTbx_MagSens_WrappedStart(void);
extern void sfun_MagLevTbx_MagSens_WrappedOutput(double *y0);
extern void sfun_MagLevTbx_MagSens_WrappedTerminate(void);
extern void sfun_MagLevTbx_CurrSens_WrappedStart(uint8_T sensorId);
extern void sfun_MagLevTbx_CurrSens_WrappedOutput(uint8_T sensorId, float *y0);
extern void sfun_MagLevTbx_CurrSens_WrappedTerminate(void);
extern void sfun_MagLevTbx_UsbSerialPacketSend_WrappedStart();
extern void sfun_MagLevTbx_UsbSerialPacketSend_WrappedOutput(uint8_T
  *pPacketBuffer, uint32_T packetSize, boolean_T waitDTR);
extern void sfun_MagLevTbx_UsbSerialPacketSend_WrappedTerminate();

#endif                                 /* TeensyNMPC_IO_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
