/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ert_main.c
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
#include "rtwtypes.h"
#include "MW_target_hardware_resources.h"

volatile int IsrOverrun = 0;
static boolean_T OverrunFlag = 0;
void rt_OneStep(void)
{
  /* Check for overrun. Protect OverrunFlag against preemption */
  if (OverrunFlag++) {
    IsrOverrun = 1;
    OverrunFlag--;
    return;
  }

  enable_rt_OneStep();
  TeensyNMPC_IO_step();

  /* Get model outputs here */
  disable_rt_OneStep();
  OverrunFlag--;
}

volatile boolean_T stopRequested;
volatile boolean_T runModel;
int main(void)
{
  float modelBaseRate = 0.005;
  float systemClock = 0;

  /* Initialize variables */
  stopRequested = false;
  runModel = false;
  MW_Arduino_Init();
  rtmSetErrorStatus(TeensyNMPC_IO_M, 0);
  TeensyNMPC_IO_initialize();
  cli();
  configureArduinoARMTimer();
  runModel =
    rtmGetErrorStatus(TeensyNMPC_IO_M) == (NULL);
  enable_rt_OneStep();
  sei();
  while (runModel) {
    stopRequested = !(
                      rtmGetErrorStatus(TeensyNMPC_IO_M) == (NULL));
    runModel = !(stopRequested);
    MW_Arduino_Loop();
  }

  /* Terminate model */
  TeensyNMPC_IO_terminate();
  MW_Arduino_Terminate();
  cli();
  return 0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
