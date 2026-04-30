/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ert_main.c
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
#include "rtwtypes.h"
#include "MW_target_hardware_resources.h"

volatile int IsrOverrun = 0;
boolean_T isRateRunning[2] = { 0, 0 };

boolean_T need2runFlags[2] = { 0, 0 };

void rt_OneStep(void)
{
  boolean_T eventFlags[2];

  /* Check base rate for overrun */
  if (isRateRunning[0]++) {
    IsrOverrun = 1;
    isRateRunning[0]--;                /* allow future iterations to succeed*/
    return;
  }

  /*
   * For a bare-board target (i.e., no operating system), the rates
   * that execute this base step are buffered locally to allow for
   * overlapping preemption.
   */
  CurrDrvSensCombo_Test_SetEventsForThisBaseStep(eventFlags);
  enable_rt_OneStep();
  CurrDrvSensCombo_Test_step0();

  /* Get model outputs here */
  disable_rt_OneStep();
  isRateRunning[0]--;
  if (eventFlags[1]) {
    if (need2runFlags[1]++) {
      IsrOverrun = 1;
      need2runFlags[1]--;              /* allow future iterations to succeed*/
      return;
    }
  }

  if (need2runFlags[1]) {
    if (isRateRunning[1]) {
      /* Yield to higher priority*/
      return;
    }

    isRateRunning[1]++;
    enable_rt_OneStep();

    /* Step the model for subrate "1" */
    switch (1)
    {
     case 1 :
      CurrDrvSensCombo_Test_step1();

      /* Get model outputs here */
      break;

     default :
      break;
    }

    disable_rt_OneStep();
    need2runFlags[1]--;
    isRateRunning[1]--;
  }
}

volatile boolean_T stopRequested;
volatile boolean_T runModel;
int main(void)
{
  float modelBaseRate = 0.01;
  float systemClock = 0;

  /* Initialize variables */
  stopRequested = false;
  runModel = false;
  MW_Arduino_Init();
  rtmSetErrorStatus(CurrDrvSensCombo_Test_M, 0);
  CurrDrvSensCombo_Test_initialize();
  cli();
  configureArduinoARMTimer();
  runModel =
    rtmGetErrorStatus(CurrDrvSensCombo_Test_M) == (NULL);
  enable_rt_OneStep();
  sei();
  while (runModel) {
    stopRequested = !(
                      rtmGetErrorStatus(CurrDrvSensCombo_Test_M) == (NULL));
    runModel = !(stopRequested);
    MW_Arduino_Loop();
  }

  /* Terminate model */
  CurrDrvSensCombo_Test_terminate();
  MW_Arduino_Terminate();
  cli();
  return 0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
