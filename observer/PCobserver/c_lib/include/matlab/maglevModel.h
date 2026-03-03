/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: maglevModel.h
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 24-Feb-2026 13:01:23
 */

#ifndef MAGLEVMODEL_H
#define MAGLEVMODEL_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void maglevModel_initialize(void);

extern void maglevModel_terminate(void);

extern void maglevSystemDynamics_fast(const double x[12], const double u[4],
                                      double dx[12]);

extern void maglevSystemMeasurements_fast(const double x[12], const double u[4],
                                          double y[3]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for maglevModel.h
 *
 * [EOF]
 */
