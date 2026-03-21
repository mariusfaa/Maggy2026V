//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: maglevModel.h
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 20-Mar-2026 12:25:01
//

#ifndef MAGLEVMODEL_H
#define MAGLEVMODEL_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void maglevModel_initialize();

extern void maglevModel_terminate();

extern void maglevSystemDynamics_red(const double x[10], const double u[4],
                                     double dx[10]);

extern void maglevSystemMeasurements_red(const double x[10], const double u[4],
                                         double y[3]);

#endif
//
// File trailer for maglevModel.h
//
// [EOF]
//
