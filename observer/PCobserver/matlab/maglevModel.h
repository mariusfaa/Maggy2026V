//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: maglevModel.h
//
// MATLAB Coder version            : 25.2
// C/C++ source code generated on  : 14-Feb-2026 17:04:30
//

#ifndef MAGLEVMODEL_H
#define MAGLEVMODEL_H

// Include Files
#include "maglevModel_spec.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
MAGLEVMODEL_DLL_EXPORT extern void maglevModel_initialize();

MAGLEVMODEL_DLL_EXPORT extern void maglevModel_terminate();

MAGLEVMODEL_DLL_EXPORT extern void
maglevSystemDynamics_fast(const double x[12], const double u[4], double dx[12]);

MAGLEVMODEL_DLL_EXPORT extern void
maglevSystemMeasurements_fast(const double x[12], const double u[4],
                              double y[3]);

#endif
//
// File trailer for maglevModel.h
//
// [EOF]
//
