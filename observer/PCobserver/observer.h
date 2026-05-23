#pragma once

#include <armadillo>
#include <cstddef>
#include "utilities.h"
#include "kalmanFilter.h"
#include "extendedKalmanFilter.h"
#include "unscentedKalmanFilter.h"

using filterPtr = std::unique_ptr<KalmanFilter>;

filterPtr initObserver(int filterVariant, double dt, size_t nx, double *x0ptr, double *Rptr, double *Qptr, double *P0ptr, bool useSRformulation=0, int RK4Iterations=0, bool updateJacobians=1, bool updateQ=0, bool cubature=0);
void runObserver(const double input[NUMBER_INPUTS], const double meas[NUMBER_MEASUREMENTS], double *stateEstimates, KalmanFilter &observer);

