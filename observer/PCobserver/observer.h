#pragma once

#include <armadillo>
#include <cstddef>
#include "utilities.h"
#include "kalmanFilter.h"
#include "extendedKalmanFilter.h"
#include "unscentedKalmanFilter.h"

using filterPtr = std::unique_ptr<KalmanFilter>;

filterPtr createObserver(int filterVariant, size_t nx, size_t nb, size_t nu, size_t ny, bool useSRformulation, int RK4Iterations, bool updateJacobians, bool updateQ, bool cubature, bool normalized=0);
filterPtr initObserver(int filterVariant, double dt, double xLp[NUMBER_OBSERVER_STATES], bool useSRformulation=0, int RK4Iterations=0, bool updateJacobians=1, bool updateQ=0, bool cubature=0, bool normalized=1);
void runObserver(const double input[NUMBER_INPUTS], const double meas[NUMBER_MEASUREMENTS],
    double stateEstimates[NUMBER_STATES], KalmanFilter &observer);

