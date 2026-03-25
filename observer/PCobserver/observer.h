#pragma once

#include <armadillo>
#include <cstddef>
#include "utilities.h"
#include "kalmanFilter.h"
#include "extendedKalmanFilter.h"
#include "unscentedKalmanFilter.h"

using filterPtr = std::unique_ptr<KalmanFilter>;

filterPtr createObserver(int filterVariant, size_t nx, size_t nu, size_t ny, bool useSRformulation, bool updateJacobians, bool updateQ, bool cubature);
filterPtr initObserver(int filterVariant, double dt, bool useSRformulation=0, bool updateJacobians=1, bool updateQ=0, bool cubature=0);
void runObserver(const double input[NUMBER_INPUTS], const double meas[NUMBER_MEASUREMENTS],
    double stateEstimates[NUMBER_STATES], KalmanFilter &observer);

