#pragma once

#include <armadillo>
#include <cstddef>
#include "utilities.h"
#include "kalmanFilter.h"
#include "extendedKalmanFilter.h"
#include "unscentedKalmanFilter.h"

// Externs
extern double NIS;
extern KalmanFilter KF;
extern ExtendedKalmanFilter EKF;
extern UnscentedKalmanFilter UKF;

// Prototypes
void initObserver(size_t filterVariant = 1);
void runObserver(const double input[NUMBER_INPUTS], const double (*z)[NUMBER_MEASUREMENTS],
    double stateEstimates[NUMBER_STATES], size_t filterVariant = 2);

