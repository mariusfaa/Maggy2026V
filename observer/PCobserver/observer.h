#pragma once

#include <armadillo>
#include "utilities.h"
#include "kalmanFilter.h"
#include "extendedKalmanFilter.h"

// Extern variables and objects
extern volatile bool newSensorReading;
extern volatile unsigned long observerTime;
extern double NIS;
extern KalmanFilter KF;

// Only declare if defined
#ifdef EXTENDED_KALMAN_FILTER_H
extern ExtendedKalmanFilter EKF;
#endif

// Prototypes
void initObserver();
void runObserver(const double input[NUMBER_INPUTS], const double (*z)[NUMBER_MEASUREMENTS],
    double stateEstimates[NUMBER_STATES]);

