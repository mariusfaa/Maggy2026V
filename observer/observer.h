#pragma once

#include "../BasicLinearAlgebra/BasicLinearAlgebra.h"
#include "utilities.h"
#include "kalmanFilter.h"
//#include "extendedKalmanFilter.h"

// Extern variables and objects
extern volatile bool newSensorReading;
extern volatile unsigned long observerTime;
extern double stateEstimates[NUMBER_STATES];
extern MeasVector innovation_means;
extern KalmanFilter KF;

// Only declare if defined
#ifdef EXTENDED_KALMAN_FILTER_H
extern ExtendedKalmanFilter EKF;
#endif

// Prototypes
void initObserver();
void runObserver(const float ux, const float uy, const float (*z)[NUMBER_MEASUREMENTS]);

