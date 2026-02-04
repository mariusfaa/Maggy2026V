#pragma once

#include "../BasicLinearAlgebra/BasicLinearAlgebra.h"
#include "observerDefinitions.h"
#include "kalmanFilter.h"


using namespace BLA;

extern volatile bool newSensorReading;
extern double stateEstimates[12];
extern BLA::Matrix<NUMBER_MEASUREMENTS, 1, double> innovation_means;
extern KalmanFilter<NUMBER_STATES_REDUCED,
                    NUMBER_INPUTS,
                    NUMBER_MEASUREMENTS> KF;

void newSensorReading_callback();
void initObserver();
void runObserver(const float pwmInputX, const float pwmInputY, const float (*sensorReading)[NUMBER_MEASUREMENTS]);
