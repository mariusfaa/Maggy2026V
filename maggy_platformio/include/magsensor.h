#pragma once

#include <Arduino.h>
#include "TLx493D_inc.hpp"
#include "solenoid.h"

using namespace ifx::tlx493d;

class MagSensor
{
private:
    bool initialized = false;
    bool calibrated = false;

    int freeze_count;
    // z = scaler * (raw - bias)
    Vector3d bias;
    Matrix<double,NUM_SOLENOIDS,3> scaler;
    Vector3d last_raw_measurement;
    Vector3d last_measurement;

public:
    TLx493D_A1B6* _sensor;

    MagSensor(TLx493D_A1B6* sensor);
    bool begin();
    void calibrate(Solenoid* solenoids);
    bool getRawMeasurements(double* x, double* y, double* z);
    bool getRawMeasurements(Vector3d& measurement);
    bool getMeasurements(double* x, double* y, double* z, Solenoid* solenoids);
    bool getMeasurements(Vector3d& measurement, Solenoid* solenoids);
};