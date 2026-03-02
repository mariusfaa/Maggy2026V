#include "konfig.h"
#include "magsensor.h"

constexpr int freeze_reset_count_threshold = 20; // number of consecutive identical readings before reset

MagSensor::MagSensor(TLx493D_A1B6* sensor) : _sensor(sensor)
{
    initialized = false;
    calibrated = false;
    freeze_count = 0;
    bias = Vector3d::Zero();
    scaler = Matrix<double,NUM_SOLENOIDS,3>::Zero();
    last_measurement = Vector3d::Zero();
}

bool MagSensor::begin()
{
    if(initialized)
    {
        return true;
    }

    const int maxRetries = 5;
    for (int tries = 0; tries < maxRetries; ++tries)
    {
        initialized =  _sensor->begin() && _sensor->isFunctional();

        if(initialized)
        {
            // _sensor->setPowerMode(TLx493D_FAST_MODE_e); // Fast mode will be more noisy, but can sample faster. Requires 1Mhz wire speed. This is not supported by the MUX.
            // _sensor->setSensitivity(TLx493D_FULL_RANGE_e);
            return true;
        }

        Serial.printf("Retrying sensor ... attempt %d\n", tries + 1);
        delay(20);
    }
    return initialized;
}

void MagSensor::calibrate(Solenoid* solenoids)
{
    constexpr int settle_delay_us = 3'000; // us
    constexpr int intermediate_delay_us = 200; // us
    constexpr int num_samples_per_solenoid = 500;

    // deactivate solenoids
    for (size_t i = 0; i < NUM_SOLENOIDS; ++i)
    {
        solenoids[i].set_current(0.0f);
    }
    delayMicroseconds(settle_delay_us); // wait for fields to settle

    bias = Vector3d::Zero();
    for (int i = 0; i < num_samples_per_solenoid; ++i)
    {
        double x, y, z;
        if(_sensor->getMagneticField(&x, &y, &z) == false)
        {
            Serial.println("Error reading mag field during calibration");
            continue;
        }
        bias(0) += x;
        bias(1) += y;
        bias(2) += z;
        delayMicroseconds(intermediate_delay_us);
    }
    bias /= static_cast<double>(num_samples_per_solenoid);

    /* Algorithm to estimate scaler matrix
        
            For each solenoid:
            1. Activate solenoid at known current
            2. Measure magnetic field NUM_SAMPLES times, average
            3. Subtract bias
            4. Scaler row = average field / current
            5. Deactivate solenoid
    */
    constexpr double test_current_levels[] = {0.2, 0.4, 0.6};
    constexpr double test_current = 0.2; // 20% duty cycle
    for (size_t i = 0; i < NUM_SOLENOIDS; ++i)
    {
        // Set solenoid to known current
        solenoids[i].set_current(test_current);
        delayMicroseconds(settle_delay_us); // wait for field to settle

        double x, y, z;
        double avg_current = 0;
        Vector3d avg_field = Vector3d::Zero();
        for (int j = 0; j < num_samples_per_solenoid; ++j)
        {
            if(_sensor->getMagneticField(&x, &y, &z) == false)
            {
                Serial.println("Error reading mag field during calibration");
                continue;
            }
            avg_field(0) += x;
            avg_field(1) += y;
            avg_field(2) += z;
            avg_current += solenoids[i].get_current();
            delayMicroseconds(intermediate_delay_us);
        }
        avg_field /= static_cast<double>(num_samples_per_solenoid);
        avg_field -= bias;

        avg_current /= static_cast<double>(num_samples_per_solenoid);

        // The scaler is the field per unit current
        scaler.row(i) = (avg_field / avg_current).transpose();

        // Deactivate solenoid
        solenoids[i].set_current(0.0f);
    }

    calibrated = true;
}

bool MagSensor::getRawMeasurements(double* x, double* y, double* z)
{
    bool status = _sensor->getMagneticField(x, y, z);
    if(status == false)
    {
        // error reading sensor, increment freeze count
        freeze_count++;
    }
    
    // check freeze
    if( (*x == last_raw_measurement(0)) && (*y == last_raw_measurement(1)) && (*z == last_raw_measurement(2)) )
    {
        // No change in readings, likely a freeze
        freeze_count++;
    }

    if(freeze_count >= freeze_reset_count_threshold)
    {
        // Serial.println("Sensor freeze detected, resetting I2C...");
        reset_i2c();
        // delay(100);
        freeze_count = 0; // Reset counter after recovery
    }

    if(status == true)
    {
        last_raw_measurement(0) = *x;
        last_raw_measurement(1) = *y;
        last_raw_measurement(2) = *z;
    }
    return status;
}

bool MagSensor::getRawMeasurements(Vector3d& measurement)
{
    double* x = &measurement(0);
    double* y = &measurement(1);
    double* z = &measurement(2);
    return MagSensor::getRawMeasurements(x, y, z);
}

bool MagSensor::getMeasurements(Vector3d& measurement, Solenoid* solenoids)
{
    // Detrend
    if(!getRawMeasurements(measurement))
    {
        return false;
    }

    Vector4d solenoid_currents = Vector4d::Zero();
    for (size_t i = 0; i < NUM_SOLENOIDS; ++i)
    {
        solenoid_currents(i) = static_cast<double>(solenoids[i].get_current());
    }

    // Estimate field from solenoids
    Vector3d estimate = scaler.transpose() * solenoid_currents + bias;

    // Subtract estimated field
    measurement -= estimate;

    // Apply first order low pass filter to reduce noise
    // constexpr double alpha = 0.8; // smoothing factor (0 < alpha < 1)
    measurement = iir_filter(last_measurement, measurement, 0.2);

    last_measurement = measurement;

    return true;
}

bool MagSensor::getMeasurements(double* x, double* y, double* z, Solenoid* solenoids)
{
    Vector3d measurement;
    if(!getMeasurements(measurement, solenoids))
    {
        return false;
    }
    *x = measurement(0);
    *y = measurement(1);
    *z = measurement(2);
    return true;
}