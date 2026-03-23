/*************************************************************************************************************
 * This file contains configuration parameters
 ************************************************************************************************************/
#ifndef KONFIG_H
#define KONFIG_H

#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>


#define ENABLE_LOG_TO_SD_CARD


// === Constants and Macros ===
// I2C reset pin
#define RESET_I2C 26
#define SDA_PIN 18
#define SCL_PIN 19

// Motor driver pins
#define MD1_IN1 4
#define MD1_IN2 5
#define MD2_IN1 2
#define MD2_IN2 3
#define MD3_IN1 6
#define MD3_IN2 7
#define MD4_IN1 8
#define MD4_IN2 9

// Builtin SD card pin
#ifndef BUILTIN_SDCARD
#define BUILTIN_SDCARD 254
#endif    

// Current sensor pins
#define CURRENT_Y_POS 20
#define CURRENT_X_NEG 21
#define CURRENT_X_POS 22
#define CURRENT_Y_NEG 23

// Sensor configuration
#define NUM_SOLENOIDS 4
#define NUM_SENSORS 1
extern const int SENSOR_CHANNELS[NUM_SENSORS];
#define PRIMARY_SENSOR_INDEX 0
#define POWER_SUPPLY_MAX_AMPS 4.0

#define ANALOG_READ_BIT_RANGE 10 // max recommended is 10 bits, due to noise: https://www.pjrc.com/store/teensy41.html#analog
#define ANALOG_MAX ((1ULL << (ANALOG_READ_BIT_RANGE)) - 1ULL)
#define PWM_BIT_RANGE 12 // see https://www.pjrc.com/teensy/td_pulse.html for reference
#define PWM_MAX ((1ULL << (PWM_BIT_RANGE)) - 1ULL) // here not -1 to be able to set pin high
#define PWM_DRIVER_FREQ 36621.09 // configured for PWM_BIT_RANGE, see https://www.pjrc.com/teensy/td_pulse.html#:~:text=table
#define SOFTRESET() SCB_AIRCR = 0x05FA0004;

/* State Space dimension */
#define SS_X_LEN    (10)
#define SS_Z_LEN    (3)
#define SS_U_LEN    (4)

/* EKF Parameters */
/* TODO */


/* MPC Parameters */
/* TODO */

/* ASSERT is evaluated locally (without function call) to lower the computation cost */
void SPEW_THE_ERROR(char const * str);
#define ASSERT(truth, str) while(0){ if (!(truth)) SPEW_THE_ERROR(str); }

void signal_sine_generator(float time_sec, float frequency_hz, float amplitude, float* signals);
void print_matrix_double(const char* prefix, const double* mat, size_t rows, size_t cols, uint8_t precision=2);
void print_matrix_float(const char* prefix, const float* mat, size_t rows, size_t cols, uint8_t precision=4);
bool run_every_ms(uint32_t *timer, uint32_t ms);
bool run_every_us(uint32_t *timer, uint32_t us);
void reset_i2c();

#define iir_filter(old_value, measurement, alpha) (alpha*measurement + (1.0-alpha)*old_value)
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define clamp(value, minimum, maximum) (((value) < (minimum)) ? (minimum) : (((value) > (maximum)) ? (maximum) : (value)))


#define US_TO_S(us)   (float_prec(us) * 1e-6f)

enum class SolenoidID : uint8_t {
  X_POS = 0,
  X_NEG,
  Y_POS,
  Y_NEG
};

#endif // KONFIG_H
