#pragma once

#include "konfig.h"

class Solenoid
{
private:
  uint8_t pin_in1;
  uint8_t pin_in2;
  uint8_t pin_readcurrent;

  float target_current; // last set pwm factor
  float previous_target_current;
  float estimated_current;
  uint32_t current_change_time;

  // calibration vars
  float mean;
  float scaler;
public:
  Solenoid(uint8_t pin_in1, uint8_t pin_in2, uint8_t pin_readcurrent);
  ~Solenoid();

  void initialize();
  void calibrate();
  float get_current();
  void set_current(float factor);

  
  uint16_t get_raw_current();
  float get_target_current();
  float get_estimated_current();

  void self_test();
};