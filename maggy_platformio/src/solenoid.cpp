#include "konfig.h"

#ifdef ARDUINO

#include "solenoid.h"

Solenoid::Solenoid(uint8_t pin_in1, uint8_t pin_in2, uint8_t pin_readcurrent) : pin_in1(pin_in1), pin_in2(pin_in2), pin_readcurrent(pin_readcurrent), mean(0), scaler(0)
{

}

void Solenoid::initialize()
{
  // Output pins for pwm drivers that drives the solenoids
  pinMode(pin_in1, OUTPUT);
  pinMode(pin_in2, OUTPUT);

  // Input pin to read analog current of solenoid
  pinMode(pin_readcurrent, INPUT);

  // Set PWM output frequency
  analogWriteFrequency(pin_in1, PWM_DRIVER_FREQ);
  analogWriteFrequency(pin_in2, PWM_DRIVER_FREQ);

  // Setting initial state to 0 (idle)
  digitalWrite(pin_in1, LOW);
  digitalWrite(pin_in2, LOW);
}

Solenoid::~Solenoid()
{
  // Setting state to 0 (idle)
  digitalWrite(pin_in1, LOW);
  digitalWrite(pin_in2, LOW);
}

void Solenoid::calibrate()
{
  constexpr uint32_t settle_time_us = 3000; // is 2ms at 15V(checked via oscilloscope), so set to 3ms for good measure
  constexpr uint32_t num_samples = 5000;
  // get mean
  mean = 0;
  this->set_current(0.0f);
  delayMicroseconds(settle_time_us);

  float accumulated_current = 0;
  for(uint32_t i = 0; i < num_samples; i++)
  {
    // accumulate samples
    accumulated_current += this->get_current();
    delayMicroseconds(7);
  }
  this->mean = accumulated_current / num_samples;
}

uint16_t Solenoid::get_raw_current()
{
  uint16_t data = analogRead(this->pin_readcurrent);
  return data;
}

float Solenoid::get_current()
{
  // Currently (maggy 4.2 has 1A current limit)
  float raw = static_cast<float>(this->get_raw_current());
  float voltage = 3.3f * raw / static_cast<float>(ANALOG_MAX);     // ADC to voltage
  float voltage_diff = voltage - 1.65f;       // Centered around 1.65V (no current)
  float current = voltage_diff/(100.0*0.015); // Gain = 100 (INA214), Rshunt = 0.015Ω

  current -= mean;
  return current;
}

float Solenoid::get_target_current()
{
  return target_current;
}

void Solenoid::set_current(float factor)
{
  // New current is same as previous
  if(factor == target_current)
  {
    return;
  }

  // Update vars for estimating current
  current_change_time = micros();
  previous_target_current = target_current;
  target_current = factor;
  
  // Set new target current
  int pwm = factor * PWM_MAX;
  if (pwm < 0) {
    analogWrite(pin_in1, PWM_MAX - abs(pwm));
    analogWrite(pin_in2, PWM_MAX);
  } else if (pwm > 0) {
    analogWrite(pin_in1, PWM_MAX);
    analogWrite(pin_in2, PWM_MAX - abs(pwm));
  } else {
    // Setting both pins low => coast, enters Low Power Standby mode after 1 ms, high impedance
    // Setting both pins high => Brake (slow decay), low impedance 
    analogWrite(pin_in1, 0);
    analogWrite(pin_in2, 0);
  }
}

float Solenoid::get_estimated_current()
{
  
  constexpr float tau = 0.0003f;
  float t = (micros() - current_change_time) * 1e-6f;
  float& x0 = previous_target_current;
  float& x = target_current;
  estimated_current = x0 + (x - x0) * (1.0f - exp(-t/tau));
  return estimated_current;
}

void Solenoid::self_test()
{
  uint32_t start = micros();
  uint32_t timer = millis();
  while(1)
  {
    const float t = (micros()-start) * 1e-6f;

    float current = this->get_current();
    float pwm_factor = sinf(t*2.0f*PI) * 0.05;
    this->set_current(pwm_factor);

    if (run_every_ms(&timer, 10))
    {
      Serial.printf("t:%.6f,I:%.6f,u:%.6f\n",
                    t,
                    current,
                    pwm_factor * 100.0f);
    }

    if((timer-start*1e-3) > 2000) // stop after x sec
    {
      delay(500);
      return;
    }
  }
}

#endif