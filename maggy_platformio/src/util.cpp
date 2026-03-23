#include "konfig.h"

#ifdef ARDUINO
    #include <Arduino.h>
#else
    #include <stdio.h>
    #include <stdlib.h>
#endif

void SPEW_THE_ERROR(char const * str)
{
#ifdef ARDUINO
    Serial.println(str);
    while (1)
    {
      delay(1000);
      digitalToggle(LED_BUILTIN);
    }
#else
    fprintf(stderr, "ERROR: %s\n", str);
    exit(EXIT_FAILURE);
#endif
}

/**
 * I2C bus recovery - toggles SCL to release stuck slaves and
 * pulses the hardware reset pin if available.
 * Call this before Wire.begin() to recover from locked I2C bus
 * that can occur when MCU resets mid-transaction.
 */
void reset_i2c()
{
#ifdef ARDUINO
  digitalWrite(SDA, HIGH);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(10);

  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(10);
  }

  // Generate STOP condition
  digitalWrite(SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(10);
#endif
}

bool run_every_ms(uint32_t *timer, uint32_t ms)
{
  if (timer == nullptr || ms == 0)
  {
    return false;
  }

  uint32_t now = millis();
  if(*timer == 0)
  {
    *timer = now;
  }

  if ((now - *timer) >= ms) 
  {
    *timer += ms;
    return true;
  }
  return false;
}

bool run_every_us(uint32_t *timer, uint32_t us)
{
  if (timer == nullptr || us == 0)
  {
    return false;
  }

  uint32_t now = micros();
  if(*timer == 0)
  {
    *timer = now;
  }

  if ((now - *timer) >= us) 
  {
    *timer += us;
    return true;
  }
  return false;
}

void print_matrix_double(const char* prefix, const double* mat, size_t rows, size_t cols, uint8_t precision)
{
  if (mat == nullptr || rows == 0 || cols == 0 || prefix == nullptr)
  {
    return;
  }
  if (prefix != nullptr)
  {
    Serial.println(prefix);
  }

    if(rows == 1 || cols == 1)
  {
    // Print as a vector
    for (size_t i = 0; i < rows * cols; ++i)
    {
      Serial.print(mat[i], precision);
      Serial.print("\t");
    }
    Serial.println();
    return;
  }

  for (size_t i = 0; i < rows; ++i)
  {
    for (size_t j = 0; j < cols; ++j)
    {
      Serial.print(mat[i + j*cols], precision);
      Serial.print("\t");
    }
    Serial.println();
  }
}

void print_matrix_float(const char* prefix, const float* mat, size_t rows, size_t cols, uint8_t precision)
{
  if (mat == nullptr || rows == 0 || cols == 0 || prefix == nullptr)
  {
    return;
  }
  if (prefix != nullptr)
  {
    Serial.println(prefix);
  }

  if(rows == 1 || cols == 1)
  {
    // Print as a vector
    for (size_t i = 0; i < rows * cols; ++i)
    {
      Serial.print(mat[i], precision);
      Serial.print("\t");
    }
    Serial.println();
    return;
  }

  for (size_t i = 0; i < rows; ++i)
  {
    for (size_t j = 0; j < cols; ++j)
    {
      Serial.print(mat[i + j*cols], precision);
      Serial.print("\t");
    }
    Serial.println();
  }
}

void signal_sine_generator(float time_sec, float frequency_hz, float amplitude, float* signals)
{
  // shift all 4 signals by same amount
  float phase = time_sec * frequency_hz * 2.0f * PI;
  for (size_t i = 0; i < 4; ++i)
  {
    float phase_offset = phase + (i * (PI / 2.0f)); // 90 degrees phase shift between solenoids
    signals[i] = sinf(phase_offset) * amplitude;
  }
}