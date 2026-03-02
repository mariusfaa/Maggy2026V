/*
 * pid.h
 *
 *  Created on: Feb 13, 2024
 *      Author: halvard
 */
// Original fra https://github.com/geekfactory/PID (12/02/24)
// har blitt redigert til eget bruk
#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
  double Kp;
  double Ki;
  double Kd;

  double error_f;
  double iterm;

  double out_min;
  double out_max;

  uint32_t lasttime;
} pidc_t;

void pid_init(pidc_t* pid, double kp, double ki, double kd);
void pid_set_k(pidc_t* pid, double kp, double ki, double kd);
void pid_limits(pidc_t* pid, double minimum, double maximum);
double pid_compute_angle(pidc_t* pid, double setpoint, double measurement);
double pid_compute_speed(pidc_t* pid, double setpoint, double measurement);
double pid_compute(pidc_t* pid, double error);
double calc_angle_error(double setpoint, double measurement);
void pid_reset(pidc_t* pid);

#endif
