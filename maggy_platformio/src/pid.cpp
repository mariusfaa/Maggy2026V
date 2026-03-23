/*
 * pid.c
 *
 *  Created on: Feb 13, 2024
 *      Author: halvard
 */
#include "pid.h"
#include "konfig.h"

void pid_init(pidc_t* pid, double kp, double ki, double kd)
{
  pid_set_k(pid, kp, ki, kd);
  pid_limits(pid,0,0);
}

void pid_set_k(pidc_t* pid, double kp, double ki, double kd)
{

  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;

  pid_reset(pid);
}

void pid_limits(pidc_t* pid, double minimum, double maximum)
{
  pid->out_min = minimum;
  pid->out_max = maximum;
}

void pid_reset(pidc_t* pid)
{
  pid->lasttime = 0x0fffffff;
}

double calc_angle_error(double setpoint, double measurement)
{
  //double error = wrap_angle(setpoint - measurement); // må gjøres ellers får man store sprang hvis man ex:
  // setpoint = -179 og measurement = 179, som skal gi en error på 2, men -179-179=-358 => wrapped=2

  double angle = setpoint - measurement;

  // wrap angle to <-180, 180]
  while (angle <= -180.0) {
      angle += 360.0;
  }
  while (angle > 180.0) {
      angle -= 360.0;
  }
  return angle;
}

double pid_compute_angle(pidc_t* pid, double setpoint, double measurement)
{
  double error = calc_angle_error(setpoint, measurement);
  return pid_compute(pid, error);
}

double pid_compute_speed(pidc_t* pid, double setpoint, double measurement)
{
  double error = setpoint-measurement;
  return pid_compute(pid, error);
}

double pid_compute(pidc_t* pid, double error)
{
  double dt = (micros() - pid->lasttime) * 1e-6; // convert to seconds
  pid->lasttime = micros();

	double output = 0;

  // P term
  double pterm = pid->Kp * error;
  output += pterm;

	// if firstrun or not run in a long time
  if(dt < 0 || dt > 1.5)
  {
    pid->error_f = error; // initial filtered error
    pid->iterm = 0; // I term reset
    return output;
  }

  double lasterror_f = pid->error_f;
  pid->error_f = iir_filter(lasterror_f,error,0.2);


  // I term
  pid->iterm += pid->Ki * error * dt;
  pid->iterm = clamp(pid->iterm, pid->out_min, pid->out_max); // limit iterm
  output += pid->iterm;

  // D term
  double dterm = pid->Kd * (pid->error_f - lasterror_f)/dt;
  output += dterm;

  // output
  output = clamp(output, pid->out_min, pid->out_max); // limit output

	return output;
}
