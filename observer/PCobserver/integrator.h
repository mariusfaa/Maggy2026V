#pragma once

#include "utilities.h"
#include "include/matlab/maglevModel.h"
#include <armadillo>

using namespace arma;

template<typename struct_type>
void eulerForward(const vec &x_k, const vec &u_k, const double &dt, struct_type &s) {
  double x_k_pad[NUMBER_STATES] = {};
  vec dx(NUMBER_STATES, arma::fill::zeros);
  vec x_next(NUMBER_STATES_REDUCED, arma::fill::zeros);

  increaseStateSpace(x_k, x_k_pad);

  maglevSystemDynamics_fast(x_k_pad, u_k.memptr(), dx.memptr());

  reduceStateSpace(dx.memptr(), x_next);

  // save the reduced form of the derivative
  *s.dx = x_next;

  // euler forward
  x_next = x_k + x_next * dt;

  // save discretized next value
  *s.x_next = x_next;
}

template<typename struct_type>
void rk4(const vec &x_k, const vec &u_k, const double &dt, struct_type &s) {
  // Pad the initial state
  double x_k_pad[NUMBER_STATES] = {};
  increaseStateSpace(x_k, x_k_pad);

  // Storage for RK4 stages
  double k1_pad[NUMBER_STATES] = {};
  double k2_pad[NUMBER_STATES] = {};
  double k3_pad[NUMBER_STATES] = {};
  double k4_pad[NUMBER_STATES] = {};

  // Temporary vectors for intermediate states
  double temp_pad[NUMBER_STATES] = {};

  // Reduced form outputs for each stage
  vec k1_reduced(NUMBER_STATES_REDUCED, arma::fill::zeros);
  vec k2_reduced(NUMBER_STATES_REDUCED, arma::fill::zeros);
  vec k3_reduced(NUMBER_STATES_REDUCED, arma::fill::zeros);
  vec k4_reduced(NUMBER_STATES_REDUCED, arma::fill::zeros);

  // Final next state
  vec x_next(NUMBER_STATES_REDUCED, arma::fill::zeros);

  // k1: derivative at initial state
  maglevSystemDynamics_fast(x_k_pad, u_k.memptr(), k1_pad);
  reduceStateSpace(k1_pad, k1_reduced);

  // k2: derivative at x_k + (dt/2)*k1
  for(int i = 0; i < NUMBER_STATES; i++) {
    temp_pad[i] = x_k_pad[i] + 0.5 * dt * k1_pad[i];
  }
  maglevSystemDynamics_fast(temp_pad, u_k.memptr(), k2_pad);
  reduceStateSpace(k2_pad, k2_reduced);

  // k3: derivative at x_k + (dt/2)*k2
  for(int i = 0; i < NUMBER_STATES; i++) {
    temp_pad[i] = x_k_pad[i] + 0.5 * dt * k2_pad[i];
  }
  maglevSystemDynamics_fast(temp_pad, u_k.memptr(), k3_pad);
  reduceStateSpace(k3_pad, k3_reduced);

  // k4: derivative at x_k + dt*k3
  for(int i = 0; i < NUMBER_STATES; i++) {
    temp_pad[i] = x_k_pad[i] + dt * k3_pad[i];
  }
  maglevSystemDynamics_fast(temp_pad, u_k.memptr(), k4_pad);
  reduceStateSpace(k4_pad, k4_reduced);

  // RK4 update: x_next = x_k + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
  x_next = x_k + (dt/6.0) * (k1_reduced + 2.0*k2_reduced + 2.0*k3_reduced + k4_reduced);

  // Save discretized next value
  *s.x_next = x_next;
}
