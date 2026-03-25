#pragma once

#include "utilities.h"
#include "include/matlab/maglevModel.h"
#include <armadillo>

using namespace arma;

template<typename struct_type>
void eulerForward(const vec &x, const vec &u, const double dt, struct_type &s) {

  // Derivative is saved in the value pointed at by s.dx
  dynamics_f(x, u, *s.dx);

  // euler forward
  *s.x_next = x + *s.dx * dt;

}


template<typename struct_type>
void rk4(const vec &x, const vec &u, const double dt, struct_type &s) {

  size_t n = NUMBER_OBSERVER_STATES;

  // Storage for RK4 stages
  vec k1(n, arma::fill::zeros);
  vec k2(n, arma::fill::zeros);
  vec k3(n, arma::fill::zeros);
  vec k4(n, arma::fill::zeros);

  // Temporary vectors for intermediate states
  vec temp(n, arma::fill::zeros);

  // Final next state
  vec x_next(n, arma::fill::zeros);

  // k1: derivative at initial state
  dynamics_f(x, u, k1);

  // k2: derivative at x_k + (dt/2)*k1
  for(int i = 0; i < n; i++) {
    temp(i) = x(i) + 0.5 * dt * k1(i);
  }
  dynamics_f(temp, u, k2);

  // k3: derivative at x_k + (dt/2)*k2
  for(int i = 0; i < n; i++) {
    temp(i) = x(i) + 0.5 * dt * k2(i);
  }
  dynamics_f(temp, u, k3);

  // k4: derivative at x_k + dt*k3
  for(int i = 0; i < n; i++) {
    temp(i) = x(i) + dt * k3(i);
  }
  dynamics_f(temp, u, k4);

  // RK4 update
  x_next = x + (dt/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);

  // Save discretized next value
  *s.x_next = x_next;
}

// Does multiple rk4 step to better simulate continuous dynamics
template<typename struct_type>
void rk4_multi(const vec &x0, const vec &u, const double dt, size_t num_substeps, struct_type &s)
{
    size_t n = NUMBER_OBSERVER_STATES;

    vec x_curr = x0;
    vec x_next(n, arma::fill::zeros);

    double h = dt / static_cast<double>(num_substeps);

    // Temporary struct for inner RK4 calls
    vec dx(n, arma::fill::zeros);
    vec x_tmp(n, arma::fill::zeros);
    struct_type inner = {&dx, &x_tmp};

    for (int i = 0; i < num_substeps; ++i) {
        rk4(x_curr, u, h, inner);
        x_curr = *inner.x_next;
    }

    // Final result
    *s.x_next = x_curr;
}
