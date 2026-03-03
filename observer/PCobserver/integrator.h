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

