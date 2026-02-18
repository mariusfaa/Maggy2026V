#pragma once

#include "utilities.h"
#include "matlab/maglevModel.h"

using namespace arma;

template<typename dt_type>
vec eulerForward(const vec& x_k, const vec& u_k, dt_type dt) {
  vec dx, x_k_pad;
  vec x_next;

  increaseStateSpace(x_k, x_k_pad.memptr());

  maglevSystemDynamics_fast(x_k.memptr(), u_k.memptr(), dx.memptr());

  reduceStateSpace(dx.memptr(), x_next);

  // euler forward
  x_next = x_k + x_next * dt;

  return x_next;
}

//StateVector RK4step(const StateVector& x_k, const InputVector& u_k, double dt);
