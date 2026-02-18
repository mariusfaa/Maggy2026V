
/*
#include "discretizer.h"
#include "matlab/maglevModel.h"

vec eulerForward(const vec& x_k, const vec& u_k, double dt) {
  vec dx, x_k_pad;
  vec x_next;

  increaseStateSpace(x_k, x_k_pad.memptr());

  maglevSystemDynamics_fast(x_k.memptr(), u_k.memptr(), dx.memptr());

  reduceStateSpace(dx.memptr(), x_next);

  // euler forward
  x_next = x_k + x_next * dt;

  return x_next;
}
*/
/*
StateVector RK4step(const StateVector& x_k, const InputVector& u_k, double dt) {
  StateVectorFull k1, k2, k3, k4, x_temp, x_k_pad;
  StateVector x_next;

  increaseStateSpace(x_k, x_k_pad.storage);

  // Stage 1: k1 = f(x_k, u_k)
  maglevSystemDynamics_fast(x_k_pad.storage, u_k.storage, k1.storage);

  // Stage 2: k2 = f(x_k + dt/2 * k1, u_k)
  x_temp = x_k_pad + k1 * (dt / 2.0);
  maglevSystemDynamics_fast(x_temp.storage, u_k.storage, k2.storage);

  // Stage 3: k3 = f(x_k + dt/2 * k2, u_k)
  x_temp = x_k_pad + k2 * (dt / 2.0);
  maglevSystemDynamics_fast(x_temp.storage, u_k.storage, k3.storage);

  // Stage 4: k4 = f(x_k + dt * k3, u_k)
  x_temp = x_k_pad + k3 * dt;
  maglevSystemDynamics_fast(x_temp.storage, u_k.storage, k4.storage);

  // RK4 update
  x_temp = x_k_pad + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * (dt / 6.0);

  reduceStateSpace(x_temp.storage, x_next);

  return x_next;
}
*/
