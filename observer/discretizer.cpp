
#include "../BasicLinearAlgebra/BasicLinearAlgebra.h"
#include "maglevModel.h"

using namespace BLA;
using StateVector = BLA::Matrix<10, 1, double>;
using StateVector_full = BLA::Matrix<12, 1, double>;
using InputVector = BLA::Matrix<4, 1, double>;

void increase_stateSpace(const StateVector& x, double x_pad[12]) {
  for (int i = 0; i < 12; i++) {
    if ((i == 5) || (i == 11)) {
      x_pad[i] = 0;
    } else {
      x_pad[i] = x(i);
    }
  }
}

void reduce_stateSpace(double x_pad[12], StateVector& x) {
  int offset = 0;
  for (int i = 0; i < 11; i++) {
    if (i == 5) {
      offset = 1;
    } else {
      x(i - offset) = x_pad[i];
    }
  }
}


StateVector eulerForward(const StateVector& x_k, const InputVector& u_k, double dt) {
  StateVector_full dx, x_k_pad;
  StateVector x_next;

  increase_stateSpace(x_k, x_k_pad.storage);

  //maglevSystemDynamics_fast(x_k.storage, u_k.storage, dx.storage);

  reduce_stateSpace(dx.storage, x_next);

  // euler forward
  x_next = x_k + x_next * dt;

  return x_next;
}


StateVector RK4step(const StateVector& x_k, const InputVector& u_k, double dt) {
  StateVector_full k1, k2, k3, k4, x_temp, x_k_pad;
  StateVector x_next;

  increase_stateSpace(x_k, x_k_pad.storage);

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

  reduce_stateSpace(x_temp.storage, x_next);

  return x_next;
}
