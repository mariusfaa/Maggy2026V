#pragma once

#include "../BasicLinearAlgebra/BasicLinearAlgebra.h"

using namespace BLA;
using StateVector = BLA::Matrix<10, 1, double>;
using StateVector_full = BLA::Matrix<12, 1, double>;
using InputVector = BLA::Matrix<4, 1, double>;

void increase_stateSpace(const StateVector& x, double x_pad[12]);

void reduce_stateSpace(double x_pad[12], StateVector& x);

StateVector eulerForward(const StateVector& x_k, const InputVector& u_k, double dt);

StateVector RK4step(const StateVector& x_k, const InputVector& u_k, double dt);
