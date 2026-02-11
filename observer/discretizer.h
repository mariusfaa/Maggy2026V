#pragma once

#include "utilities.h"

StateVector eulerForward(const StateVector& x_k, const InputVector& u_k, double dt);
//StateVector RK4step(const StateVector& x_k, const InputVector& u_k, double dt);
