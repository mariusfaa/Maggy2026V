#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include "kalmanFilter.h"
#include "integrator.h"
#include "utilities.h"
#include "include/matlab/maglevModel.h"
#include <algorithm>
#include <armadillo>
#include <cstddef>

class ExtendedKalmanFilter: public KalmanFilter {
  using Base = KalmanFilter;

private:
  bool updateJacobians;
  bool updateQ;

protected:
  vec dx;

  using Base::dt;

  using Base::nx;
  using Base::nu;
  using Base::nz;

  using Base::x_est;
  using Base::x_pred;
  using Base::z_pred;

  using Base::F;
  using Base::H;

  using Base::P;
  using Base::Q;
  using Base::R;
  using Base::S;

  using Base::I;
  using Base::W;

public:
  derivatives_struct dxd;

  // Constructor
  ExtendedKalmanFilter(size_t numberStates, size_t numberInputs, size_t numberMeasurements):
    dx(arma::zeros(NUMBER_STATES_REDUCED)),
    Base(numberStates, numberInputs, numberMeasurements) {
      maglevModel_initialize();
      dxd.dx = &dx;
      dxd.x_next = &x_pred;
      updateJacobians = 1;
      updateQ = 0;
    }


  void predict(vec &u) override {
    // x = f(x,u)
    eulerForward(x_est, u, dt, dxd);

    mat Ac;
    // Calculates new F
    if (updateJacobians) {
      // Ac = calculateJacobian(x_est, u, x_pred, 0, 2);
      // F = discretize_A(Ac, dt);
      F = calculateJacobian(x_est, u, x_pred, dt, 2);
    }

    // P = F * P * F^T + Q
    P = F * P * F.t() + Q;
  }

  void update(vec &z) override {
    double x_pad[NUMBER_STATES] = {};

    // Assuming feedthrough is compensated for
    vec u = zeros(nu);

    // z_pred = h(x)
    increaseStateSpace(x_est, x_pad);
    maglevSystemMeasurements_fast(x_pad, u.memptr(), z_pred.memptr());

    // Calculates new H
    if (updateJacobians) {
      H = calculateJacobian(x_pred, u, z_pred, 0, 2);
    }

    // Calculate innovation
    v = z - z_pred;

    // Calculate innovation covariance
    S = H * P * H.t() + R;

    // Only use to calculate NIS; use solve otherwise
    // mat Sinv = inv(S, inv_opts::likely_sympd);

    // Calculate Kalman gain
    //W = P * H.t() * Sinv;
    // S W^T = (P H^T)^T
    W = solve(S, H*P, solve_opts::likely_sympd).t();

    // calculateNIS(v, Sinv);

    // Update state estimate
    x_est = x_pred + W * v;

    // Update covariance estimate
    // P = (I - W * H) * P;
    P = (I - W*H) * P * (I - W*H).t() + W*R*W.t();
  }

};

#endif // EXTENDED_KALMAN_FILTER_H
