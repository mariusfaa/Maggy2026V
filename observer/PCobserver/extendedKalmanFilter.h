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
  bool useSRformulation;

protected:
  vec dx; // Continuous derivatives

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

  using Base::Ps;
  using Base::Qs;
  using Base::Rs;
  using Base::Ss;

  using Base::I;
  using Base::W;

public:
  derivatives_struct dxd;

  ExtendedKalmanFilter(size_t numberStates, size_t numberInputs, size_t numberMeasurements):
    dx(arma::zeros(NUMBER_STATES_REDUCED)),
    Base(numberStates, numberInputs, numberMeasurements) {
      maglevModel_initialize();
      dxd.dx = &dx;
      dxd.x_next = &x_pred;
      updateJacobians = 1;
      updateQ = 1;
      useSRformulation = 1;
    }


  void predict(vec &u) override {
    // Predict mean
    eulerForward(x_est, u, dt, dxd);

    // Calculates new F
    if (updateJacobians) {
      mat Ac = calculateJacobian(x_est, u, x_pred, 0, 2);
      if (updateQ) {
        van_loan_struct vls = van_loan(Ac, Q, dt);
        F = vls.Ad;
        Q = vls.Qd;
        if (useSRformulation) {
          Qs = chol(Q);
        }
      } else {
        F = discretize_A(Ac, dt);
      }
      // F = calculateJacobian(x_est, u, x_pred, dt, 2);
    }

    // Predict covariance
    if (useSRformulation) {
      Ps = QRr(join_vert(Ps*F.t(), Qs));
    }
    else {
      P = F * P * F.t() + Q;
    }
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

    // Innovation
    v = z - z_pred;

    // Innovation covariance
    if (useSRformulation) {
        Ss = QRr(join_vert(Ps*H.t(), Rs));
    }
    else {
        S = H * P * H.t() + R;

        // Averaging for symmetry. Small regularization for positive definiteness
        S = (S + S.t())*0.5 + eye(nz, nz)*1e-9;
        if (!S.is_sympd()) {
          std::cout << "S is not symmetric positive definite!" << endl;
        }
    }

    
    // Only use to calculate NIS; use solve otherwise
    // mat Sinv = inv(S, inv_opts::likely_sympd);

    // Calculate Kalman gain
    //W = P * H.t() * Sinv;
    if (useSRformulation) {
      mat _temp = solve(trimatl(Ss.t()), H);
      mat __temp = solve(trimatu(Ss), _temp);
      W = (__temp*Ps.t()*Ps).t();
    }
    else {
      W = solve(S, H*P, solve_opts::likely_sympd).t();
    }

    // calculateNIS(v, Sinv);

    // Update state estimate
    x_est = x_pred + W * v;

    // Update covariance estimate
    if (useSRformulation) {
      Ps = QRr(join_vert(Ps*(I-W*H).t(), Rs*W.t()));
    }
    else {
      // P = (I - W * H) * P;
      P = (I - W*H) * P * (I - W*H).t() + W*R*W.t();
    }
  }

};

#endif // EXTENDED_KALMAN_FILTER_H
