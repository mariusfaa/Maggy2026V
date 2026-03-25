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
  vec dx; // Continuous derivatives
  derivatives_struct dxd;

  using Base::useSRformulation;

  using Base::dt;

  using Base::nx;
  using Base::nu;
  using Base::nz;

  using Base::x_est;
  using Base::x_pred;
  using Base::z_pred;
  using Base::innovation;

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
  ExtendedKalmanFilter(size_t numberStates, size_t numberInputs, size_t numberMeasurements, bool useSRformulation, bool updateJacobians=1, bool updateQ=1):
    dx(arma::zeros(numberStates)),
    updateJacobians(updateJacobians),
    updateQ(updateQ),
    Base(numberStates, numberInputs, numberMeasurements, useSRformulation) {
      maglevModel_initialize();
      dxd.dx = &dx;
      dxd.x_next = &x_pred;
    }


   virtual void predict(vec &u) override {
    // Predict mean
    // eulerForward(x_est, u, dt, dxd);
    rk4_multi(x_est, u, dt, 10, dxd);

    // Calculates new F
    if (updateJacobians) {
      mat Ac = calculateJacobian(x_est, u, 1, x_pred, 0, 2);
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
      // F = calculateJacobian(x_est, u, 1, x_pred, dt, 2);
    }

    // Predict covariance
    if (useSRformulation) {
      mat _Q;
      qr_econ(_Q, Ps, join_vert(Ps*F.t(), Qs));
    }
    else {
      P = F * P * F.t() + Q;
      // Averaging for symmetry. Small regularization for positive definiteness
      P = (P + P.t())*0.5 + eye(nx, nx)*1e-9;
      if (!P.is_sympd(1e-9)) {
        std::cout << "P is not symmetric positive definite!" << endl;
      }
    }
  }

  virtual void update(vec &z) override {

    // Assuming feedthrough is compensated for
    vec u = zeros(nu);

    // z_pred = h(x)
    measurements_h(x_est, u, z_pred);

    // Calculates new H
    if (updateJacobians) {
      H = calculateJacobian(x_pred, u, 0, z_pred, 0, 2);
    }

    // Innovation
    innovation = z - z_pred;

    // Innovation covariance
    if (useSRformulation) {
        mat _Q;
        qr_econ(_Q, Ss, join_vert(Ps*H.t(), Rs));
    }
    else {
        S = H * P * H.t() + R;

        // Averaging for symmetry. Small regularization for positive definiteness
        S = (S + S.t())*0.5 + eye(nz, nz)*1e-9;
        if (!S.is_sympd(1e-9)) {
          std::cout << "S is not symmetric positive definite!" << endl;
        }
    }

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

    // Update state estimate
    x_est = x_pred + W * innovation;

    // Update covariance estimate
    if (useSRformulation) {
      mat _Q;
      qr_econ(_Q, Ps, join_vert(Ps*(I-W*H).t(), Rs*W.t()));
    }
    else {
      // P = (I - W * H) * P;
      P = (I - W*H) * P * (I - W*H).t() + W*R*W.t();
      if (!P.is_sympd(1e-9)) {
        std::cout << "P is not symmetric positive definite!" << endl;
      }
    }
  }

};

#endif // EXTENDED_KALMAN_FILTER_H
