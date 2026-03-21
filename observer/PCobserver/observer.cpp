
#include "unscentedKalmanFilter.h"
#include "extendedKalmanFilter.h"
#include "kalmanFilter.h"
#include "matrices.h"
#include "utilities.h"
#include <armadillo>

#define NUMBER_OBSERVER_STATES NUMBER_STATES_REDUCED

using namespace arma;

KalmanFilter KF(NUMBER_OBSERVER_STATES, NUMBER_INPUTS, NUMBER_MEASUREMENTS, 0, 1);
ExtendedKalmanFilter EKF(NUMBER_OBSERVER_STATES, NUMBER_INPUTS, NUMBER_MEASUREMENTS, 0, 1);
UnscentedKalmanFilter UKF(NUMBER_OBSERVER_STATES, NUMBER_INPUTS, NUMBER_MEASUREMENTS, 0, 1);


// Filter independent way of running an entire estimating cycle
template<typename FilterType>
vec runFilter(FilterType &filter, vec &u, vec &z) {
  filter.predict(u);
  filter.update(z);
  return filter.getState();
}


// filter: 0 KF, 1 EKF, 2 UKF
void initObserver(size_t filterVariant) {
  double dt = 0.01; // 100 Hz

  // Initial state
  vec x0(NUMBER_OBSERVER_STATES, arma::fill::zeros);

  // Equilibrium
  double zEq = 0.030119178665731;
  vec xEq(NUMBER_OBSERVER_STATES, arma::fill::zeros);
  vec uEq(NUMBER_INPUTS, arma::fill::zeros);
  xEq(3) = zEq;

  // Function values around equilibrium
  vec dx(NUMBER_OBSERVER_STATES, arma::fill::zeros);
  vec meas(NUMBER_MEASUREMENTS, arma::fill::zeros);
  dynamics_f(xEq, uEq, dx);
  measurements_h(xEq, uEq, meas);

  // Linearizing system
  mat A = calculateJacobian(xEq, uEq, 1, dx, dt, 2);
  mat H = calculateJacobian(xEq, uEq, 0, meas, dt, 2);

  // Discretizing system
  mat Q = get_Q();
  mat B = get_B_fast();
  van_loan_struct vls = van_loan(A, Q, dt);
  mat Ad = vls.Ad;
  mat Qd = vls.Qd;

  mat Bd = discretize_B(A, Ad, B);

  mat P0 = get_P0();
  mat R = get_R();

  // Initialize filter
  switch (filterVariant) {
    case 0:
      KF.init(x0, P0, Ad, Bd, Qd, H, R, dt);
      break;

    case 1:
      EKF.init(x0, P0, Ad, Bd, Qd, H, R, dt);
      break;

    case 2:
      UKF.init(x0, P0, Qd, R, dt);
      break;
  }
}


// Stores estimates in provided stateEstimates array
void runObserver(const double input[NUMBER_INPUTS], const double z[NUMBER_MEASUREMENTS],
    double stateEstimates[NUMBER_STATES], size_t filterVariant) {

  vec u(NUMBER_INPUTS);
  u(0) = input[0];
  u(1) = input[1];
  u(2) = input[2];
  u(3) = input[3];
  vec meas(NUMBER_MEASUREMENTS);

  bool sim = 1;
  if (!sim) {
  // reading is in mT, but h(x) uses T
  meas(0) = z[0]*1e-3;   // bx
  meas(1) = z[1]*1e-3;   // by
  meas(2) = -z[2]*1e-3;  // bz; is inverted
  }
  else {
    meas(0) = z[0];
    meas(1) = z[1];
    meas(2) = z[2];
  }


  vec estimate;

  switch (filterVariant) {
    case 0:
      estimate = runFilter(KF, u, meas);
      break;

    case 1:
      estimate = runFilter(EKF, u, meas);
      break;

    case 2:
      estimate = runFilter(UKF, u, meas);
      break;
  }

  // Add back rotation around z axis as 0
  increaseStateSpace(estimate, stateEstimates);
}

