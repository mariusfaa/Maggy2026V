
#include "unscentedKalmanFilter.h"
#include "extendedKalmanFilter.h"
#include "kalmanFilter.h"
#include "matrices.h"
#include "utilities.h"
#include <armadillo>
#include <memory>

using namespace arma;
using filterPtr = std::unique_ptr<KalmanFilter>;

filterPtr createObserver(int filterVariant, size_t nx, size_t nu, size_t nz, bool useSRformulation, bool updateJacobians, bool updateQ, bool cubature) {
  switch (filterVariant) {
    case 0:
      return std::make_unique<KalmanFilter>(nx, nu, nz, useSRformulation);

    case 1:
      return std::make_unique<ExtendedKalmanFilter>(nx, nu, nz, useSRformulation, updateJacobians, updateQ);

    case 2:
      return std::make_unique<UnscentedKalmanFilter>(nx, nu, nz, useSRformulation, cubature);

    default:
      throw std::invalid_argument("Invalid filter variant");
  }
}


// filter: 0 KF, 1 EKF, 2 UKF
filterPtr initObserver(int filterVariant, double dt, bool useSRformulation, bool updateJacobians, bool updateQ, bool cubature) {

  // Initial state
  vec x0(NUMBER_OBSERVER_STATES, arma::fill::zeros);

  // Equilibrium
  double zEq = 0.030119178665731;
  vec xEq(NUMBER_OBSERVER_STATES, arma::fill::zeros);
  vec uEq(NUMBER_INPUTS, arma::fill::zeros);
  xEq(2) = zEq;
  x0(2) = zEq;

  // Function values around equilibrium
  vec dx(NUMBER_OBSERVER_STATES, arma::fill::zeros);
  vec meas(NUMBER_MEASUREMENTS, arma::fill::zeros);
  dynamics_f(xEq, uEq, dx);
  measurements_h(xEq, uEq, meas);

  // Linearizing system
  mat A = calculateJacobian(xEq, uEq, 1, dx, dt, 2);
  // mat A = get_A_cv();
  mat H = calculateJacobian(xEq, uEq, 0, meas, dt, 2);

  // Discretizing system
  mat Q;
  if (NUMBER_OBSERVER_STATES == NUMBER_STATES_TEST) {
    Q = eye(NUMBER_STATES_TEST, NUMBER_STATES_TEST)*1e+1; Q(0,0) = Q(1,1) = 0.0;
  } else {
    Q = get_Q()*1e+1;
  }
  mat B = get_B_fast();
  van_loan_struct vls = van_loan(A, Q, dt);
  mat Ad = vls.Ad;
  mat Qd = vls.Qd;
  mat Bd, P0;
  if (NUMBER_OBSERVER_STATES == NUMBER_STATES_TEST) {
    Bd = zeros(NUMBER_OBSERVER_STATES, NUMBER_INPUTS);
    P0 = eye(NUMBER_STATES_TEST, NUMBER_STATES_TEST);
  } else {
    Bd = discretize_B(A, Ad, B);
    P0 = get_P0()*1e-2;
  }

  mat R = get_R();
  if (NUMBER_OBSERVER_STATES != NUMBER_STATES_TEST) {
    R *= 1e-6; // R is in mT, but filter uses T
  }

  FilterParams params {x0, P0, Ad, Bd, H, Qd, R, dt};

  // Initialize filter
  filterPtr observer = createObserver(filterVariant, NUMBER_OBSERVER_STATES, NUMBER_INPUTS, NUMBER_MEASUREMENTS, useSRformulation, updateJacobians, updateQ, cubature);

  observer->init(params);

  return observer;
}


// Stores estimates in provided stateEstimates array
void runObserver(const double input[NUMBER_INPUTS], const double meas[NUMBER_MEASUREMENTS],
    double stateEstimates[NUMBER_STATES], KalmanFilter &observer) {

  vec u(NUMBER_INPUTS);
  u(0) = input[0];
  u(1) = input[1];
  u(2) = input[2];
  u(3) = input[3];

  vec z(NUMBER_MEASUREMENTS);
  if (NUMBER_OBSERVER_STATES != NUMBER_STATES_TEST) {
    // reading is in mT, but h(x) uses T
    z(0) = meas[0]*1e-3;   // bx
    z(1) = meas[1]*1e-3;   // by
    z(2) = -meas[2]*1e-3;  // bz; is inverted
  } else {
    z(0) = meas[0];
    z(1) = meas[1];
    z(2) = meas[2];
  }

  observer.predict(u);
  observer.update(z);
  vec estimate = observer.getState();

  for (size_t i = 0; i < NUMBER_OBSERVER_STATES; ++i) {
    stateEstimates[i] = estimate(i);
  }

  /*
  if (NUMBER_OBSERVER_STATES != NUMBER_STATES_TEST) {
    // Add back unobserved states as 0
    increaseStateSpace(estimate, stateEstimates);
  } else {
    for (size_t i = 0; i < NUMBER_STATES_TEST; ++i) {
      stateEstimates[i] = estimate(i);
    }
  }
  */
}

