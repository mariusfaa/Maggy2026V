#include "unscentedKalmanFilter.h"
#include "extendedKalmanFilter.h"
#include "kalmanFilter.h"
#include "matrices.h"
#include "utilities.h"
#include <armadillo>
#include <cstddef>
#include <cstdlib>
#include <memory>

using namespace arma;
using filterPtr = std::unique_ptr<KalmanFilter>;

filterPtr createObserver(int filterVariant, size_t nx, size_t nb, size_t nu, size_t nz, bool useSRformulation, int RK4Iterations, bool updateJacobians, bool updateQ, bool cubature, bool normalized) {
  switch (filterVariant) {
    case 0:
      return std::make_unique<KalmanFilter>(nx, nb, nu, nz, useSRformulation);

    case 1:
      return std::make_unique<ExtendedKalmanFilter>(nx, nb, nu, nz, useSRformulation, RK4Iterations, updateJacobians, updateQ);

    case 2:
      return std::make_unique<UnscentedKalmanFilter>(nx, nb, nu, nz, useSRformulation, RK4Iterations, cubature, normalized);

    default:
      throw std::invalid_argument("Invalid filter variant");
  }
}


// filter: 0 KF, 1 EKF, 2 UKF
filterPtr initObserver(int filterVariant, double dt, double x0[NUMBER_OBSERVER_STATES], bool useSRformulation, int RK4Iterations, bool updateJacobians, bool updateQ, bool cubature, bool normalized) {

  size_t nx = NUMBER_OBSERVER_STATES;
  size_t nb = NUMBER_BIAS_STATES; // Only available for linear KF
  size_t nu = NUMBER_INPUTS;
  size_t nz = NUMBER_MEASUREMENTS;

  bool onlyDisplacement = nx == NUMBER_STATES_REDUCED_EXTRA;

  // x0 acts as linearization point
  vec xLp(NUMBER_OBSERVER_STATES, arma::fill::zeros);
  for (size_t i = 0; i < NUMBER_OBSERVER_STATES; ++i) {
    xLp(i) = x0[i];
  }
  vec uLp(NUMBER_INPUTS, arma::fill::zeros);

  // Function values around equilibrium
  vec dx(nx, arma::fill::zeros);
  vec meas(nz, arma::fill::zeros);
  dynamics_f(xLp, uLp, dx);
  measurements_h(xLp, uLp, meas);

  // Linearizing system
  mat A = calculateJacobian(xLp, uLp, 0, dx, dt, 2);
  mat H = calculateJacobian(xLp, uLp, 1, meas, dt, 2);

  if (nb != 0) {
    A = join_horiz(join_vert(A, zeros(nb, nx)), zeros(nx+nb, nb));
    H = join_horiz(H, zeros(nz, nb));

    for (size_t i = 0; i < nb; ++i) {
      H(i, nx+i) = 1;
    }
  }

  mat D = get_D_xred();

  // Discretizing system
  mat Q, Qd, Ad, B, Bd, P0;

  Q = onlyDisplacement ? get_Q_xred() : get_Q();
  if (nb != 0) {
    Q = join_horiz(join_vert(Q, zeros(nb, nx)), zeros(nx+nb, nb));

    for (size_t i = 0; i < nb; ++i) {
      Q(nx+i, nx+i) = 1;
    }
  }

  // Linear integrating part of system. Only used to calculate discretized Q
  mat Aint = onlyDisplacement ? get_A_integrator_xred() : get_A_integrator();

  van_loan_struct vls = van_loan(Aint, Q, dt);
  Qd = vls.Qd;

  Ad = discretize_A(A, dt);

  B =  onlyDisplacement ? get_B_xred() : get_B_fast();

  if (nb != 0) {
    B = join_vert(B, zeros(nb, nu));
  }

  Bd = discretize_B(A, Ad, B);

  P0 = (onlyDisplacement ? get_P0_xred()*1e-6 : get_P0())*1e-8;

  if (nb != 0) {
    P0 = join_horiz(join_vert(P0, zeros(nb, nx)), zeros(nx+nb, nb));

    for (size_t i = 0; i < nb; ++i) {
      P0(nx+i, nx+i) = 1;
    }
    xLp = join_vert(xLp, zeros(nb,1));
  }



  mat R = get_R();

  FilterParams params {xLp, P0, Ad, Bd, H, D, Qd, R, dt};

  // Initialize filter
  filterPtr observer = createObserver(filterVariant, nx, nb, nu, nz, useSRformulation, RK4Iterations, updateJacobians, updateQ, cubature, normalized);

  observer->init(params);

  return observer;
}


// Stores estimates in provided stateEstimates array
void runObserver(const double input[NUMBER_INPUTS], const double meas[NUMBER_MEASUREMENTS],
    double stateEstimates[NUMBER_STATES], KalmanFilter &observer) {

  vec u(input, NUMBER_INPUTS);

  vec z(NUMBER_MEASUREMENTS);
  for (size_t i = 0; i < NUMBER_MEASUREMENTS; i += NUMBER_MEASUREMENTS_PER_SENSOR) {
    if (!testing) {
      // reading is in mT, but h(x) uses T
      z(0+i) = meas[0+i]*1e-3;   // bx
      z(1+i) = meas[1+i]*1e-3;   // by
      z(2+i) = -meas[2+i]*1e-3;  // bz; is inverted
    } else {
      z(0) = meas[0+i];
      z(1) = meas[1+i];
      z(2) = meas[2+i];
    }
  }

  observer.predict(u);
  vec u_meas = zeros(NUMBER_INPUTS, 1); // Assuming throughput is compensated for
  observer.update(z, u);
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

