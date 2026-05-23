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

filterPtr createObserver(int filterVariant, size_t nx, size_t nu, size_t nz, bool useSRformulation, int RK4Iterations, bool updateJacobians, bool updateQ, bool cubature) {
  switch (filterVariant) {
    case 0:
      return std::make_unique<KalmanFilter>(nx, nu, nz, useSRformulation);

    case 1:
      return std::make_unique<ExtendedKalmanFilter>(nx, nu, nz, useSRformulation, RK4Iterations, updateJacobians, updateQ);

    case 2:
      return std::make_unique<UnscentedKalmanFilter>(nx, nu, nz, useSRformulation, RK4Iterations, cubature);

    default:
      throw std::invalid_argument("Invalid filter variant");
  }
}


// filter: 0 KF, 1 EKF, 2 UKF
filterPtr initObserver(int filterVariant, double dt, size_t nx, double* x0ptr, double* Rptr, double* Qptr, double* P0ptr, bool useSRformulation, int RK4Iterations, bool updateJacobians, bool updateQ, bool cubature) {

  size_t nu = NUMBER_INPUTS;
  size_t nz = NUMBER_MEASUREMENTS;

  bool onlyDisplacement = nx == NUMBER_STATES_REDUCED_EXTRA;

  // x0 acts as linearization point
  vec xLp(x0ptr, nx);
  vec uLp(NUMBER_INPUTS, arma::fill::zeros);

  // Function values around equilibrium
  vec dx(nx, arma::fill::zeros);
  vec meas(nz, arma::fill::zeros);
  dynamics_f(xLp, uLp, dx);
  measurements_h(xLp, uLp, meas);

  // Linearizing system
  mat A = calculateJacobian(xLp, uLp, 0, dx, dt, 2);
  mat H = calculateJacobian(xLp, uLp, 1, meas, dt, 2);

  mat D = get_D();

  // Discretizing system
  mat Ad = discretize_A(A, dt);

  mat B =  onlyDisplacement ? get_B_xred() : get_B();

  mat Bd = discretize_B(A, Ad, B);

  // Noise
  mat R(Rptr, nz, nz);
  mat Qd(Qptr, nx, nx);
  mat P0(P0ptr, nx, nx);

  FilterParams params {xLp, P0, Ad, Bd, H, D, Qd, R, dt};

  // Initialize filter
  filterPtr observer = createObserver(filterVariant, nx, nu, nz, useSRformulation, RK4Iterations, updateJacobians, updateQ, cubature);

  observer->init(params);

  return observer;
}


// Stores estimates in provided stateEstimates array. stateEstimates has to be of same length as number of observed states
void runObserver(const double input[NUMBER_INPUTS], const double meas[NUMBER_MEASUREMENTS],
    double *stateEstimates, KalmanFilter &observer) {

  vec u(input, NUMBER_INPUTS);

  vec z(NUMBER_MEASUREMENTS);
  for (size_t i = 0; i < NUMBER_MEASUREMENTS; i += NUMBER_MEASUREMENTS_PER_SENSOR) {
    // reading is in mT, but h(x) uses T
    z(0+i) = meas[0+i]*1e-3;   // bx
    z(1+i) = meas[1+i]*1e-3;   // by
    z(2+i) = -meas[2+i]*1e-3;  // bz; is inverted
  }

  observer.predict(u);
  observer.update(z, u);
  vec estimate = observer.getState();

  // Add back states that are observed
  size_t nx = observer.getNstates();
  for (size_t i = 0; i < nx; ++i) {
    stateEstimates[i] = estimate(i);
  }
}

