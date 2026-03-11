
#include "unscentedKalmanFilter.h"
#include "extendedKalmanFilter.h"
#include "kalmanFilter.h"
#include "matrices.h"
#include "utilities.h"

volatile bool newSensorReading = false;
double NIS = 0;


KalmanFilter KF(NUMBER_STATES_REDUCED, NUMBER_INPUTS, NUMBER_MEASUREMENTS);
ExtendedKalmanFilter EKF(NUMBER_STATES_REDUCED, NUMBER_INPUTS, NUMBER_MEASUREMENTS);
UnscentedKalmanFilter UKF(NUMBER_STATES_REDUCED, NUMBER_INPUTS, NUMBER_MEASUREMENTS);


// Filter independent way of running an entire estimating cycle
template<typename FilterType>
vec runFilter(FilterType &filter, vec &u, vec &z) {
  filter.predict(u);
  filter.update(z);
  NIS = filter.getNIS();
  return filter.getState();
}


// Call once at start
void initObserver() {
  double dt = 0.0005; // 500 µs

  // Initial state
  vec x0(NUMBER_STATES_REDUCED, fill::zeros);

  // Discretizing system
  van_loan_struct vls = van_loan(get_A_fast(), get_Q(), dt);
  mat Ad = vls.Ad;
  mat Qd = vls.Qd;
  mat Bd = discretize_B(get_A_fast(), Ad, get_B_fast());

  mat P0 = get_P0();
  mat H = get_H_fast();
  mat R = get_R();

  // Initialize filter
  // KF.init(x0, P0, Ad, Bd, Qd, H, R, dt);
  // EKF.init(x0, P0, Ad, Bd, Qd, H, R, dt);
  UKF.init(x0, P0, Qd, R, dt);
}


// Stores estimates in provided stateEstimates array
void runObserver(const double input[NUMBER_INPUTS], const double (*z)[NUMBER_MEASUREMENTS],
    double stateEstimates[NUMBER_STATES]) {

  vec u(NUMBER_INPUTS);  // ux, uy, -ux, -uy
  u(0) = input[0];
  u(1) = input[1];
  u(2) = input[2];
  u(3) = input[3];
  vec meas(NUMBER_MEASUREMENTS);

  static int measCount = 0;
  ++measCount;

  // reading is in mT, but h(x) uses T
  meas(0) = z[0][0]*1e-3;   // bx
  meas(1) = z[0][1]*1e-3;   // by
  meas(2) = -z[0][2]*1e-3;  // bz; is inverted

  vec estimate = runFilter(UKF, u, meas);

  // Add back rotation around z axis as 0
  increaseStateSpace(estimate, stateEstimates);
}

