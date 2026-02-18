
//#include "extendedKalmanFilter.h"
#include "kalmanFilter.h"
#include "matrices.h"
#include "utilities.h"
#include <chrono>

using namespace std::chrono;

volatile bool newSensorReading = false;
volatile unsigned long observerTime = 0;
double NIS = 0;

//ExtendedKalmanFilter EKF;

KalmanFilter KF(NUMBER_STATES_REDUCED, NUMBER_INPUTS, NUMBER_MEASUREMENTS);

// Filter independent way of running an entire estimating cycle
template<typename FilterType>
vec runFilter(FilterType &filter, vec u, vec z) {
  filter.predict(u);
  filter.update(z);
  NIS = filter.nis;
  return filter.getState();
}


// Call once at start
void initObserver() {
  const double dt = 0.0005; // 500 µs

  // Initial state
  vec x0(NUMBER_STATES_REDUCED, fill::zeros);

  // Discretizing system
  mat Ad = discretize_A(get_A_fast(), dt);
  mat Bd = discretize_B(get_A_fast(), Ad, get_B_fast());
  mat Qd = van_loan(get_A_fast(), get_Q(), dt);

  // Initialize filter
  KF.init(x0, get_P0(), Ad, Bd, Qd, get_H(), get_R(), dt);
}


// Stores estimates in provided stateEstimates array
void runObserver(const float input[NUMBER_INPUTS], const float (*z)[NUMBER_MEASUREMENTS],
    double *stateEstimates) {
  //unsigned long start = micro;
  vec u(NUMBER_INPUTS);  // ux, uy, -ux, -uy
  u(0) = input[0];
  u(1) = input[1];
  u(2) = input[2];
  u(3) = input[3];
  vec meas(NUMBER_MEASUREMENTS);

  static int measCount = 0;
  ++measCount;

  for (int i = 0; i < 3; ++i) {
    meas(i) = z[0][i]*1e-3; // reading is in mT, but h(x) uses T
  }

  vec estimate = runFilter(KF, u, meas);

  // Add back rotation around z axis as 0
  increaseStateSpace(&estimate, stateEstimates);
  //unsigned long stop = micros();

  //observerTime = stop - start; // µs
}

