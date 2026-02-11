
//#include "extendedKalmanFilter.h"
#include "kalmanFilter.h"
#include "matrices.h"
#include "utilities.h"

volatile bool newSensorReading = false;
volatile unsigned long observerTime = 0;
MeasVector innovation_means = {0,0,0};
double stateEstimates[NUMBER_STATES];

KalmanFilter KF;
//ExtendedKalmanFilter EKF;

template<typename FilterType>
StateVector runFilter(FilterType &filter, InputVector &u, MeasVector &z) {
  filter.predict(u);
  filter.update(z);
  return filter.getState();
}


// Run once at start of program
void initObserver() {
  double dt = 0.0005; // 500 µs

  // Initialize state
  StateVector x0; x0.Fill(0);

  // Initialize filter
  KF.init(x0, P0, A_d_fast, B_d_fast, Q_d_fast, H_fast, R, dt);
}

// Will save states in extern stateEstimates[12]
void runObserver(const float ux, const float uy, const float (*z)[NUMBER_MEASUREMENTS]) {
  unsigned long start = micros();
  MeasVector meas;
  InputVector u;

  static int measCount = 0;
  ++measCount;

  for (int i = 0; i < 3; ++i) {
    meas(i) = z[0][i]*1e-3; // reading is in mT, but h(x) uses T
  }

  u(0) =  ux;
  u(1) =  uy;
  u(2) = -ux;
  u(3) = -uy;


  // Innovation mean
  //innovation_means = (innovation_means + meas - KF.getMeasPred())/(double)measCount;
  
  StateVector estimate = runFilter(KF, u, meas);

  // Add back rotation around z axis as 0
  increaseStateSpace(estimate, stateEstimates);
  unsigned long stop = micros();

  observerTime = stop - start; // µs
}

