#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include "kalmanFilter.h"
#include "discretizer.h"
#include "utilities.h"
#include "maglevModel.h"

class ExtendedKalmanFilter: public KalmanFilter {
  using Base = KalmanFilter;

protected:
  using Base::dt;
  using Base::x;
  using Base::z_pred;
  using Base::I;
  using Base::F;
  using Base::H;
  using Base::K;
  using Base::P;
  using Base::Q;
  using Base::R;

public:
  // Constructor
  ExtendedKalmanFilter(): Base() {
    maglevModel_initialize();
  }


  void predict(InputVector& u) override {
    // x = f(x,u)
    x = eulerForward(x, u, dt);

    // P = F * P * F^T + Q
    P = F * P * ~F + Q;
  }

  void update(MeasVector& z) override {
    float x_pad[NUMBER_STATES];
    increaseStateSpace(x, x_pad);
    // z_pred = h(x)
    maglevSystemMeasurements_fast(x_pad, z.storage, z_pred.storage);

    // Calculate innovation
    MeasVector v = z - z_pred;

    // Calculate innovation covariance
    R_type S = H * P * ~H + R;

    // Calculate Kalman gain
    K = P * ~H * Inverse(S);

    // Update state estimate
    x = x + K * v;

    // Update covariance estimate
    P = (I - K * H) * P;
  }

};

#endif // EXTENDED_KALMAN_FILTER_H
