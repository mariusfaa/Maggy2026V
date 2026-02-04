
#include "../BasicLinearAlgebra/BasicLinearAlgebra.h"
#include "../BasicLinearAlgebra/ElementStorage.h"

#include "observer.h"
#include "kalmanFilter.h"
#include "observerDefinitions.h"
#include "discretizer.h" // for increase_stateSpace

using namespace BLA;

// alisases
using A_type = BLA::Matrix<NUMBER_STATES_REDUCED, NUMBER_STATES_REDUCED, double>;
using B_type = BLA::Matrix<NUMBER_STATES_REDUCED, NUMBER_INPUTS, double>;
using H_type = BLA::Matrix<NUMBER_MEASUREMENTS, NUMBER_STATES_REDUCED, double>;
using Q_type = A_type;
using x_type = BLA::Matrix<NUMBER_STATES_REDUCED, 1, double>;


volatile bool newSensorReading = false;
BLA::Matrix<3, 1, double> innovation_means = {0,0,0};
double stateEstimates[12];
KalmanFilter<NUMBER_STATES_REDUCED, NUMBER_INPUTS, NUMBER_MEASUREMENTS> KF;


void newSensorReading_callback() {
  newSensorReading = true;
}

// Easier concatenation
template <typename TopType, typename BottomType>
auto vconcat(const TopType& top, const BottomType& bottom) {
    return BLA::VerticalConcat<TopType, BottomType>(top, bottom);
}

template <typename LeftType, typename RightType>
auto hconcat(const LeftType& top, const RightType& bottom) {
    return BLA::HorizontalConcat<LeftType, RightType>(top, bottom);
}

template <int nx, int nw>
auto van_loan(const BLA::Matrix<nx, nx, double>& A, const BLA::Matrix<nx, nw, double>& G, const BLA::Matrix<nx, nx, double>& Q_c, const double dt) {
    return Q_c;
}


// Run once at start of program
void initObserver() {
  double dt = 0.0005; // 500 µs

  const int observerType = 0; //  0 KF 1 EKF 2 UKF

  const int nx = NUMBER_STATES_REDUCED;
  const int nu = NUMBER_INPUTS;
  const int nz = NUMBER_MEASUREMENTS;

  /*
  const int nb = NUMBER_BIASED_MEASUREMENTS;
  const int nxb = nx+nb;  // combined states and bias-states

  BLA::Zeros<nb, nx, double> A_bottompad;
  BLA::Zeros<nxb, nb, double> A_rightpad;
  BLA::Zeros<nb, nu, double> B_bottompad;
  BLA::Matrix<nz, nb, double> H_rightpad = {
    1,0,0,
    0,1,0,
    0,0,1
  };
  
  BLA::Matrix<nb, nb, double> Q_rightpad = {
    1,0,0,
    0,1,0,
    0,0,1
  };
  */

  A_type A_c_filament = {
    0,0,0,0,0,1,0,0,0,0,
    0,0,0,0,0,0,1,0,0,0,
    0,0,0,0,0,0,0,1,0,0,
    0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,1,
    2022.7425,-0,0,-0,49.15137,0,0,0,0,0,
    0,2022.7425,0,-49.15137,-0,0,0,0,0,0,
    0,0,-4045.48503,-0,-0,0,0,0,0,0,
    0.00011,-17376638.85028,8e-05,-1366851.06201,9e-05,0,0,0,0,0,
    17376638.8503,2e-05,2e-05,-5e-05,-1366851.06198,0,0,0,0,
  };

  B_type B_c_filament = {
    0,0,0,0,
    0,0,0,0,
    0,0,0,0,
    0,0,0,0,
    0,0,0,0,
    2.41669,0,-2.41669,0,
    0,2.41669,-0,-2.41669,
    -2.33118,-2.33118,-2.33118,-2.33118,
    -0,5760.68861,-0,-5760.68861,
    -5760.68861,0,5760.68861,0
  };

  H_type H_filament = {
    0.32779,0,-0.00641,0,0.00111,0,0,0,0,0,
    0,0.32781,0,-0.00111,0,0,0,0,0,0,
    -0.00641,0,-0.6556,0,-0.0001,0,0,0,0,0
  };

  // covariance matrices
  BLA::Eye<nx, nx, double> P0;
  Q_type Q_c = {
    1,0,0,0,0,0,0,0,0,0,
    0,1,0,0,0,0,0,0,0,0,
    0,0,1,0,0,0,0,0,0,0,
    0,0,0,1,0,0,0,0,0,0,
    0,0,0,0,1,0,0,0,0,0,
    0,0,0,0,0,1,0,0,0,0,
    0,0,0,0,0,0,1,0,0,0,
    0,0,0,0,0,0,0,1,0,0,
    0,0,0,0,0,0,0,0,1,0,
    0,0,0,0,0,0,0,0,0,1
  };

  BLA::Matrix<nz, nz, double> R = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
  };

  // discretization
  BLA::Eye<nx, nx, double> I;
  BLA::Matrix<nx, nx, double> A_d;
  BLA::Matrix<nx, nu, double> B_d;
  BLA::Matrix<nx, nx, double> Q_d;

  A_d = I + A_c_filament*dt;
  B_d = B_c_filament*dt;
  Q_d = Q_c;

  /*
  if (nb != 0) {
      // concatenate to make use of bias states
      BLA::Matrix<nxb, nxb, double> A_b = hconcat(vconcat(A_c_filament, A_bottompad), A_rightpad);
      BLA::Matrix<nxb, nu, double> B_b = vconcat(B_c_filament, B_bottompad);
      BLA::Matrix<nz, nxb, double> H_b = hconcat(H_filament, H_rightpad);
      BLA::Matrix<nxb, nxb, double> Q_b = hconcat(vconcat(Q_c, A_bottompad), Q_rightpad);

      // discretization with bias states
      BLA::Eye<nxb, nxb, double> I;
      BLA::Matrix<nxb, nxb, double> A_d = I + A_b*dt;
      BLA::Matrix<nxb, nu, double> B_d = B_b*dt;
      BLA::Matrix<nxb, nxb, double> Q_d = Q_b;
  }
  */
  
  // initialize state
  x_type x0; x0.Fill(0);

  // use nonlinear model for EKF and UKF
  if (observerType != 0) maglevModel_initialize();

  KF.init(x0, P0, A_d, B_d, Q_d, H_filament, R, dt);
}


// Will save states in extern stateEstimates[12]
void runObserver(const float pwmInputX, const float pwmInputY, const float (*sensorReading)[3]) {
  BLA::Matrix<3, 1, double> meas;
  BLA::Matrix<4, 1, double> u;

  newSensorReading = false;

  static int measCount = 0;
  measCount++;

  for (int i = 0; i < 3; ++i) {
    meas(i) = sensorReading[0][i]*0.001; // reading is in mT, but h(x) uses T
  }

  u(0) = pwmInputX;
  u(1) = pwmInputY;
  u(2) = -pwmInputX;
  u(3) = -pwmInputY;

  KF.predict(u, false);
  KF.update(meas, false);

  // Innovation mean
  innovation_means = (innovation_means + meas - KF.getMeasPred())/(double)measCount;


  // Add back rotation around z axis
  increase_stateSpace(KF.getState(), stateEstimates);
}
