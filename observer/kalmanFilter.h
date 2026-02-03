#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "../BasicLinearAlgebra/BasicLinearAlgebra.h"
#include "maglevModel.h"
#include "observerDefinitions.h"
#include "discretizer.h"

using namespace BLA;

template<int n, int m, int p>
class KalmanFilter {
private:
    // State vector: n x 1
    BLA::Matrix<n, 1, double> x;

    // Predicted measurement p x 1
    BLA::Matrix<p, 1, double> z_pred;
    
    // State covariance: n x n
    BLA::Matrix<n, n, double> P;
    
    // State transition: n x n
    BLA::Matrix<n, n, double> F;
    
    // Input matrix: m x n
    BLA::Matrix<n, m, double> B;
    
    // Process noise covariance: n x n
    BLA::Matrix<n, n, double> Q;
    
    // Measurement matrix: p x n
    BLA::Matrix<p, n, double> H;
    
    // Measurement noise covariance: p x p
    BLA::Matrix<p, p, double> R;
    
    // Kalman gain: n x p
    BLA::Matrix<n, p, double> K;
    
    // Identity matrix: n x n
    BLA::Matrix<n, n, double> I;

    double dt;
    
public:
    // Constructor
    KalmanFilter() {
        // Initialize identity matrix
        for(int i = 0; i < n; i++) {
            I(i, i) = 1.0f;
        }
        
        // Initialize all matrices to zero
        x.Fill(0);
        z_pred.Fill(0);
        P.Fill(0);
        F.Fill(0);
        B.Fill(0);
        Q.Fill(0);
        H.Fill(0);
        R.Fill(0);
        K.Fill(0);
    }
    
    // Initialize the filter
    void init(const BLA::Matrix<n, 1, double>& initialState,
              const BLA::Matrix<n, n, double>& initialCovariance,
              const BLA::Matrix<n, n, double>& stateTransition,
              const BLA::Matrix<n, m, double>& inputMatrix,
              const BLA::Matrix<n, n, double>& processNoise,
              const BLA::Matrix<p, n, double>& measurementMatrix,
              const BLA::Matrix<p, p, double>& measurementNoise,
              const double& dt){
        x = initialState;
        P = initialCovariance;
        F = stateTransition;
        B = inputMatrix;
        Q = processNoise;
        H = measurementMatrix;
        R = measurementNoise;
    }
    
    // Prediction step
    template<int c>
    void predict(const BLA::Matrix<c, 1, double>& u, bool useEkf) {
        // x = F * x + B * u
        if (useEkf) {
            //x = RK4step(x, u, dt);
            x = eulerForward(x, u, dt);
        } else {
            x = F * x + B * u;
        }
        
        // P = F * P * F^T + Q
        P = F * P * ~F + Q;
    }
    
    // Update step
    void update(const BLA::Matrix<p, 1, double>& z, bool useEkf) {
        if (useEkf) {
            double x_pad[n];
            increase_stateSpace(x, x_pad);
            maglevSystemMeasurements_fast(x_pad, z.storage, z_pred.storage);
        } else {
            z_pred = H * x;
        }
        // Calculate innovation
        BLA::Matrix<p, 1, double> v = z - z_pred;
        
        // Calculate innovation covariance
        BLA::Matrix<p, p, double> S = H * P * ~H + R;
        
        // Calculate Kalman gain
        K = P * ~H * Inverse(S);
        
        // Update state estimate
        x = x + K * v;
        
        // Update covariance estimate
        P = (I - K * H) * P;
    }
    
    // Get current state
    BLA::Matrix<n, 1, double> getState() const {
        return x;
    }

    // Get current predicted value
    BLA::Matrix<p, 1, double> getMeasPred() const {
        return z_pred;
    }
    
    // Get current covariance
    BLA::Matrix<n, n, double> getCovariance() const {
        return P;
    }
};


#endif // KALMAN_FILTER_H
