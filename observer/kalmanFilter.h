#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "../BasicLinearAlgebra/BasicLinearAlgebra.h"
#include "utilities.h"


class KalmanFilter {
protected:
    // State vector: n x 1
    StateVector x;

    // Predicted measurement p x 1
    MeasVector z_pred;
    
    // State covariance: n x n
    Q_type P;
    
    // State transition: n x n
    A_type F;
    
    // Input matrix: m x n
    B_type B;
    
    // Process noise covariance: n x n
    Q_type Q;
    
    // Measurement matrix: p x n
    H_type H;
    
    // Measurement noise covariance: p x p
    R_type R;
    
    // Kalman gain: n x p
    K_type K;
    
    // Identity matrix: n x n
    A_type I;

    // Discretization time
    double dt;
    
public:
    // Constructor
    KalmanFilter() {
        // Initialize identity matrix
        for(int i = 0; i < NUMBER_STATES_REDUCED; i++) {
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
    void init(StateVector& initialState,
              Q_type& initialCovariance,
              A_type& stateTransition,
              B_type& inputMatrix,
              Q_type& processNoise,
              H_type& measurementMatrix,
              R_type& measurementNoise,
              double& dt){
        x = initialState;
        P = initialCovariance;
        F = stateTransition;
        B = inputMatrix;
        Q = processNoise;
        H = measurementMatrix;
        R = measurementNoise;
    }
    
    // Prediction step
    virtual void predict(InputVector& u) {
        // x = F * x + B * u
        x = F * x + B * u;
        
        // P = F * P * F^T + Q
        P = F * P * ~F + Q;
    }
    
    // Update step
    virtual void update(MeasVector& z) {
        z_pred = H * x;
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
    
    // Get current state
    StateVector getState() const {
        return x;
    }

    // Get current predicted value
    MeasVector getMeasPred() const {
        return z_pred;
    }
    
    // Get current covariance
    Q_type getCovariance() const {
        return P;
    }
};


#endif // KALMAN_FILTER_H
