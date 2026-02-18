#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <armadillo>
#include "utilities.h"

using namespace arma;

class KalmanFilter {
public:
    int n;
    int m;
    int p;
    vec x_est;
    vec x_pred;
    vec z_pred;
    mat P;
    mat F;
    mat B;
    mat Q;
    mat H;
    mat R;
    mat S;
    mat K;
    double dt;

    // Innovations
    vec vi;

    // Normalized innovations squared
    double nis;

    // Constructor
    KalmanFilter(int numberStates, int numberInputs, int numberMeasurements) {
        n = numberStates;
        m = numberInputs;
        p = numberMeasurements;
    };

    // Initializing
    void init(vec initialState,
              mat initialCovariance,
              mat stateTransition,
              mat inputMatrix,
              mat processNoise,
              mat measurementMatrix,
              mat measurementNoise,
              double discretizationTime) {
        x_est = initialState;
        P = initialCovariance;
        F = stateTransition;
        B = inputMatrix;
        Q = processNoise;
        H = measurementMatrix;
        R = measurementNoise;
        dt = discretizationTime;
    }

    // Prediction step
    virtual void predict(vec &u) {
        // x = F * x + B * u
        x_pred = F * x_est + B * u;

        // P = F * P * F^T + Q
        P = F * P * F.t() + Q;
    }

    // Update step
    virtual void update(vec &z) {
        // Predicted measurement
        z_pred = H * x_pred;

        // Calculate innovation
        vec v = z - z_pred;

        // Calculate innovation covariance
        S = H * P * H.t() + R;

        // Only use to calculate NIS; use solve otherwise
        mat Sinv = inv(S, inv_opts::likely_sympd);

        // Calculate Kalman gain
        //K = P * H.t() * Sinv;
        // S K^T = (P H^T)^T
        K = solve(S, H*P, solve_opts::likely_sympd).t();

        // Calculate NIS
        calculateNIS(v, Sinv);

        // Update state estimate
        x_est = x_pred + K * v;

        // Update covariance estimate
        P = (eye(n,n) - K * H) * P;
    }

    // Get state estimate
    vec getState() const {
        return x_est;
    }

    // Get predicted measurement
    vec getMeasPred() const {
        return z_pred;
    }

    // Get covariance
    mat getCovariance() const {
        return P;
    }

    // Calculate normalised innovations squared
    void calculateNIS(vec &v, mat &Sinv) {
        //vi = Sinv*v;
        double nis = as_scalar(v.t()*Sinv*v);
    }
};


#endif // KALMAN_FILTER_H
