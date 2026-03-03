#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <armadillo>
#include <cstddef>
#include "utilities.h"

using namespace arma;

class KalmanFilter {
protected:
    double dt;

    size_t nx;
    size_t nu;
    size_t nz;

    vec x_est;
    vec x_pred;
    vec z_pred;

    mat F;
    mat B;
    mat H;

    mat P;
    mat Q;
    mat R;
    mat S;

    mat I;
    mat W;

    vec v; // Innovations
    double nis; // Normalized innovations squared

public:
    // Constructor
    KalmanFilter(size_t numberStates, size_t numberInputs, size_t numberMeasurements):
        nx(numberStates),
        nu(numberInputs),
        nz(numberMeasurements),

        x_est(arma::zeros(nx)),
        x_pred(arma::zeros(nx)),
        z_pred(arma::zeros(nz)),

        F(arma::zeros(nx, nx)),
        B(arma::zeros(nx, nu)),
        H(arma::zeros(nz, nx)),

        P(arma::eye(nx, nx)),
        Q(arma::eye(nx, nx)),
        R(arma::eye(nx, nx)),
        S(arma::eye(nx, nx)),

        I(arma::eye(nx,nx)),
        W(arma::zeros(nx, nz))
        {}

    // Initializing
    virtual void init(vec initialState,
              mat initialCovariance,
              mat stateTransition,
              mat inputMatrix,
              mat processNoise,
              mat measurementMatrix,
              mat measurementNoise,
              double discretizationTime
              ) {
        dt = discretizationTime;
        x_est = initialState;
        P = initialCovariance;
        F = stateTransition;
        B = inputMatrix;
        Q = processNoise;
        H = measurementMatrix;
        R = measurementNoise;
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
        v = z - z_pred;

        // Calculate innovation covariance
        S = H * P * H.t() + R;

        // Only use to calculate NIS; use solve otherwise
        // mat Sinv = inv(S, inv_opts::likely_sympd);

        // Calculate Kalman gain
        //W = P * H.t() * Sinv;
        // S W^T = (P H^T)^T
        W = solve(S, H*P, solve_opts::likely_sympd).t();

        // calculateNIS(v, Sinv);

        // Update state estimate
        x_est = x_pred + W * v;

        // Update covariance estimate
        P = (I - W * H) * P;
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

    // Get system matrix
    mat getF() const {
        return F;
    }

    // Get input matrix
    mat getB() const {
        return B;
    }

    // Calculate normalised innovations squared
    void calculateNIS(vec &v, mat &Sinv) {
        //vi = Sinv*v;
        double nis = as_scalar(v.t()*Sinv*v);
    }

    double getNIS() const {
        return nis;
    }
};


#endif // KALMAN_FILTER_H
