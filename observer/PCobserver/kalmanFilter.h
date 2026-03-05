#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <armadillo>
#include <cstddef>
#include "utilities.h"

using namespace arma;

class KalmanFilter {
private:
    bool useSRformulation;

protected:
    double dt; // Discretization time

    size_t nx; // Number of states
    size_t nu; // Number of inputs
    size_t nz; // Number of measurements

    vec x_est;  // Estimated states
    vec x_pred; // Predicted states
    vec z_pred; // Predicted measurements

    mat F; // State transition matrix
    mat B; // Input matrix
    mat H; // Measurement matrix

    mat P; // Prediction/estimate covariance
    mat Q; // Process covariance
    mat R; // Measurement covariance
    mat S; // Innovation covariance

    // Square roots of covariances
    mat Ps;
    mat Qs;
    mat Rs;
    mat Ss;

    mat I; // Identity matrix
    mat W; // Kalman gain

    vec v; // Innovations
    double nis; // Normalized innovations squared

public:
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
        R(arma::eye(nz, nz)),
        S(arma::eye(nz, nz)),

        Ps(arma::eye(nx, nx)),
        Qs(arma::eye(nx, nx)),
        Rs(arma::eye(nz, nz)),
        Ss(arma::eye(nz, nz)),

        I(arma::eye(nx, nx)),
        W(arma::zeros(nx, nz))
        {
        useSRformulation = 1;
        }

    virtual void init(vec &initialState,
              mat &initialCovariance,
              mat &stateTransition,
              mat &inputMatrix,
              mat &processNoise,
              mat &measurementMatrix,
              mat &measurementNoise,
              double &discretizationTime
              ) {
        dt = discretizationTime;
        x_est = initialState;
        P = initialCovariance;
        F = stateTransition;
        B = inputMatrix;
        Q = processNoise;
        H = measurementMatrix;
        R = measurementNoise;

        Ps = chol(P);
        Qs = chol(Q);
        Rs = chol(R);
    }

    virtual void predict(vec &u) {
        // Predict mean
        x_pred = F * x_est + B * u;

        // Predict covariance
        if (useSRformulation) {
            Ps = QRr(join_vert(Ps*F.t(), Qs));
        }
        else {
            P = F * P * F.t() + Q;
        }
    }

    virtual void update(vec &z) {
        // Predicted measurement
        z_pred = H * x_pred;

        // Innovation
        v = z - z_pred;

        // Innovation covariance
        if (useSRformulation) {
            Ss = QRr(join_vert(Ps*H.t(), Rs));
        }
        else {
            S = H * P * H.t() + R;
        }

        // Only use to calculate NIS; use solve otherwise
        // mat Sinv = inv(S, inv_opts::likely_sympd);

        // Calculate Kalman gain
        //W = P * H.t() * Sinv;
        if (useSRformulation) {
          mat _temp = solve(trimatl(Ss.t()), H);
          mat __temp = solve(trimatu(Ss), _temp);
          W = (__temp*Ps.t()*Ps).t();
        }
        else {
            W = solve(S, H*P, solve_opts::likely_sympd).t();
        }
        // calculateNIS(v, Sinv);

        // Update state estimate
        x_est = x_pred + W * v;

        // Update covariance estimate
        if (useSRformulation) {
            Ps = QRr(join_vert(Ps*(I-W*H).t(), Rs*W.t()));
        }
        else {
            // P = (I - W * H) * P;
            P = (I - W*H) * P * (I - W*H).t() + W*R*W.t();
        }
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
