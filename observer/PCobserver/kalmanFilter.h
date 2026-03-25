#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <armadillo>
#include <cstddef>
#include "utilities.h"

using namespace arma;

class KalmanFilter {
protected:
    bool useSRformulation;
    bool useNIS;

    double dt; // Discretization time

    size_t nx; // Number of states
    size_t nu; // Number of inputs
    size_t nz; // Number of measurements

    vec x_est;  // Estimated states
    vec x_pred; // Predicted states
    vec z_pred; // Predicted measurements
    vec innovation; // Innovation

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

public:
    // Destructor
    ~KalmanFilter() = default;

    // Constructor
    KalmanFilter(size_t numberStates, size_t numberInputs, size_t numberMeasurements, bool useSRformulation):
        nx(numberStates),
        nu(numberInputs),
        nz(numberMeasurements),

        x_est(arma::zeros(nx)),
        x_pred(arma::zeros(nx)),
        z_pred(arma::zeros(nz)),
        innovation(arma::zeros(nz)),

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
        W(arma::zeros(nx, nz)),
        useSRformulation(useSRformulation)
        {}

    virtual void init(const FilterParams &params) {
        x_est = params.x0; // Initial state
        P = params.P0;     // Initial covariance
        F = params.Ad;     // State transition matrix
        B = params.Bd;     // Input matrix
        Q = params.Qd;     // Process noise covariance
        H = params.H;      // Measurement matrix
        R = params.R;      // Measurement noise covariance
        dt = params.dt;    // Discretization time

        Ps = chol(P);
        Qs = chol(Q);
        Rs = chol(R);
    }

    virtual void predict(vec &u) {
        // Predict mean
        x_pred = F * x_est + B * u;

        // Predict covariance
        if (useSRformulation) {
            mat _Q;
            qr_econ(_Q, Ps, join_vert(Ps*F.t(), Qs));
        }
        else {
            P = F * P * F.t() + Q;
            P = (P + P.t())*0.5 + eye(nx, nx)*1e-9;
            if (!P.is_sympd(1e-9)) {
                std::cout << "P is not symmetric positive definite!" << endl;
            }
        }
    }

    virtual void update(vec &z) {
        // Predicted measurement
        z_pred = H * x_pred;

        // Innovation
        innovation = z - z_pred;

        // Innovation covariance
        if (useSRformulation) {
            mat _Q;
            qr_econ(_Q, Ss, join_vert(Ps*H.t(), Rs));
 
        }
        else {
            S = H * P * H.t() + R;
        }

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

        // Update state estimate
        x_est = x_pred + W * innovation;

        // Update covariance estimate
        if (useSRformulation) {
            mat _Q;
            qr_econ(_Q, Ps, join_vert(Ps*(I-W*H).t(), Rs*W.t()));
        }
        else {
            // P = (I - W * H) * P;
            P = (I - W*H) * P * (I - W*H).t() + W*R*W.t();
            if (!P.is_sympd(1e-9)) {
                std::cout << "P is not symmetric positive definite!" << endl;
            }
        }
    }

    // Get state estimate
    vec getState() const {
        return x_est;
    }

    // Get predicted state
    vec getStatePred() const {
        return x_pred;
    }

    // Get predicted measurement
    vec getMeasPred() const {
        return z_pred;
    }

    // Get covariance
    mat getCovariance() const {
        if (useSRformulation) {
            if (Ps.is_trimatu()) {
                return Ps.t()*Ps;
            } else if(Ss.is_trimatl()) {
                return Ps*Ps.t();
            } else {
                std::cout << "Covariance square root is not triangular!" << endl;
                return eye(nx, nx);
            }
        } else {
            return P;
        }
    }

    // Get square root covariance
    mat getSRCovariance() const {
        return Ps;
    }

    // Get innovation covariance
    mat getInnovationCovariance() const {
        if (useSRformulation) {
            if (Ss.is_trimatu()) {
                return Ss.t()*Ss;
            } else if(Ss.is_trimatl()) {
                return Ss*Ss.t();
            } else {
                std::cout << "Innovation covariance square root is not triangular!" << endl;
                return eye(nz, nz);
            }
        } else {
            return S;
        }
    }

    // Get square root covariance
    mat getSRInnovationCovariance() const {
        return Ps;
    }

    // Get system matrix
    mat getF() const {
        return F;
    }

    // Get measurement matrix
    mat getH() const {
        return H;
    }

    // Get input matrix
    mat getB() const {
        return B;
    }

    // Get Kalman gain
    mat getKalmanGain() const {
        return W;
    }

    // Get Innovation
    vec getInnovation() const {
        return innovation;
    }
};


#endif // KALMAN_FILTER_H
