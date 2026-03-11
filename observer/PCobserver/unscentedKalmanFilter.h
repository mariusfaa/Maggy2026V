#ifndef UNSCENTED_KALMAN_FILTER_H
#define UNSCENTED_KALMAN_FILTER_H

#include <armadillo>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>
#include "include/matlab/maglevModel.h"
#include "integrator.h"
#include "utilities.h"
#include "kalmanFilter.h"
#include "extendedKalmanFilter.h"

using namespace arma;

class UnscentedKalmanFilter: public ExtendedKalmanFilter {
    using Base = ExtendedKalmanFilter;

private:
    bool useSRformulation;

protected:
    using Base::dt;

    using Base::nx;
    using Base::nu;
    using Base::nz;

    using Base::x_est;
    using Base::x_pred;
    using Base::z_pred;

    using Base::P;
    using Base::Q;
    using Base::R;
    using Base::S;

    using Base::Ps;
    using Base::Qs;
    using Base::Rs;
    using Base::Ss;

    using Base::I;
    using Base::W;

    double alpha;
    double beta;
    double kappa;
    double lambda;
    double eta;

    size_t ns;             // Number of sigma points
    mat sigma_points;
    mat sigma_points_pred; // Predicted state of sigma points
    mat sigma_points_meas; // Predicted measurement of sigma points

    vec weights_mean;
    vec weights_cov;

    // Square root of weights
    vec weights_cov_sr;

public:
    UnscentedKalmanFilter(size_t numberStates, size_t numberInputs, size_t numberMeasurements):
    Base(numberStates, numberInputs, numberMeasurements),
    ns(2*nx+1),
    sigma_points(arma::zeros(nx, ns)),
    sigma_points_pred(arma::zeros(nx, ns)),
    sigma_points_meas(arma::zeros(nz, ns)),
    weights_mean(arma::zeros(ns)),
    weights_cov(arma::zeros(ns)),
    weights_cov_sr(arma::zeros(ns)),
    useSRformulation(1)
    {
    setParameters();
    }

    virtual void init(vec &initialState,
            mat &initialCovariance,
            mat &processNoise,
            mat &measurementNoise,
            double &discretizationTime
            ) {
        dt = discretizationTime;
        x_est = initialState;
        P = initialCovariance;
        Q = processNoise;
        R = measurementNoise;

        Ps = chol(P);
        Qs = chol(Q);
        Rs = chol(R);
    }

    void setParameters(const double &alpha_=1e-3, const double &beta_=2.0, const double &kappa_=0) {
        alpha = alpha_;
        beta = beta_;
        kappa = kappa_;
        lambda = pow(alpha, 2)/(nx + kappa) - nx;
        eta = sqrt(nx + lambda);

        // Weights for each sigma point. Normalizes mean and covariance
        weights_mean(0) = lambda/(nx + lambda);
        weights_cov(0) = lambda/(nx + lambda) + 1.0 - pow(alpha, 2) + beta;

        if (useSRformulation) {
            // 0th weight might be negative; keep sign and sqrt only absolute value to avoid imaginary number
            double _sign = 1.0;
            if (weights_cov(0) < 0) {
                _sign = -1.0;
            }

            weights_cov_sr(0) = _sign*sqrt(fabs(weights_cov(0)));
        }

        for (size_t i = 1; i < ns; ++i) {
            weights_mean(i) = weights_cov(i) = 0.5/(nx + lambda);
            if (useSRformulation) {
                weights_cov_sr(i) = sqrt(weights_mean(i));
            }
        }
    }

    void predict(vec &u) override {

        if (!useSRformulation) {
            Ps = chol(P);
        }

        // Calculate sigma points
        sigma_points.col(0) = x_est;
        for (size_t i = 1; i < nx + 1; ++i) {
            sigma_points.col(i) = x_est + eta*Ps.col(i-1);
            sigma_points.col(i + nx) = x_est - eta*Ps.col(i-1);
        }

        // Predict transformed state of sigma points
        for (size_t i = 0; i < ns; ++i) {
            eulerForward(sigma_points.col(i), u, dt, dxd);
            sigma_points_pred.col(i) = x_pred;
        }

        // Calculate mean of predicted sigma points
        x_pred = arma::zeros(nx);
        for (size_t i = 0; i < ns; ++i) {
            x_pred += weights_mean(i) * sigma_points_pred.col(i);
        }

        // Calculate covariance of predicted sigma points
        if (useSRformulation) {
            mat _left = arma::zeros(nx, ns-1);
            for (size_t i = 1; i < ns; ++i) {
                _left.col(i-1) = weights_cov_sr(i) * (sigma_points_pred.col(i) - x_pred);
            }
            Ps = QRr(join_horiz(_left, Qs).t());
            Ps = cholUpdate(Ps.t(), sigma_points_pred.col(0) - x_pred, weights_cov_sr(0)).t();
        }
        if(1) {
            P = Q;
            for (size_t i = 1; i < ns; ++i) {
                vec _e = sigma_points_pred.col(i) - x_pred;
                P += weights_cov(i) * _e * _e.t();
            }

            // Averaging for symmetry. Small regularization for positive definiteness
            P = (P + P.t())*0.5 + eye(nx, nx)*1e-12;
            if (!P.is_sympd()) {
                std::cout << "P is not symmetric positive definite!" << endl;
            }
        }
    }

    void update(vec &z) override {
        // Assuming feedthrough is compensated for
        vec u = zeros(nu);

        // Predicted measurement for each predicted sigma point
        for (size_t i = 0; i < ns; ++i) {
            double x_pad[NUMBER_STATES] = {};

            increaseStateSpace(sigma_points_pred.col(i), x_pad);
            maglevSystemMeasurements_fast(x_pad, u.memptr(), z_pred.memptr());
            sigma_points_meas.col(i) = z_pred;
        }

        // Calculate mean of predicted measurements
        z_pred = arma::zeros(nz);
        for (size_t i = 0; i < ns; ++i) {
            z_pred += weights_mean(i) * sigma_points_meas.col(i);
        }

        // Innovation
        v = z - z_pred;

        // Calculate covariance of predicted measurements
        if (useSRformulation) {
            mat _left = arma::zeros(nz, ns-1);
            for (size_t i = 1; i < ns; ++i) {
                _left.col(i-1) = weights_cov_sr(i) * (sigma_points_meas.col(i) - z_pred);
            }
            Ss = QRr(join_horiz(_left, Rs).t());
            Ss = cholUpdate(Ss.t(), sigma_points_meas.col(0) - z_pred, weights_cov_sr(0)).t();
        }
        if(1) {
            S = R;
            for (size_t i = 0; i < ns; ++i) {
                vec _e = sigma_points_meas.col(i) - z_pred;
                S += weights_cov(i) * _e * _e.t();
            }
            // Averaging for symmetry. Small regularization for positive definiteness
            // S = (S + S.t())*0.5 + eye(nz, nz)*1e-12;
            if (!S.is_sympd()) {
                std::cout << "S is not symmetric positive definite!" << endl;
            }
        }

        // Calculate cross-covariance
        mat Pxz = arma::zeros(nx, nz);
        for (size_t i = 0; i < ns; ++i) {
            vec _e = sigma_points_pred.col(i) - x_pred;
            vec _ee = sigma_points_meas.col(i) - z_pred;
            Pxz += weights_cov(i) * _e * _ee.t();
        }

        if (useSRformulation) {
            // W = solve(Ss.t()*Ss, Pxz.t(), solve_opts::likely_sympd).t();
            W = solve(trimatu(Ss), solve(trimatl(Ss.t()), Pxz.t())).t();
        }
        else {
            // Only use to calculate NIS; use solve otherwise
            // mat Sinv = inv(S, inv_opts::likely_sympd);

            // Calculate Kalman gain
            //W = Pxz * Sinv;
            W = solve(S, Pxz.t(), solve_opts::likely_sympd).t();
        }

        // Update state estimate
        x_est = x_pred + W * v;

        if (useSRformulation) {
            Ps = cholUpdate(Ps.t(), W*Ss, -1.0).t();
        }
        if(1) {
            // Update covariance estimate
            P = P - W * S * W.t();
        }
        std::cout << P << endl;
        std::cout << Ps.t()*Ps << endl;
        std::cout << S << endl;
        std::cout << Ss.t()*Ss << endl;
    }
};

#endif // UNSCENTED_KALMAN_FILTER_H
