#ifndef UNSCENTED_KALMAN_FILTER_H
#define UNSCENTED_KALMAN_FILTER_H

#include <armadillo>
#include <cmath>
#include <cstddef>
#include <cstdlib>
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
    bool cubature;

protected:
    using Base::dxd;

    // 0: euler. Otherwise RK4 with n iterations
    using Base::RK4Iterations;

    using Base::useSRformulation;

    using Base::dt;

    using Base::nx;
    using Base::nu;
    using Base::nz;

    using Base::x_est;
    using Base::x_pred;
    using Base::z_pred;
    using Base::innovation;

    using Base::P;
    using Base::Q;
    using Base::R;
    using Base::S;
    mat Pxz; // Cross-covariance

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

    // Weighed difference between predicted mean and sigma points. Only used in SR formulation
    mat x_pred_errors;
    mat z_pred_errors;

    vec weights_mean;
    vec weights_cov;

    // Square root of covariance weights
    vec weights_cov_sr;

    // Sign of central covariance weight
    double central_cov_sgn;

public:
    UnscentedKalmanFilter(size_t numberStates, size_t numberInputs, size_t numberMeasurements, bool useSRformulation, int RK4Iterations, bool cubature=0):
    cubature(cubature),
    ns(2*nx+static_cast<size_t>(!cubature)),
    sigma_points(arma::zeros(nx, ns)),
    sigma_points_pred(arma::zeros(nx, ns)),
    sigma_points_meas(arma::zeros(nz, ns)),
    weights_mean(arma::zeros(ns)),
    x_pred_errors(arma::zeros(nx, 2*nx)),
    z_pred_errors(arma::zeros(nz, 2*nx)),
    weights_cov(arma::zeros(ns)),
    weights_cov_sr(arma::zeros(ns)),
    Pxz(arma::zeros(nx, nz)),
    Base(numberStates, numberInputs, numberMeasurements, useSRformulation, RK4Iterations) {
        if (cubature) {
            weights_cov = weights_mean = ones(ns)/ns;
            weights_cov_sr = arma::sqrt(weights_cov);
            eta = sqrt(ns/2.0);
        } else {
            setParameters();
        }
    }

    virtual void init(const FilterParams &params) override {
        x_est = params.x0; // Initial state
        P = params.P0;     // Initial covariance
        Q = params.Qd;     // Process noise covariance
        R = params.R;      // Measurement noise covariance
        dt = params.dt;    // Discretization time

        Ps = chol(P, "lower");
        Qs = chol(Q, "lower");
        Rs = chol(R, "lower");
    }

    void setParameters(const double alpha_=1e-3, const double beta_=2.0, const double kappa_=0.0) {

        alpha = alpha_;
        beta = beta_;
        kappa = kappa_;
        lambda = pow(alpha, 2)/(nx + kappa) - nx;
        eta = sqrt(nx + lambda);

        // Weights for each sigma point. Normalizes mean and covariance
        weights_mean(0) = lambda/(nx + lambda);
        weights_cov(0) = lambda/(nx + lambda) + 1.0 - pow(alpha, 2) + beta;

        if (useSRformulation) {
            weights_cov_sr(0) = sqrt(fabs(weights_cov(0)));
            central_cov_sgn = (weights_cov(0) >= 0.0) ? 1.0 : -1.0;
        }

        for (size_t i = 1; i < ns; ++i) {
            weights_mean(i) = weights_cov(i) = 0.5/(nx + lambda);
            if (useSRformulation) {
                weights_cov_sr(i) = sqrt(fabs(weights_mean(i)));
            }
        }
    }

    void predict(vec &u) override {

        if (!useSRformulation) {
            Ps = chol(P, "lower");
        }

        // Calculate sigma points
        if (cubature) {
            for (size_t i = 0; i < nx; ++i) {
                sigma_points.col(i) = x_est + eta*Ps.col(i);
                sigma_points.col(i + nx) = x_est - eta*Ps.col(i);
            }
        }
        else {
            sigma_points.col(0) = x_est;
            for (size_t i = 1; i < nx + 1; ++i) {
                sigma_points.col(i) = x_est + eta*Ps.col(i-1);
                sigma_points.col(i + nx) = x_est - eta*Ps.col(i-1);
            }
        }

        // Predict transformed state of sigma points
        for (size_t i = 0; i < ns; ++i) {
            if (!RK4Iterations) {
                eulerForward(sigma_points.col(i), u, dt, dxd);
            } else {
            rk4_multi(sigma_points.col(i), u, dt, RK4Iterations, dxd);
            }
            sigma_points_pred.col(i) = *(dxd.x_next);

        }

        // Calculate mean of predicted sigma points
        x_pred = arma::zeros(nx);
        for (size_t i = 0; i < ns; ++i) {
            x_pred += weights_mean(i) * sigma_points_pred.col(i);

        }

        // Calculate covariance of predicted sigma points
        if (useSRformulation) {
            x_pred_errors = zeros(nx, 2*nx);
            if (cubature) {
                for (size_t i = 0; i < ns; ++i) {
                    x_pred_errors.col(i) = weights_cov_sr(i) * (sigma_points_pred.col(i) - x_pred);
                }
            }
            else {
                for (size_t i = 1; i < ns; ++i) {
                    x_pred_errors.col(i-1) = weights_cov_sr(i) * (sigma_points_pred.col(i) - x_pred);
                }
                mat _X = join_horiz(x_pred_errors, Qs).t();
                mat _Q;
                mat _R;
                qr_econ(_Q, _R, _X);
                Ps = _R.t();
                cholUpdate(Ps, weights_cov_sr(0)*(sigma_points_pred.col(0) - x_pred), central_cov_sgn);
            }
        }
        else {
            P = Q;
            for (size_t i = 0; i < ns; ++i) {
                vec _e = sigma_points_pred.col(i) - x_pred;
                P += weights_cov(i) * _e * _e.t();
            }

            // Averaging for symmetry. Small regularization for positive definiteness
            P = (P + P.t())*0.5 + eye(nx, nx)*1e-12;
            if (!P.is_sympd(1e-9)) {
                std::cout << "P is not symmetric positive definite!" << endl;
            }
        }
    }

    void update(vec &z) override {
        // Assuming feedthrough is compensated for
        vec u = zeros(nu);

        // Calculate sigma points based on predicted density
        if (cubature) {
            if (!useSRformulation) {
                Ps = chol(P, "lower");
            }
            for (size_t i = 0; i < nx; ++i) {
                sigma_points_pred.col(i) = x_pred + eta*Ps.col(i);
                sigma_points_pred.col(i + nx) = x_pred - eta*Ps.col(i);
            }
        }

        // Predicted measurement for each predicted sigma point
        for (size_t i = 0; i < ns; ++i) {
            measurements_h(sigma_points_pred.col(i), u, z_pred);
            sigma_points_meas.col(i) = z_pred;
        }

        // Calculate mean of predicted measurements
        z_pred = arma::zeros(nz);
        for (size_t i = 0; i < ns; ++i) {
            z_pred += weights_mean(i) * sigma_points_meas.col(i);
        }

        // Calculate covariance of predicted measurements
        if (useSRformulation) {
            z_pred_errors = zeros(nz, 2*nx);
            if (cubature) {
                for (size_t i = 0; i < ns; ++i) {
                    z_pred_errors.col(i) = weights_cov_sr(i) * (sigma_points_meas.col(i) - z_pred);
                }
            }
            else {
                for (size_t i = 1; i < ns; ++i) {
                    z_pred_errors.col(i-1) = weights_cov_sr(i) * (sigma_points_meas.col(i) - z_pred);
                }
                mat _X = join_horiz(z_pred_errors, Rs).t();
                mat _Q;
                mat _R;
                qr_econ(_Q, _R, _X);
                Ss = _R.t();
                vec _e = sigma_points_meas.col(0) - z_pred;
                cholUpdate(Ss, weights_cov_sr(0)*(sigma_points_meas.col(0) - z_pred), central_cov_sgn);
            }
        }
        else {
            S = R;
            for (size_t i = 0; i < ns; ++i) {
                vec _e = sigma_points_meas.col(i) - z_pred;
                S += weights_cov(i) * _e * _e.t();
            }
            // Averaging for symmetry. Small regularization for positive definiteness
            S = (S + S.t())*0.5 + eye(nz, nz)*1e-12;
            if (!S.is_sympd(1e-9)) {
                std::cout << "S is not symmetric positive definite!" << endl;
            }
        }

        // Calculate cross-covariance
        Pxz = arma::zeros(nx, nz);
        if (useSRformulation) {
            Pxz += weights_cov(0)*(sigma_points_pred.col(0) - x_pred)*(sigma_points_meas.col(0) - z_pred).t();
            Pxz += x_pred_errors * z_pred_errors.t();
        }
        else {
            for (size_t i = 0; i < ns; ++i) {
                vec _e = sigma_points_pred.col(i) - x_pred;
                vec _ee = sigma_points_meas.col(i) - z_pred;
                Pxz += weights_cov(i) * _e * _ee.t();
            }
        }

        if (useSRformulation) {
            // W = solve(Ss.t()*Ss, Pxz.t(), solve_opts::likely_sympd).t();
            W = solve(trimatu(Ss.t()), solve(trimatl(Ss), Pxz.t())).t();
        }
        else {
            // Calculate Kalman gain
            //W = Pxz * Sinv;
            W = solve(S, Pxz.t(), solve_opts::likely_sympd).t();
        }

        // Innovation
        innovation = z - z_pred;

        // Update state estimate
        x_est = x_pred + W * innovation;

        if (useSRformulation) {
            if (!cubature) {
                cholUpdate(Ps, W*Ss, -1.0);
            }
            else {
                mat _Q;
                mat _R;
                mat _X = join_horiz(x_pred_errors - W*z_pred_errors, W*Rs).t();
                qr_econ(_Q, _R, _X);
                Ps = _R.t();
            }
        }
        else {
            // Update covariance estimate
            P = P - W * S * W.t();
            // Averaging for symmetry. Small regularization for positive definiteness
            P = (P + P.t())*0.5 + eye(nx, nx)*1e-12;
            if (!P.is_sympd(1e-9)) {
                std::cout << "P is not symmetric positive definite!" << endl;
            }
        }
    }

    double getAlpha() const {
        return alpha;
    }

    double getBeta() const {
        return beta;
    }

    double getKappa() const {
        return kappa;
    }

    double getLambda() const {
        return lambda;
    }

    double getWeightMean0() const {
        return weights_mean(0);
    }

    double getWeightCov0() const {
        return weights_cov(0);
    }

    // All weights for covariance and mean are the same for index != 0
    double getWeights() const {
        return weights_mean(1);
    }

    mat getSigmaPoints() const {
        return sigma_points;
    }

    mat getSigmaPoints_pred() const {
        return sigma_points_pred;
    }

    mat getSigmaPointsMeas() const {
        return sigma_points_meas;
    }

    mat getCrossCovariance() const {
        return Pxz;
    }
};

#endif // UNSCENTED_KALMAN_FILTER_H
