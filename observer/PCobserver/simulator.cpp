
#include "integrator.h"
#include "kalmanFilter.h"
#include "utilities.h"
#include "matrices.h"
#include "observer.h"
#include <armadillo>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <random>
#include <chrono>

#define NUMBER_SIMULATOR_STATES NUMBER_STATES_REDUCED
bool extra_reduced = NUMBER_OBSERVER_STATES == NUMBER_STATES_REDUCED_EXTRA;

using namespace arma;
using namespace std::chrono;
using filterPtr = std::unique_ptr<KalmanFilter>;

const double dt = 0.01; // 100 Hz

int main(int argc, char* argv[]) {

    // Default arguments
    int filterVariant = 1;
    bool useSRformulation = 0;
    int RK4Iterations = 0;
    bool updateJacobians = 1;
    bool updateQ = 0;
    bool cubature = 0;
    bool normalized = 0;

    // Process arguments
    if (argc > 1) {
        filterVariant = *argv[1] - '0';
        if (argc > 2) {
            useSRformulation = *argv[2] - '0';
            if (argc > 3) {
                RK4Iterations = *argv[3] - '0';
                if (argc > 4) {
                    updateJacobians = *argv[4] - '0';
                    if (argc > 5) {
                        updateQ = *argv[5] - '0';
                        if (argc > 6) {
                            cubature = *argv[6] - '0';
                            if (argc > 7) {
                                normalized = *argv[7] - '0';
                            }
                        }
                    }
                }
            }
        }
    }

    // Saving simulation data
    char fileName[100];
    sprintf(fileName, "results/simulation_results.csv");
    std::ofstream file(fileName);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file" << std::endl;
        return 1;
    }

    // Writing header and flushing to file immediately
    file << "t,"
        // True states
        << "x,y,z,alpha,beta,x_dot,y_dot,z_dot,alpha_dot,beta_dot,"
        // Estimated states
        << "x_est,y_est,z_est,";
    if (!extra_reduced) {
        file << "alpha_est,beta_est,";
    }
    file << "x_dot_est,y_dot_est,z_dot_est,";
    if (!extra_reduced) {
        file << "alpha_dot_est,beta_dot_est,";
    }
    // State estimation errors
    file << "err_x,err_y,err_z,";
    if (!extra_reduced) {
        file << "err_alpha,err_beta,";
    }
    file << "err_x_dot,err_y_dot,err_z_dot,";
    if (!extra_reduced) {
        file << "err_alpha_dot,err_beta_dot,";
    }
    // Confidence bounds (standard deviations)
    file << "x_std,y_std,z_std,";
    if (!extra_reduced) {
        file << "alpha_std,beta_std,";
    }
    file << "x_dot_std,y_dot_std,z_dot_std,";
    if (!extra_reduced) {
        file << "alpha_dot_std,beta_dot_std,";
    }
    // Measurements
    if (NUMBER_SENSORS == 1) {
        file << "bx,by,bz,";
    } else if (NUMBER_SENSORS == 3) {
        file << "bx0,by0,bz0,bx1,by1,bz1,bx2,by2,bz2,";
    }
    // Estimated measurements
    if (NUMBER_SENSORS == 1) {
        file << "bx_est,by_est,bz_est,";
    } else if (NUMBER_SENSORS == 3) {
        file << "bx0_est,by0_est,bz0_est,bx1_est,by1_est,bz1_est,bx2_est,by2_est,bz2_est,";
    }
    // Confidence bounds (standard deviations)
    if (NUMBER_SENSORS == 1) {
        file << "bx_std,by_std,bz_std,";
    } else if (NUMBER_SENSORS == 3) {
        file << "bx0_std,by0_std,bz0_std,bx1_std,by1_std,bz1_std,bx2_std,by2_std,bz2_std,";
    }

    // NIS and NEES
    file << "nis,nees,"
        // Numerical diagnostics and runtime
        << "cond_P,det_P,"
        << "cond_S,det_S,"
        << "max_eigen_P,min_eigen_P,ratio_eigen_P,"
        << "max_eigen_S,min_eigen_S,ratio_eigen_S,"
        << "pos_def_P,pos_def_S,"
        << "ratio_eigen_F,"
        << "runtime\n";

    file << std::flush;

    // Random number generator setup for measurement noise
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dis_x(0.0, std::sqrt(get_R()(0,0)));
    std::normal_distribution<> dis_y(0.0, std::sqrt(get_R()(1,1)));
    std::normal_distribution<> dis_z(0.0, std::sqrt(get_R()(2,2)));

    // Process noise. Discretizing with van loan by using integral of states system matrix
    arma::arma_rng::set_seed_random();
    mat NSD_sim(5, 5, arma::fill::zeros); // Spectral density
    NSD_sim.load("NSD_sim.txt");
    // NSD_sim(0, 0) = 1.6e-8;
    // NSD_sim(1, 1) = 1.6e-8;
    // NSD_sim(2, 2) = 1.6e-8;
    // NSD_sim(3, 3) = 1e-8;
    // NSD_sim(4, 4) = 1e-8;
    mat Q_sim(NUMBER_SIMULATOR_STATES, NUMBER_SIMULATOR_STATES, arma::fill::zeros);
    Q_sim.submat(5, 5, NUMBER_SIMULATOR_STATES-1, NUMBER_SIMULATOR_STATES-1) = NSD_sim;
    van_loan_struct vls = van_loan(get_A_integrator(), Q_sim, dt);
    mat sim_Qd = vls.Qd;

    mat sim_noise_std = chol(sim_Qd, "lower");


    // Linearization point at equilibrium
    double zEq = 0.030119178665731;
    vec xLp_true(NUMBER_SIMULATOR_STATES, arma::fill::zeros);

    xLp_true(2) = zEq;

    // Linearization point that is used for the observer
    // Only works if the numerical value is the
    vec xLp = xLp_true.rows(0, NUMBER_OBSERVER_STATES-1);


    // Initial state of true system is linearization point
    vec x0 = xLp_true;

    // Control gain
    mat K(NUMBER_INPUTS, NUMBER_SIMULATOR_STATES);
    K.load("feedbackGain.txt");

    // Observer
    filterPtr observer = initObserver(filterVariant, dt, xLp.memptr(), useSRformulation, RK4Iterations, updateJacobians, updateQ, cubature, normalized);

    // Time vector
    double t_start = 0.0;
    double t_end = 1.0;
    int N = static_cast<int>((t_end - t_start) / dt) + 1;

    vec t = regspace(t_start, dt, t_end);

    // Preallocate arrays
    mat x(NUMBER_SIMULATOR_STATES, N, arma::fill::zeros);
    mat u(NUMBER_INPUTS, N, arma::fill::zeros);
    mat z(NUMBER_MEASUREMENTS, N, arma::fill::zeros);
    mat x_est(NUMBER_OBSERVER_STATES+NUMBER_BIAS_STATES, N, arma::fill::zeros);
    vec nis(N, arma::fill::zeros);
    vec nees(N, arma::fill::zeros);

    x.col(0) = x0;
    x_est.col(0) = join_vert(xLp, zeros(NUMBER_BIAS_STATES, 1));

    vec xk_est = x_est.col(0);

    // These values should be popped when doing analysis
    vec zk(NUMBER_MEASUREMENTS, arma::fill::zeros);
    long long runtime = 0;

    // Derivative
    vec dx(NUMBER_SIMULATOR_STATES, arma::fill::zeros);
    vec x_next(NUMBER_SIMULATOR_STATES, arma::fill::zeros);
    derivatives_struct Hatsune_Miku = {&dx, &x_next};

    // Simulation loop
    for (int k = 0; k < N; ++k) {

        if (k != 0) {

        double tk = t(k);
        vec x_prev = x.col(k-1);
        vec u_prev = u.col(k-1);

        // Simulate continuous plant using RK4
        rk4_multi(x_prev, u_prev, dt, 100, NUMBER_SIMULATOR_STATES, Hatsune_Miku);
        x.col(k) = *Hatsune_Miku.x_next;

        // Process noise
        x.col(k) += sim_noise_std*randn<vec>(NUMBER_SIMULATOR_STATES);

        // Get measurement at current time
        zk = zeros(NUMBER_MEASUREMENTS);
        measurements_h(x.col(k), u_prev, zk);
        z.col(k) = zk;

        // Simulating how the maggy sensor reads field in mT with inverted z
        for (size_t i = 0; i < NUMBER_MEASUREMENTS; i+= NUMBER_MEASUREMENTS_PER_SENSOR) {
            zk(0+i) *= 1e+3;
            zk(1+i) *= 1e+3;
            zk(2+i) *= -1e+3;

            // Adding random noise to measurements
            zk(0+i) += dis_x(gen);
            zk(1+i) += dis_y(gen);
            zk(2+i) += dis_z(gen);
        }

        // Estimate with runtime
        xk_est = zeros(NUMBER_OBSERVER_STATES+NUMBER_BIAS_STATES);
        auto start = steady_clock::now();
        runObserver(u_prev.memptr(), zk.memptr(), xk_est.memptr(), *observer);
        auto end = steady_clock::now();
        runtime = duration_cast<microseconds>(end - start).count();
        x_est.col(k) = xk_est;

        // Compute control input using true state
        u.col(k) = -K * (x.col(k) - xLp_true);
        }


        // Various debug information saved to file each step in loop

        // NIS
        mat S = observer->getInnovationCovariance();
        mat Sinv = inv(S);
        vec innovation = observer->getInnovation();
        nis(k) = as_scalar(innovation.t()*Sinv*innovation);

        // NEES
        mat P = observer->getCovariance().submat(0,0,NUMBER_OBSERVER_STATES-1,NUMBER_OBSERVER_STATES-1);
        mat Pinv = inv(P);
        vec error(NUMBER_OBSERVER_STATES, arma::fill::zeros);
        if (!extra_reduced) {
            error = xk_est.rows(0, NUMBER_OBSERVER_STATES-1) - x.col(k).rows(0, NUMBER_OBSERVER_STATES-1);
        } else { // Considering offset from true angles being between position and derivative states
            error.rows(0,2) = xk_est.rows(0,2) - x.col(k).rows(0,2);
            error.rows(3,5) = xk_est.rows(3,5) - x.col(k).rows(5,7);
        }
        nees(k) = as_scalar(error.t()*Pinv*error);

        // Standard deviations
        vec deviations = arma::sqrt(P.diag());
        vec meas_deviations = arma::sqrt(S.diag());

        // Condition number and determinant
        double cond_P = cond(P);
        double det_P = det(P);

        double cond_S = arma::cond(S);
        double det_S = arma::det(S);

        // Eigenvalue analysis
        cx_vec eigval_P;
        cx_mat eigvec_P;
        eig_gen(eigval_P, eigvec_P, P);
        double max_eigen_P = arma::max(arma::real(eigval_P));
        double min_eigen_P = arma::min(arma::real(eigval_P));
        double ratio_eigen_P = max_eigen_P / min_eigen_P;

        cx_vec eigval_S;
        cx_mat eigvec_S;
        eig_gen(eigval_S, eigvec_S, S);
        double max_eigen_S = arma::max(arma::real(eigval_S));
        double min_eigen_S = arma::min(arma::real(eigval_S));
        double ratio_eigen_S = max_eigen_S / min_eigen_S;

        // Innovation magnitude
        double max_innovation = arma::max(arma::abs(innovation));

        // Check positive definiteness
        bool pos_def_P = min_eigen_P > 0.0;
        bool pos_def_S = min_eigen_S > 0.0;

        // Filter gain norm
        // double kf_gain_norm = norm(observer->getKalmanGain(), "fro");

        // Residual norm (innovation norm)
        // double residual_norm = norm(innovation);

        // Stiffness ratio
        cx_vec eigval_F;
        cx_mat eigvec_F;
        eig_gen(eigval_F, eigvec_F, observer->getF());
        double max_eigen_F = arma::max(arma::real(eigval_F));
        double min_eigen_F = arma::min(arma::real(eigval_F));
        double ratio_eigen_F = max_eigen_F / min_eigen_F;

        // Write all data for this time step
        file << t(k) << ",";

        // True states
        for (size_t i = 0; i < NUMBER_SIMULATOR_STATES; ++i) {
            file << x.col(k)(i) << ",";
        }

        // Estimated states
        for (size_t i = 0; i < NUMBER_OBSERVER_STATES; ++i) {
            file << xk_est(i) << ",";
        }

        // Errors
        for (size_t i = 0; i < NUMBER_OBSERVER_STATES; ++i) {
            file << error(i) << ",";
        }

        // State estimate confidence bounds
        for (size_t i = 0; i < NUMBER_OBSERVER_STATES; ++i) {
            file << deviations(i) << ",";
        }

        // Measurements. Given in tesla
        for (size_t i = 0; i < NUMBER_SENSORS; ++i) {
            file << zk(0+i*NUMBER_SENSORS)*1e-3 << ",";
            file << zk(1+i*NUMBER_SENSORS)*1e-3 << ",";
            file << -zk(2+i*NUMBER_SENSORS)*1e-3 << ",";

            // Estimated measurements. Given in tesla
            vec measPred = observer->getMeasPred();
            file << measPred(0+i*NUMBER_SENSORS) << ",";
            file << measPred(1+i*NUMBER_SENSORS) << ",";
            file << measPred(2+i*NUMBER_SENSORS) << ",";

            // Measurement confidence bounds
            file << meas_deviations(0+i*NUMBER_SENSORS) << ",";
            file << meas_deviations(1+i*NUMBER_SENSORS) << ",";
            file << meas_deviations(2+i*NUMBER_SENSORS) << ",";
        }

            // NIS and NEES
        file << nis(k) << "," << nees(k) << ","
            // Numerical diagnostics and runtime
            << cond_P << "," << det_P << ","
            << cond_S << "," << det_S << ","
            << max_eigen_P << "," << min_eigen_P << "," << ratio_eigen_P << ","
            << max_eigen_S << "," << min_eigen_S << "," << ratio_eigen_S << ","
            << pos_def_P << "," << pos_def_S << ","
            << ratio_eigen_F << ","
            << runtime << "\n";

        // Flush after each time step to ensure data is saved
        // This is critical for debugging crashes
        file << std::flush;


    }

    file.close();

    return 0;
}


