
#include "integrator.h"
#include "kalmanFilter.h"
#include "utilities.h"
#include "matrices.h"
#include "observer.h"
#include <armadillo>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <random>

using namespace arma;
using filterPtr = std::unique_ptr<KalmanFilter>;

const double dt = 0.01;

int main(int argc, char* argv[]) {

    // Default arguments
    int filterVariant = 1;
    bool useSRformulation = 0;
    bool updateJacobians = 1;
    bool updateQ = 0;
    bool cubature = 0;

    // Process arguments
    if (argc > 1) {
        filterVariant = *argv[1] - '0';
        if (argc > 2) {
            useSRformulation = *argv[2] - '0';
            if (argc > 3) {
                updateJacobians = *argv[3] - '0';
                if (argc > 4) {
                    updateQ = *argv[4] - '0';
                    if (argc > 5) {
                        cubature = *argv[5] - '0';
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
        << "x,y,z,alpha,beta,"
        << "x_dot,y_dot,z_dot,alpha_dot,beta_dot,"
        // Estimated states
        << "x_est,y_est,z_est,alpha_est,beta_est,"
        << "x_dot_est,y_dot_est,z_dot_est,alpha_dot_est,beta_dot_est,"
        // State estimation errors
        << "err_x,err_y,err_z,err_alpha,err_beta,"
        << "err_x_dot,err_y_dot,err_z_dot,err_alpha_dot,err_beta_dot,"
        // Confidence bounds (standard deviations)
        << "x_std,y_std,z_std,alpha_std,beta_std,"
        << "x_dot_std,y_dot_std,z_dot_std,alpha_dot_std,beta_dot_std,"
        // NIS and NEES
        << "nis,nees,"
        // Numerical diagnostics
        << "cond_P,det_P,"
        << "cond_S,det_S,"
        << "max_eigen_P,min_eigen_P,ratio_eigen_P,"
        << "max_eigen_S,min_eigen_S,ratio_eigen_S,"
        << "pos_def_P,pos_def_S,"
        << "ratio_eigen_F\n";

    file << std::flush;

    // Random number generator setup
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dis_x(0.0, std::sqrt(get_R()(0,0)));
    std::normal_distribution<> dis_y(0.0, std::sqrt(get_R()(1,1)));
    std::normal_distribution<> dis_z(0.0, std::sqrt(get_R()(2,2)));

    // Initial conditions
    double zEq = 0.030119178665731;
    vec xLp(NUMBER_OBSERVER_STATES, arma::fill::zeros);
    xLp(2) = zEq;
    vec x0 = xLp;


    // Perturbation
    x0(0) += 0.0001;
    x0(1) += -0.0001;
    x0(2) += 0.001;

    // Control gain
    mat K = get_K();

    // Observer
    filterPtr observer = initObserver(filterVariant, dt, useSRformulation, updateJacobians, updateQ, cubature);

    // Time vector
    double t_start = 0.0;
    double t_end = 5.0;
    int N = static_cast<int>((t_end - t_start) / dt) + 1;

    vec t = regspace(t_start, dt, t_end);

    // Preallocate arrays
    mat x(NUMBER_OBSERVER_STATES, N, arma::fill::zeros);
    mat u(NUMBER_INPUTS, N, arma::fill::zeros);
    mat z(NUMBER_MEASUREMENTS, N, arma::fill::zeros);
    mat x_est(NUMBER_OBSERVER_STATES, N, arma::fill::zeros);
    vec nis(N, arma::fill::zeros);
    vec nees(N, arma::fill::zeros);

    x.col(0) = x0;
    x_est.col(0) = xLp;

    // Derivative
    vec dx(NUMBER_OBSERVER_STATES, arma::fill::zeros);
    vec x_next(NUMBER_OBSERVER_STATES, arma::fill::zeros);
    derivatives_struct Hatsune_Miku = {&dx, &x_next};

    // Simulation loop
    for (int k = 1; k < N; ++k) {

        double tk = t(k);
        vec x_prev = x.col(k-1);
        vec u_prev = u.col(k-1);

        // Simulate continuous plant using RK4
        rk4_multi(x_prev, u_prev, dt, 100, Hatsune_Miku);
        x.col(k) = *Hatsune_Miku.x_next;

        // Get measurement at current time; assume input is compensated for
        vec zk(NUMBER_MEASUREMENTS, arma::fill::zeros);
        measurements_h(x.col(k), zeros(NUMBER_INPUTS,1), zk);
        z.col(k) = zk;

        // Simulating how the maggy sensor reads field in mT with inverted z
        zk(0) *= 1e+3;
        zk(1) *= 1e+3;
        zk(2) *= -1e+3;

        // Adding random noise to measurements
        zk(0) += dis_x(gen);
        zk(1) += dis_y(gen);
        zk(2) += dis_z(gen);

        // Estimate
        vec u_obs = u_prev; //zeros(4,1);
        vec xk_est(NUMBER_OBSERVER_STATES, arma::fill::zeros);
        runObserver(u_obs.memptr(), zk.memptr(), xk_est.memptr(), *observer);
        x_est.col(k) = xk_est;

        // Compute control input using estimated state
        vec control_state = xk_est;

        // Compute control input using true state
        control_state = x.col(k);

        // x_est.col(k) = true_state;
        u.col(k) = -K * (control_state - xLp);


        // Various debug information saved to file each step in loop

        // NIS
        mat S = observer->getInnovationCovariance();
        mat Sinv = inv(S);
        vec innovation = observer->getInnovation();
        nis(k) = as_scalar(innovation.t()*Sinv*innovation);

        // NEES
        mat P = observer->getCovariance();
        mat Pinv = inv(P);
        vec xk_est_red(NUMBER_OBSERVER_STATES, arma::fill::zeros);
        vec error = xk_est - x.col(k);
        nees(k) = as_scalar(error.t()*Pinv*error);

        // Standard deviations
        vec deviations = arma::sqrt(P.diag());

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
        for (size_t i = 0; i < NUMBER_OBSERVER_STATES; ++i) {
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

        // Confidence bounds (standard deviations)
        for (size_t i = 0; i < NUMBER_OBSERVER_STATES; ++i) {
            file << deviations(i) << ",";
        }

            // NIS and NEES
        file << nis(k) << "," << nees(k) << ","
            // Numerical diagnostics
            << cond_P << "," << det_P << ","
            << cond_S << "," << det_S << ","
            << max_eigen_P << "," << min_eigen_P << "," << ratio_eigen_P << ","
            << max_eigen_S << "," << min_eigen_S << "," << ratio_eigen_S << ","
            << pos_def_P << "," << pos_def_S << ","
            << ratio_eigen_F << "\n";

        // Flush after each time step to ensure data is saved
        // This is critical for debugging crashes
        file << std::flush;


    }

    file.close();

    return 0;
}


