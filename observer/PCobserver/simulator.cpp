
#include "integrator.h"
#include "kalmanFilter.h"
#include "utilities.h"
#include "matrices.h"
#include "observer.h"
#include <armadillo>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <random>
#include <chrono>

#define NUMBER_SIMULATOR_STATES NUMBER_STATES_REDUCED

using namespace arma;
using namespace std::chrono;
using filterPtr = std::unique_ptr<KalmanFilter>;

const double dt = 0.005; // 200 Hz

int main(int argc, char* argv[]) {

    // Default arguments
    int filterVariant = 2;
    size_t numberObserverStates = 6;
    int8_t systemModel = 0;    // Default is fast
    bool useSRformulation = 0;
    int RK4Iterations = 0;
    bool updateJacobians = 1;
    bool updateQ = 0;
    bool cubature = 0;

    // Process arguments
    if (argc > 1) {
        filterVariant = *argv[1] - '0';
        if (argc > 2) {
            numberObserverStates = static_cast<size_t>(atoi(argv[2]));
            if (argc > 3) {
                systemModel = static_cast<int8_t>(atoi(argv[3]));
                if (argc > 4) {
                    useSRformulation = *argv[4] - '0';
                    if (argc > 5) {
                        RK4Iterations = atoi(argv[5]);
                        if (argc > 6) {
                            updateJacobians = *argv[6] - '0';
                            if (argc > 7) {
                                updateQ = *argv[7] - '0';
                                if (argc > 8) {
                                    cubature = *argv[8] - '0';
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Whether to estimate angles or just position
    bool onlyDisplacement = numberObserverStates == NUMBER_STATES_REDUCED_EXTRA;

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
    if (!onlyDisplacement) {
        file << "alpha_est,beta_est,";
    }
    file << "x_dot_est,y_dot_est,z_dot_est,";
    if (!onlyDisplacement) {
        file << "alpha_dot_est,beta_dot_est,";
    }
    // State estimation errors
    file << "err_x,err_y,err_z,";
    if (!onlyDisplacement) {
        file << "err_alpha,err_beta,";
    }
    file << "err_x_dot,err_y_dot,err_z_dot,";
    if (!onlyDisplacement) {
        file << "err_alpha_dot,err_beta_dot,";
    }
    // Confidence bounds (standard deviations)
    file << "x_std,y_std,z_std,";
    if (!onlyDisplacement) {
        file << "alpha_std,beta_std,";
    }
    file << "x_dot_std,y_dot_std,z_dot_std,";
    if (!onlyDisplacement) {
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
    mat R;
    R.load("R.txt");
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dis_x(0.0, std::sqrt(R(0,0)));
    std::normal_distribution<> dis_y(0.0, std::sqrt(R(1,1)));
    std::normal_distribution<> dis_z(0.0, std::sqrt(R(2,2)));

    // Process noise. Discretizing with van loan by using integral of states system matrix
    arma::arma_rng::set_seed_random();
    mat NSD_sim(5, 5, arma::fill::zeros); // Spectral density
    NSD_sim.load("NSD_sim.txt");
    mat Q_sim(NUMBER_SIMULATOR_STATES, NUMBER_SIMULATOR_STATES, arma::fill::zeros);
    Q_sim.submat(5, 5, NUMBER_SIMULATOR_STATES-1, NUMBER_SIMULATOR_STATES-1) = NSD_sim;
    van_loan_struct vls_sim = van_loan(get_A_integrator(), Q_sim, dt);
    mat sim_Qd = vls_sim.Qd;

    mat sim_noise_std = chol(sim_Qd, "lower");

    // Control gain
    mat K(NUMBER_INPUTS, NUMBER_SIMULATOR_STATES);
    K.load("feedbackGain.txt");

    // Linearization point at equilibrium
    double zEq = 0.030119178665731;
    vec xLp_true(NUMBER_SIMULATOR_STATES, arma::fill::zeros);

    xLp_true(2) = zEq;

    // Linearization point that is used for the observer
    // Only works if the numerical value is the
    vec xLp = xLp_true.rows(0, numberObserverStates-1);


    /*Observer Setup*/
    // Initial state of true system is linearization point
    vec x0 = xLp_true;

    // Assumed process noise
    mat Q, P0;
    onlyDisplacement ? Q.load("Qxred.txt") : Q.load("Q.txt");
    onlyDisplacement ? P0.load("P0xred.txt") : P0.load("P0.txt");

    // Linear integrating part of system. Only used to calculate discretized Q
    mat Aint = onlyDisplacement ? get_A_integrator_xred() : get_A_integrator();

    // Discretizing process noise
    van_loan_struct vls_obs = van_loan(Aint, Q, dt);
    mat Qd = vls_obs.Qd;

    // Creating observer object
    filterPtr observer = initObserver(filterVariant, dt, numberObserverStates, xLp.memptr(), R.memptr(), Qd.memptr(), P0.memptr(), useSRformulation, RK4Iterations, updateJacobians, updateQ, cubature);

    // Time vector
    double t_start = 0.0;
    double t_end = 1.0;
    int N = static_cast<int>((t_end - t_start) / dt) + 1;

    vec t = regspace(t_start, dt, t_end);

    // Preallocate arrays
    mat x(NUMBER_SIMULATOR_STATES, N, arma::fill::zeros);
    mat u(NUMBER_INPUTS, N, arma::fill::zeros);
    mat z(NUMBER_MEASUREMENTS, N, arma::fill::zeros);
    mat x_est(numberObserverStates, N, arma::fill::zeros);
    vec nis(N, arma::fill::zeros);
    vec nees(N, arma::fill::zeros);

    x.col(0) = x0;

    x_est.col(0) = xLp;
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
        rk4_multi(x_prev, u_prev, dt, 100, NUMBER_SIMULATOR_STATES, Hatsune_Miku, systemModel);
        x.col(k) = *Hatsune_Miku.x_next;

        // Abort if abnormally high values (indicating the magnet crashing)
        if (std::fabs(x.col(k)(0)) > 1e-1) {
            std::cout << "Simulation Failed" << endl;
            abort();
        }

        // Process noise
        x.col(k) += sim_noise_std*randn<vec>(NUMBER_SIMULATOR_STATES);

        // Get measurement at current time
        zk = zeros(NUMBER_MEASUREMENTS);
        measurements_h(x.col(k), u_prev, zk, systemModel);
        z.col(k) = zk;

        // Simulating how the maggy sensor reads field in mT with inverted z and adding random noise to measurements
        for (size_t i = 0; i < NUMBER_MEASUREMENTS; i+= NUMBER_MEASUREMENTS_PER_SENSOR) {
            zk(0+i) += dis_x(gen);
            zk(1+i) += dis_y(gen);
            zk(2+i) += dis_z(gen);

            zk(0+i) *= 1e+3;
            zk(1+i) *= 1e+3;
            zk(2+i) *= -1e+3;
        }

        // Estimate with runtime
        xk_est = zeros(numberObserverStates);
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
        mat P = observer->getCovariance().submat(0,0,numberObserverStates-1,numberObserverStates-1);
        mat Pinv = inv(P);
        vec error(numberObserverStates, arma::fill::zeros);
        if (!onlyDisplacement) {
            error = xk_est.rows(0, numberObserverStates-1) - x.col(k).rows(0, numberObserverStates-1);
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

        // Multiplying by 1e+3 to get mm and mrad values instead of very low meter and rad values

        // True states
        for (size_t i = 0; i < NUMBER_SIMULATOR_STATES; ++i) {
            file << x.col(k)(i)*1e+3 << ",";
        }

        // Estimated states
        for (size_t i = 0; i < numberObserverStates; ++i) {
            file << xk_est(i)*1e+3 << ",";
        }

        // Errors
        for (size_t i = 0; i < numberObserverStates; ++i) {
            file << error(i)*1e+3 << ",";
        }

        // State estimate confidence bounds
        for (size_t i = 0; i < numberObserverStates; ++i) {
            file << deviations(i)*1e+3 << ",";
        }

        // Measurements. Given in millitesla
        for (size_t i = 0; i < NUMBER_SENSORS; ++i) {
            file << zk(0+i*NUMBER_SENSORS) << ",";
            file << zk(1+i*NUMBER_SENSORS) << ",";
            file << -zk(2+i*NUMBER_SENSORS) << ",";

            // Estimated measurements. Given in millitesla
            vec measPred = observer->getMeasPred();
            file << measPred(0+i*NUMBER_SENSORS)*1e+3 << ",";
            file << measPred(1+i*NUMBER_SENSORS)*1e+3 << ",";
            file << measPred(2+i*NUMBER_SENSORS)*1e+3 << ",";

            // Measurement confidence bounds
            file << meas_deviations(0+i*NUMBER_SENSORS)*1e+3 << ",";
            file << meas_deviations(1+i*NUMBER_SENSORS)*1e+3 << ",";
            file << meas_deviations(2+i*NUMBER_SENSORS)*1e+3 << ",";
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


