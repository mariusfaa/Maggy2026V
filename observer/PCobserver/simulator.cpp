
#include "integrator.h"
#include "kalmanFilter.h"
#include "utilities.h"
#include "matrices.h"
#include "observer.h"
#include <armadillo>
#include <cmath>

using namespace arma;

const double dt = 0.01;

int main() {

    // Initial conditions
    double zEq = 0.030119178665731;
    vec xLp(NUMBER_STATES, arma::fill::zeros);
    xLp(3) = zEq;
    vec x0 = xLp.rows(0, NUMBER_STATES_REDUCED-1);

    // Perturbation
    x0(0) += 0.001;
    x0(1) += -0.000;
    x0(2) += 0.00;

    // Control gain
    mat K = get_K();

    // Observer
    initObserver(0);

    // Time vector
    double t_start = 0.0;
    double t_end = 1.0;
    int N = static_cast<int>((t_end - t_start) / dt) + 1;

    vec t = regspace(t_start, dt, t_end);

    // Preallocate arrays
    mat x(NUMBER_STATES_REDUCED, N, arma::fill::zeros);
    mat u(NUMBER_INPUTS, N, arma::fill::zeros);
    mat z(NUMBER_MEASUREMENTS, N, arma::fill::zeros);
    mat x_est(NUMBER_STATES, N, arma::fill::zeros);
    vec nis(N, arma::fill::zeros);
    vec nees(N, arma::fill::zeros);

    x.col(0) = x0;

    // Derivative
    vec dx(NUMBER_STATES_REDUCED, arma::fill::zeros);
    vec x_next(NUMBER_STATES_REDUCED, arma::fill::zeros);
    derivatives_struct Hatsune_Miku = {&dx, &x_next};

    // Simulation loop
    for (int k = 1; k < N; ++k) {

        double tk = t(k);
        vec zk = z.col(k);
        vec xk_est = x_est.col(k);
        vec x_prev = x.col(k-1);
        vec u_prev = u.col(k-1);

        // Simulate continuous plant using RK4
        rk4(x_prev, u_prev, dt, Hatsune_Miku);
        x.col(k) = *Hatsune_Miku.x_next;

        // Get measurement at current time
        measurements_h(x.col(k), zeros(NUMBER_INPUTS,1), zk);
        z.col(k) = zk;

        // Estimate
        runObserver(u_prev.memptr(), zk.memptr(), xk_est.memptr());
        x_est.col(k) = xk_est;

        // Compute control input using estimated state
        u.col(k) = -K * (xk_est - xLp);

        // Debug
        nis(k-1) = KF.getNIS();
        mat P = KF.getCovariance();
        mat Pinv = inv(P);
        vec xk_est_red(NUMBER_STATES_REDUCED, arma::fill::zeros);
        reduceStateSpace(xk_est.memptr(), xk_est_red);
        vec error = xk_est_red - x.col(k);
        nees(k) = as_scalar(error.t()*Pinv*error);

        // std::cout << xk_est - x.col(k) << endl;
        // std::cout << KF.getMeasPred() - zk << endl;
        std::cout << KF.getInnovationCovariance() << endl;
        std::cout << KF.getCovariance() << endl;
    }

#define SAVE_TO_CSV
#ifdef SAVE_TO_CSV
    std::ofstream file("simulation_results.csv");
    file << "t,x,y,z,x_est,y_est,z_est,nis,nees\n";
    for (int i = 0; i < N; ++i) {
        file << t(i) << "," 
             << x(0,i) << "," << x(1,i) << "," << x(2,i) << ","
             << x_est(0,i) << "," << x_est(1,i) << "," << x_est(2,i) << ","
             << nis(i) << "," << nees(i)

             << "\n";
    }
    file.close();
#endif
    return 0;
}


