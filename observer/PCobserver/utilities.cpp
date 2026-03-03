
#include "utilities.h"
#include "include/matlab/maglevModel.h"
#include "integrator.h"
#include <armadillo>
#include <cstdio>

using namespace arma;


// Set states that are observed
void increaseStateSpace(const vec &x, double x_pad[NUMBER_STATES]) {
  int offset = 0;
  for (int i = 0; i < NUMBER_STATES; i++) {
    if (i == 5 || i == 11) {
      offset++;
    } else {
      x_pad[i] = x(i - offset);
    }
  }
}


// Pop unobservable states
void reduceStateSpace(const double x_pad[NUMBER_STATES], vec &x) {
  int offset = 0;
  for (int i = 0; i < NUMBER_STATES; i++) {
    if (i == 5 || i == 11) {
      offset++;
    } else {
      x(i - offset) = x_pad[i];
    }
  }
}

// mode: 0 forward differemce, 1 backward difference, 2 central
mat calculateJacobian(const vec &x, const vec &u, const vec &curr, const double &dt, const int &mode) {

  double delta = 1e-12;

  size_t n = curr.n_rows;

  mat jac = zeros(n, NUMBER_STATES_REDUCED);

  for (int i = 0; i < NUMBER_STATES_REDUCED; ++i) {

    // Initialize in loop to ensure reset to 0
    double x_pad[NUMBER_STATES] = {};

    // Zero except for the index of the variable to be differentiated
    vec deltavec = zeros(NUMBER_STATES_REDUCED);
    deltavec(i) = delta;

    vec x_diff = x + deltavec;
    vec diff_x = x - deltavec;


    // Container for f(x + delta)
    vec df = zeros(NUMBER_STATES_REDUCED);

    // Container for f(x - delta)
    vec fd = zeros(NUMBER_STATES_REDUCED);


    if (n == NUMBER_STATES_REDUCED) {
      if (dt == 0) { // continuous jacobian Ac
        double dx_pad[NUMBER_STATES] = {};

        if (mode == 0 || mode == 2) {
          increaseStateSpace(x_diff, x_pad);
          maglevSystemDynamics_fast(x_pad, u.memptr(), dx_pad);
          reduceStateSpace(dx_pad, df);
        }
        if (mode == 1 || mode == 2) {
          increaseStateSpace(diff_x, x_pad);
          maglevSystemDynamics_fast(x_pad, u.memptr(), dx_pad);
          reduceStateSpace(dx_pad, fd);
        }
      }
      else { // discrete jacobian F
        vec dx = zeros(NUMBER_STATES_REDUCED);
        derivatives_struct ds;
        derivatives_struct sd;
        ds.dx = &dx; // unused
        ds.x_next = &df;
        sd.dx = &dx; // unused
        sd.x_next = &fd;

        if (mode == 0 || mode == 2) {
          eulerForward(x_diff, u, dt, ds);
        }
        if (mode == 1 || mode == 2) {
          eulerForward(diff_x, u, dt, sd);
        }
      }
    }
    // Jacobian H
    else if (n == NUMBER_MEASUREMENTS) {
      double dh[NUMBER_MEASUREMENTS] = {};

      if (mode == 0 || mode == 2) {
        increaseStateSpace(x_diff, x_pad);
        maglevSystemMeasurements_fast(x_pad, u.memptr(), dh);
        df.resize(3);
        memcpy(df.memptr(), dh, sizeof(dh));
      }
      if (mode == 1 || mode == 2) {
        increaseStateSpace(diff_x, x_pad);
        maglevSystemMeasurements_fast(x_pad, u.memptr(), dh);
        fd.resize(3);
        memcpy(fd.memptr(), dh, sizeof(dh));
      }

    }
    else {
      printf("Unknown Jacobian type!");
    }

    vec col = zeros(NUMBER_STATES_REDUCED);

    if (mode == 0) {
      col = (df - curr)/delta;
    }
    else if (mode == 1) {
      col = (curr - fd)/delta;
    }
    else if (mode == 2) {
      col = (df - fd)/(2*delta);
    }
    else {
      printf("Unknown mode type!");
    }

    // Write to jacobian
    jac.col(i) = col;
  }
  return jac;
}


// Exact discretization of system matrix
mat discretize_A(const mat &A, const double &dt) {
  return expmat(A*dt);
}


// Exact discretization of input matrix. Assumes that A is invertible
mat discretize_B(const mat &A, const mat &Ad, const mat &B) {
  size_t n = A.n_cols;
  return inv(A)*(Ad + eye(n,n))*B;
}


// Using van-loan's method to discretize process covariance matrix
mat van_loan(const mat &A, const mat &Q, const double &dt) {
  size_t n = A.n_rows;

  mat M(2*n, 2*n, arma::fill::zeros);
  M.submat(0, 0, n-1, n-1) = -A;
  M.submat(n, n, 2*n-1, 2*n-1) = A.t();
  M.submat(0, n, n-1, 2*n-1) = Q;

  mat phi = expmat(M*dt);
  mat phi12 = phi.submat(0, n, n-1, 2*n-1);
  mat phi22 = phi.submat(n, n, 2*n-1, 2*n-1);

  mat Qd = phi22.t()*phi12;

  if (!Qd.is_sympd()) {
    std::cout << "Qd is not symmetric positive definite!" << endl;
  }

  return Qd;
}

