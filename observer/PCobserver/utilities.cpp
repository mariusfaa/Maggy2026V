
#include "utilities.h"
#include "include/matlab/maglevModel.h"
#include "integrator.h"
#include <armadillo>
#include <csignal>
#include <cstddef>
#include <cstdio>

using namespace arma;

bool testing = 0;

// Set states that are observed
void increaseStateSpace(const vec &x, double x_pad[NUMBER_STATES]) {
  size_t offset = 0;
  for (size_t i = 0; i < NUMBER_STATES; i++) {
    if (x.n_elem == NUMBER_STATES_REDUCED_EXTRA) {
      if (i == 9 || i == 10) {
        offset++;
      }
    }
    if (i == 5 || i == 11) {
      offset++;
    } else {
      x_pad[i] = x(i - offset);
    }
  }
}


// Pop unobserved states
void reduceStateSpace(const double x_pad[NUMBER_STATES], vec &x) {
  int offset = 0;
  for (size_t i = 0; i < NUMBER_STATES; i++) {
    if (i == 5 || i == 11) {
      offset++;
    } else {
      x(i - offset) = x_pad[i];
    }
  }
}


void dynamics_f(const vec &x, const vec &u, vec &dx) {
  switch (x.n_elem) {
    case NUMBER_STATES_REDUCED:
    maglevSystemDynamics_red(x.memptr(), u.memptr(), dx.memptr());
    break;

    case NUMBER_STATES_REDUCED_EXTRA:
    maglevSystemDynamics_xred(x.memptr(), u.memptr(), dx.memptr());
    break;

    // Test model for filter verification
    case NUMBER_STATES_TEST:
    dx(0) = x(3)*cos(x(2));
    dx(1) = x(3)*sin(x(2));
    dx(2) = x(4);
    dx(3) = u(0);
    dx(4) = u(1);
    break;
  }
}


void measurements_h(const vec &x, const vec &u, vec &z) {
  switch (x.n_elem) {
    case NUMBER_STATES_REDUCED:
    maglevSystemMeasurements_red(x.memptr(), u.memptr(), z.memptr());
    break;

    case NUMBER_STATES_REDUCED_EXTRA:
    maglevSystemMeasurements_xred(x.memptr(), u.memptr(), z.memptr());
    break;

    // Test model for filter verification
    case NUMBER_STATES_TEST:
    // Three range sensors in a triangle around origin
    double posx0, posx1, posx2, posy0, posy1, posy2;
    posx0 = posy0 = posy1 = -1.0;
    posx1 = posy2 = 1.0;
    posx2 = 0.0;
    z(0) = sqrt(pow(x(0)-posx0, 2) + pow(x(1)-posy0, 2));
    z(1) = sqrt(pow(x(0)-posx1, 2) + pow(x(1)-posy1, 2));
    z(2) = sqrt(pow(x(0)-posx2, 2) + pow(x(1)-posy2, 2));
    break;
  }
}

// mode: 0 forward difference, 1 backward difference, 2 central
// jacType: 0 is process, 1 is measurement
mat calculateJacobian(const vec &x, const vec &u, const int jacType, const vec &curr,  const double dt, const int mode) {

  double delta = 1e-12;

  size_t n = curr.n_rows;
  size_t m = x.n_elem;

  mat jac = zeros(n, m);

  for (int i = 0; i < m; ++i) {

    // Zero except for the index of the variable to be differentiated
    vec deltavec = zeros(m);
    deltavec(i) = delta;

    vec x_diff = x + deltavec;
    vec diff_x = x - deltavec;


    // Container for f(x + delta)
    vec df = zeros(n);

    // Container for f(x - delta)
    vec fd = zeros(n);

    // Process jacobian
    if (jacType == 0) {
      if (dt == 0) { // continuous jacobian Ac

        if (mode == 0 || mode == 2) {
          dynamics_f(x_diff, u, df);
        }
        if (mode == 1 || mode == 2) {
          dynamics_f(diff_x, u, fd);
        }
      }
      else { // discrete jacobian F
        vec dx = zeros(m);
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
    else if (jacType == 1) {
      double dh[NUMBER_MEASUREMENTS] = {};

      if (mode == 0 || mode == 2) {
        measurements_h(x_diff, u, df);
      }
      if (mode == 1 || mode == 2) {
        measurements_h(diff_x, u, fd);
      }
    }
    else {
      printf("Unknown Jacobian type!");
    }

    vec col = zeros(m);

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
mat discretize_A(const mat &A, const double dt) {
  return expmat(A*dt);
}


// Exact discretization of input matrix. Assumes that A is invertible
mat discretize_B(const mat &A, const mat &Ad, const mat &B) {
  size_t n = A.n_cols;
  return solve(A, (Ad + eye(n,n))*B);
}


// Using van-loan's method to discretize process covariance matrix
van_loan_struct van_loan(const mat &A, const mat &Q, const double dt) {
  size_t n = A.n_rows;

  mat M(2*n, 2*n, arma::fill::zeros);
  M.submat(0, 0, n-1, n-1) = -A;
  M.submat(n, n, 2*n-1, 2*n-1) = A.t();
  M.submat(0, n, n-1, 2*n-1) = Q;

  mat phi = expmat(M*dt);
  mat phi12 = phi.submat(0, n, n-1, 2*n-1);
  mat phi22 = phi.submat(n, n, 2*n-1, 2*n-1);
  
  mat Ad = phi22.t();

  mat Qd = Ad*phi12;

  // Averaging for symmetry. Small regularization for positive definiteness
  Qd = (Qd + Qd.t())*0.5 + eye(n, n)*1e-9;

  if (!Qd.is_sympd(1e-9)) {
    std::cout << "Qd is not symmetric positive definite!" << endl;
  }

  return {Qd, Ad};
}

// If L = chol(P, "lower") then this computes chol(P + c*x*x^T, "lower")
void cholUpdate(mat &L, const mat &x_, const double c_) {
  size_t n = x_.n_rows;
  size_t m = x_.n_cols;
  vec x = x_.col(m-1);
  double c = c_;

  // If u is a matrix
  if (m != 1) {
    // Pop last column and use as new input
    mat _x = x_.submat(0, 0, n-1, m-2);

    // Modify cholesky factor recursively
    cholUpdate(L, _x, c);
  }

  // Rank 1 update
  for (size_t i = 0; i < n-1; ++i) {
    vec l = L.col(i);
    double li = l(i);
    double xi = x(i);
    double di = sqrt(pow(li, 2) + c*pow(xi, 2)); // New diagonal value
    L.col(i) = (li/di)*l + (c*xi/di)*x;     // New column value
    x = x - l*(xi/li);
    c = c * pow(li/di, 2);
  }
  L(n-1, n-1) = sqrt(pow(L(n-1, n-1), 2) + c*pow(x(n-1), 2));
}

