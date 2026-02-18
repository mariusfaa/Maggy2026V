
#include "utilities.h"

using namespace arma;


// Set states that are observed
void increaseStateSpace(const vec *x, double x_pad[NUMBER_STATES]) {
  int offset = 0;
  for (int i = 0; i < NUMBER_STATES; i++) {
    if (i == 5 || i == 11) {
      offset++;
    } else {
      x_pad[i] = (*x)(i - offset);
    }
  }
}


// Pop unobservable states
void reduceStateSpace(const double x_pad[NUMBER_STATES], vec x) {
  int offset = 0;
  for (int i = 0; i < NUMBER_STATES; i++) {
    if (i == 5 || i == 11) {
      offset++;
    } else {
      x(i - offset) = x_pad[i];
    }
  }
}


// Exact discretization of system matrix
mat discretize_A(const mat A, const double dt) {
  return expmat(A*dt);
}


// Exact discretization of input matrix. Assumes that A is invertible
mat discretize_B(const mat A, const mat Ad, const mat B) {
  int n = arma::size(A)[0];
  return inv(A)*(Ad + eye(n,n))*B;
}


// Using van-loan's method to discretize process covariance matrix
mat van_loan(const mat A, const mat Q, const double dt) {
  int n = arma::size(A)[0];

  mat M(2*n, 2*n, fill::zeros);
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

