
#include <algorithm>
#include <armadillo>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <iostream>
#include "matrices.h"
#include "include/matlab/maglevModel.h"
#include "utilities.h"
#include "integrator.h"
#include <chrono>

using namespace arma;
using namespace std::chrono;

mat rel_diff(const mat A, const mat B) {
  double eps = 1e-12;
  size_t n = A.n_rows;
  size_t m = B.n_cols;
  mat C = zeros(n, m);

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < m; ++j) {
      double a = A(i, j);
      double b = B(i, j);
      C(i, j) = std::fabs(a - b)/std::max(std::max(std::fabs(a), std::fabs(b)), eps);
    }
  }
  return C;
}

void AB_diff() {
  mat A_matlab = get_A_d();
  mat B_matlab = get_B_d();
  mat A = get_A_fast();
  mat B = get_B_fast();
  double dt = 0.0005;
  mat A_arma = arma::expmat(A*dt);
  //mat A_diff = A_arma.clean(1e-9) - A_matlab;

  mat B_arma = arma::inv(A)*(A_arma - arma::eye(10, 10))*B;
  mat B_diff = B_arma.clean(1e-9) - B_matlab;

  std::cout << B_diff << endl;
}

mat van_loan() {
  mat A = get_A_fast();
  int n = A.n_rows;
  double dt = 0.0005;

  mat M(2*n, 2*n, fill::zeros);
  M.submat(0, 0, n-1, n-1) = -A;
  M.submat(n, n, 2*n-1, 2*n-1) = A.t();
  M.submat(0, n, n-1, 2*n-1) = get_Q();

  mat phi = expmat(M*dt);
  mat phi12 = phi.submat(0, n, n-1, 2*n-1);
  mat phi22 = phi.submat(n, n, 2*n-1, 2*n-1);

  mat Qd = phi22.t()*phi12;

  if (!Qd.is_sympd()) {
    std::cout << "Qd is not symmetric positive definite!" << endl;
  }

  return Qd;
}

void Q_diff () {
  mat Q_arma = van_loan();
  mat Q_matlab = get_Q_d();

  mat diff = Q_arma.clean(1e-9) - Q_matlab;

  std::cout << diff << endl;
}


void jacobian_diff() {

  mat A_matlab = get_A_fast();
  mat F_matlab = get_A_d();
  mat H_matlab = get_H_fast();

  double dt = 0.0005;
  double zEq = 0.030119178665731;

  vec x = zeros(10);
  double x_pad[12] = {};
  x(2) = zEq;
  x_pad[2] = zEq;

  vec u = zeros(4);
  vec dx_pad = zeros(12);
  vec dx = zeros(10);
  vec z = zeros(3);


  maglevSystemDynamics_fast(x_pad, u.memptr(), dx_pad.memptr());
  reduceStateSpace(dx_pad.memptr(), dx);

  derivatives_struct s;
  vec _dx = zeros(10);
  vec x_next = zeros(10);
  s.dx = &_dx;
  s.x_next = &x_next;
  eulerForward(x, u, dt, s);

  maglevSystemMeasurements_fast(x_pad, u.memptr(), z.memptr());

  mat Ac = calculateJacobian(x, u, dx);
  mat F_redisc = arma::eye(10, 10) + Ac*dt;
  mat F_redisc_good = discretize_A(Ac, dt);

  mat F_ref = discretize_A(get_A_fast(), dt);
  mat F = calculateJacobian(x, u, x_next, dt);
  mat F1 = calculateJacobian(x, u, x_next, dt, 1);
  mat F2 = calculateJacobian(x, u, x_next, dt, 2);

  mat H = calculateJacobian(x, u, z);
  mat H1 = calculateJacobian(x, u, z, 0, 1);
  mat H2 = calculateJacobian(x, u, z, 0, 2);


  mat diffF = F_redisc_good - F_ref;
  mat diffF1 = F1 - F_ref;
  mat diffF2 = F2 - F_ref;

  mat diffH = H - H_matlab;
  mat diffH1 = H1 - H_matlab;
  mat diffH2 = H2 - H_matlab;

  std::cout << F << endl;
  std::cout << "============================" << endl;
  std::cout << F_ref << endl;
  std::cout << "============================" << endl;
  std::cout << diffF << endl;
  // std::cout << diffH << endl;
  // std::cout << "============================" << endl;
  // std::cout << diffH1 << endl;
  // std::cout << "============================" << endl;
  // std::cout << diffH2 << endl;

}


void cholUpdateTest() {
  mat P = {{100, -50},
           {-50, 100}};
  mat L = chol(P, "lower");
  vec x = {1, 2};
  mat xmat = {{1, 3},
              {2, 4}};
  double c = -1;

  mat smart = L;
  mat smart2 = L;
  cholUpdate(smart, x, c);
  cholUpdate(smart2, xmat, c);
  mat naive = chol(L*L.t() + c*xmat.col(0)*xmat.col(0).t(), "lower");
  mat naive2 = chol(naive*naive.t() + c*xmat.col(1)*xmat.col(1).t(), "lower");

  std::cout << smart2 << endl;
  std::cout << naive2 << endl;
}


int main() {

  vec a = {4, 5, 6};
  mat A = {{3, 2, 1},
           {6, 5, 4},
           {9, 8, 7}};
  vec b(2);
  double *c = a.memptr();
  double *C = A.memptr();
  mat Atr = A.submat(0,1,1,2); // top right
  mat Aleft = A.submat(0,0,2,0);
  mat Abottom = A.submat(2,0,2,2);
  mat Arecon(3,3); Arecon.col(0) = Aleft; Arecon.row(2) = Abottom; Arecon.submat(0,1,1,2) = Atr;
  mat vcat = join_vert(a.t(), A);

  auto start = steady_clock::now();

  //AB_diff();
  //Q_diff();

  // jacobian_diff();
  cholUpdateTest();


  auto end = steady_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
  // std::cout << duration.count() << endl;

  return 0;
}
