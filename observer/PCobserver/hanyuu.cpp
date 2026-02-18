
#include <armadillo>
#include <iostream>
#include "matrices.h"

using namespace arma;

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
  int n = arma::size(A)[0];
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

  //AB_diff();
  //Q_diff();
  std::cout << norm(get_A_fast()) << endl;


  return 0;
}
