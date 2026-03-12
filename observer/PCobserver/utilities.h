#pragma once

#include <armadillo>

#define NUMBER_STATES 12         // all states
#define NUMBER_STATES_REDUCED 10 // states minus rotation around z axis
#define NUMBER_INPUTS 4          // input from solenoids
#define NUMBER_MEASUREMENTS 3    // magnetic field measurements in xyz directions

using namespace arma;

struct van_loan_struct {
    mat Qd; // discrete process covariance
    mat Ad; // discrete system matrix
};

struct derivatives_struct {
    vec *dx;      // continuous derivative
    vec *x_next;  // discretized next value based on dx
};


void increaseStateSpace(const vec &x, double x_pad[NUMBER_STATES]);
void reduceStateSpace(const double x_pad[NUMBER_STATES], vec &x);
mat calculateJacobian(const vec &x, const vec &u, const vec &curr, const double &dt = 0, const int &mode = 0);
mat discretize_A(const mat &A, const double &dt);
mat discretize_B(const mat &A, const mat &Ad, const mat &B);
van_loan_struct van_loan(const mat &A, const mat &Q, const double &dt);
mat QRr(const mat &X);
void cholUpdate(mat &L, const mat &x, const double &c);
