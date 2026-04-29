#pragma once

#include <armadillo>

#define NUMBER_STATES 12         // all states
#define NUMBER_STATES_REDUCED 10 // states minus rotation around z axis
#define NUMBER_STATES_REDUCED_EXTRA 6 // states minus rotation around z axis and orientation
#define NUMBER_STATES_TEST 5 // only for testing observer on coordinated turn model
#define NUMBER_OBSERVER_STATES NUMBER_STATES_REDUCED
#define NUMBER_INPUTS 4          // input from solenoids
#define NUMBER_SENSORS 1         // number of magnetic sensors
#define NUMBER_MEASUREMENTS_PER_SENSOR 3 // magnetic field measurements in xyz directions
#define NUMBER_MEASUREMENTS NUMBER_SENSORS*NUMBER_MEASUREMENTS_PER_SENSOR // total measurements

extern bool testing;

using namespace arma;

struct van_loan_struct {
    mat Qd; // discrete process covariance
    mat Ad; // discrete system matrix
};

struct derivatives_struct {
    vec *dx;      // continuous derivative
    vec *x_next;  // discretized next value based on dx
};

struct FilterParams {
    vec x0;
    mat P0, Ad, Bd, H, D, Qd, R;
    double dt;
};


void increaseStateSpace(const vec &x, double x_pad[NUMBER_STATES]);
void reduceStateSpace(const double x_pad[NUMBER_STATES], vec &x);
void dynamics_f(const vec &x, const vec &u, vec &dx);
void measurements_h(const vec &x, const vec &u, vec &z);
mat calculateJacobian(const vec &x, const vec &u, const int jacType, const vec &curr, const double dt = 0, const int mode = 0);
mat discretize_A(const mat &A, const double dt);
mat discretize_B(const mat &A, const mat &Ad, const mat &B);
van_loan_struct van_loan(const mat &A, const mat &Q, const double dt);
void cholUpdate(mat &L, const mat &x, const double c);
