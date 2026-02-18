#pragma once

#include <armadillo>

#define NUMBER_STATES 12         // all states
#define NUMBER_STATES_REDUCED 10 // states minus rotation around z axis
#define NUMBER_INPUTS 4          // input from solenoids
#define NUMBER_MEASUREMENTS 3    // magnetic field measurements in xyz directions

using namespace arma;

void increaseStateSpace(const vec *x, double x_pad[NUMBER_STATES]);
void reduceStateSpace(const double x_pad[NUMBER_STATES], vec x);
mat discretize_A(const mat A, const double dt);
mat discretize_B(const mat A, const mat Ad, const mat B);
mat van_loan(const mat A, const mat Q, const double dt);
