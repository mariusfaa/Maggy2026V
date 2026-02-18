#pragma once

#include "../BasicLinearAlgebra/BasicLinearAlgebra.h"

#define NUMBER_STATES 12              // all states
#define NUMBER_STATES_REDUCED 10      // states minus rotation around z axis
#define NUMBER_INPUTS 4               // input from solenoids
#define NUMBER_MEASUREMENTS 3         // magnetic field measurements in xyz directions

// matrices
#define A_type BLA::Matrix<NUMBER_STATES_REDUCED, NUMBER_STATES_REDUCED, double>
#define B_type BLA::Matrix<NUMBER_STATES_REDUCED, NUMBER_INPUTS, double>
#define H_type BLA::Matrix<NUMBER_MEASUREMENTS, NUMBER_STATES_REDUCED, double>
#define R_type BLA::Matrix<NUMBER_MEASUREMENTS, NUMBER_MEASUREMENTS, double>
#define Q_type A_type
#define K_type BLA::Matrix<NUMBER_STATES_REDUCED, NUMBER_MEASUREMENTS, double>

// vectors
#define StateVector BLA::Matrix<NUMBER_STATES_REDUCED, 1, double>
#define StateVectorFull BLA::Matrix<NUMBER_STATES, 1, double>
#define InputVector BLA::Matrix<NUMBER_INPUTS, 1, double>
#define MeasVector BLA::Matrix<NUMBER_MEASUREMENTS, 1, double>

// prototypes
void increaseStateSpace(const StateVector& x, double x_pad[NUMBER_STATES]);
void reduceStateSpace(const double x_pad[NUMBER_STATES], StateVector& x);
