#include "include/matlab/maglevModel.h"
#include "observer.h"
#include "utilities.h"
#include <chrono>
#include <iostream>
#include <random>

using namespace std::chrono;


int main() {

    // Random number generator setup
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    initObserver();

    double meas[1][NUMBER_MEASUREMENTS] = {};
    double input[NUMBER_INPUTS] = {};
    double stateEstimates[NUMBER_STATES] = {};

    for (int i = 0; i < 4; ++i) {
        input[i] = dis(gen);
    }
    for (int i = 0; i < 3; ++i) {
        meas[0][i] = dis(gen);
    }

    while (1) {
        auto start = steady_clock::now();

        runObserver(input, meas, stateEstimates);
        // double x[12] = {};
        // double u[4] = {};
        // maglevModel_initialize();
        // maglevSystemDynamics_fast(x, u, stateEstimates);

        auto end = steady_clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        std::cout << duration.count() << endl;
        break;
    }
    // maglevModel_terminate();
    return 0;
}
