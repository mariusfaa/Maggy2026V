#include "observer.h"
#include "utilities.h"
#include <chrono>
#include <iostream>
#include <random>
#include <vector>

#include "include/matlab/maglevModel.h"

using namespace std::chrono;

int main() {
  // Number of iterations
  const int N = 10000;

  // Vector to store all runtimes
  std::vector<long long> runtimes;
  runtimes.reserve(N);

  // Random number generator setup
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-10.0, 10.0);

  initObserver();

  float meas[1][NUMBER_MEASUREMENTS] = {};
  float input[NUMBER_INPUTS] = {};
  double stateEstimates[NUMBER_STATES] = {};

  double x[12];
  double u[4];
  double y[1][3];

  std::cout << "Running " << N << " iterations..." << std::endl;

  // Perform N iterations
  for (int iteration = 0; iteration < N; ++iteration) {

    for (int i = 0; i < 12; ++i) {
      x[i] = dis(gen);
    }

    for (int i = 0; i < 4; ++i) {
      u[i] = dis(gen);
    }

    for (int i = 0; i < 3; ++i) {
      y[0][i] = dis(gen);
    }

    auto start = steady_clock::now();

    // maglevSystemDynamics_fast(x, u, stateEstimates);
    // maglevSystemMeasurements_fast(x, u, y);
    runObserver(u, y, stateEstimates);

    auto end = steady_clock::now();
    auto duration = duration_cast<microseconds>(end - start);

    runtimes.push_back(duration.count());
  }

  // Calculate statistics
  long long total_runtime = 0;
  long long worst_case = 0;

  for (const auto& runtime : runtimes) {
    total_runtime += runtime;
    if (runtime > worst_case) {
      worst_case = runtime;
    }
  }

  double average_runtime = static_cast<double>(total_runtime) / N;

  // Output results
  std::cout << "\n========== RESULTS ==========" << std::endl;
  std::cout << "Number of iterations: " << N << std::endl;
  std::cout << "Average runtime: " << average_runtime << " microseconds" << std::endl;
  std::cout << "Worst-case runtime: " << worst_case << " microseconds" << std::endl;
  std::cout << "Total runtime: " << total_runtime << " microseconds" << std::endl;
  std::cout << "=============================" << std::endl;

  return 0;
}
