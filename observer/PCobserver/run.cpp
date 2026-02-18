#include "observer.h"
#include "utilities.h"
#include <chrono>
#include <iostream>

using namespace std::chrono;


int main() {

  initObserver();

  float meas[1][NUMBER_MEASUREMENTS] = {};
  float input[NUMBER_INPUTS] = {};
  double stateEstimates[NUMBER_STATES] = {};


  while (1) {

    if (newSensorReading) {
      auto start = steady_clock::now();

      runObserver(input, meas, stateEstimates);

      auto end = steady_clock::now();
      auto duration = duration_cast<microseconds>(end - start);
      std::cout << duration.count() << endl;
    }
  }

  return 0;
}
