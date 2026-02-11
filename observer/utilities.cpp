
#include "utilities.h"

// Pad unobservable states with 0
void increaseStateSpace(const StateVector& x, double x_pad[NUMBER_STATES]) {
  for (int i = 0; i < NUMBER_STATES; i++) {
    if (i == 5 || i == 11) {
      x_pad[i] = 0;
    } else {
      x_pad[i] = x(i);
    }
  }
}

// Pop unobservable states
void reduceStateSpace(const double x_pad[NUMBER_STATES], StateVector& x) {
  int offset = 0;
  for (int i = 0; i < NUMBER_STATES; i++) {
    if (i == 5 || i == 11) {
      offset = 1;
    } else {
      x(i - offset) = x_pad[i];
    }
  }
}
