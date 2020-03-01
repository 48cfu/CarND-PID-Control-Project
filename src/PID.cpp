#include "PID.h"
#include <algorithm>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  start = std::chrono::system_clock::now();
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  K = {Kp_, Ki_, Kd_};

  p_error = 0;
  i_error = 0;
  d_error = 0;
  previous_cte = 0;
  dt = 1;

}

void PID::UpdateError(double cte) {
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  //dt = elapsed_seconds.count();

  /**
   * Update PID errors based on cte.
   */
  
  p_error = cte;
  i_error += cte * dt;
  d_error = (cte - previous_cte) / dt;
  previous_cte = cte; 
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  return p_error + i_error + d_error;  // TODO: Add your total error calc here!
}

double PID::Control(){
  double to_return = -(K[0] * p_error + K[1] * i_error + K[2] * d_error);
  // saturator
  return std::min({1.0, std::max({-1.0, to_return})});
}