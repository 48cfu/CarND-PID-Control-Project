#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  K = {Kp_, Ki_, Kd_};
  dp = {0.5, 0.5, 0.5};

  p_error = 0;
  i_error = 0;
  d_error = 0;
  previous_cte = 0;
  dt = 0.02;
  count = 0;
  previously_improved = true;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  p_error = cte;
  i_error += cte;
  d_error = (cte - previous_cte) / dt;
  previous_cte = cte; 

  count++;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  return p_error + i_error + d_error;  // TODO: Add your total error calc here!
}

double PID::Control(){
  
  double cte = p_error;
  if (cte * cte < previous_cte * previous_cte){ //improvement
    dp[(count - 1) % 3] *= 1.1;
    previously_improved = true;
  } else { // no improvement
    K[(count - 1) % 3] -= 2 * dp[(count - 1) % 3];
    if (!previously_improved){
      K[(count - 1) % 3] += dp[(count - 1) % 3];
      dp[(count - 1) % 3]*= 0.9;
    }
    previously_improved = false;
  }

  
  K[count % 3] += dp[count % 3];


  if (-(K[0] * p_error + K[1] * i_error + K[2] * d_error) < -1)
    return -1;
  else if (-(K[0] * p_error + K[1] * i_error + K[2] * d_error) > 1)
    return 1;
  else
    return -(K[0] * p_error + K[1] * i_error + K[2] * d_error);
}