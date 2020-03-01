#ifndef PID_H
#define PID_H
#include <iostream>
#include <vector>
#include <chrono>
#include <ctime>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  double Control();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  /**
   * Params for derivative error
   */
  double previous_cte;
  double dt;

  /**
   * PID Coefficients
   */ 
  std::vector<double> K;

  std::chrono::time_point<std::chrono::system_clock> start;
};

#endif  // PID_H