#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "PID.h"
#include <vector>

/**
 * An instance of this class is responsible for tuning/determining the parameters Kp, Ki and Kd
 * of the PID controller for the self driving car of the Udacity Term 2 simulator.
 */
class Twiddle {

  // Should we do anything at all?
  bool is_active;

  // Number of maximum steps
  unsigned int max_steps;

  // Best error so far
  double best_error_so_far;

  // Current delta values for Kp, Ki and Kd
  std::vector<double> dp;

  // Index of the 
  size_t current_param_idx;

  // Step size for parameter change (actually a multiplier when added to or subtracted from 1)
  const double DELTA_DP = 0.01;

  // Coefficients
  std::vector<double> params;

public:

  // Current step count
  unsigned int step_count;

  // Return status codes for the main twiddle-processing-step function (yeah, I know, how about enums.)
  static const unsigned int DONE = 0;
  static const unsigned int FINISHED = 1;
  static const unsigned int CTE_LIMIT_BREACH = 2;

  // If the CTE becomes ever bigger than the limit defined here, we'll break the current twiddle-session,
  // reset the simulator and start over the process.
  static constexpr double CTE_LIMIT = 3.0;

  /*
   * Constructor
   */
  Twiddle(bool is_active,
          double Kp, double Ki, double Kd,
          unsigned int max_steps = 1100);

  /*
   * Destructor
   */
  virtual ~Twiddle();

  /**
   * Wrapper function for `is_active`
   */
  bool isActive();

  /**
   * Start a whole new twiddle cycle.
   */
  void Init(PID &pid);

  /**
   * Execute a twiddle step
   */
  unsigned int doOneStep(PID &pid);

};

#endif /* TWIDDLE_H */
