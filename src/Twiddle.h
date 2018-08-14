#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "PID.h"
#include <vector>
#include <time.h>

/**
 * An instance of this class is responsible for tuning/determining the parameters Kp, Ki and Kd
 * of the PID controller for the self driving car of the Udacity Term 2 simulator.
 */
class Twiddle {

  // Should we do anything at all?
  bool is_active;

  // Number of maximum steps
  unsigned int max_steps;

  // limit for stopping the twiddle process
  double tolerance;

  // Best error so far
  double best_error_so_far;

  // Index of the currently tuned/changed parameter index
  size_t current_idx;

  // Are we probing an increased or decreased parameter value?
  bool parameter_increased;

  // Coefficients Kp, Ki and Kd currently used by the PID controller
  std::vector<double> params;

  // Best coefficients Kp, Ki and Kd
  std::vector<double> best_params;

  // Current delta values for Kp, Ki and Kd we've found so far
  std::vector<double> delta_params;

  // Step size for parameter change. It will used as a multiplier for increasing/decreasing
  // the value of a parameter in the form of adding to or subtracting from 1.
  const double DELTA_PARAM_CHANGE = 0.02;

  // Nunber of runs / loops done by the algorithm
  unsigned int loop_count;

  // Step count in the current run
  unsigned int step_count;

  // Total error of the current loop
  double loop_error;

  // As the names say
  time_t twiddle_start;
  time_t loop_start;

public:

  // Return status codes for the main twiddle-processing-step function
  static const int RESTART_LOOP = -1;
  static const int NOTHING_SPECIAL = 0;
  static const int FINISHED = 1;

  // If the CTE ever becomes bigger than the limit defined here, we'll break the current twiddle-session,
  // reset the simulator and start over the loop.
  static constexpr double CTE_LIMIT = 2.85;

  /*
   * Constructor.
   * @param is_active -- flag signalizing whether the twiddle algoritm will be used
   * @param Kp - initial Kp coefficient
   * @param Ki - initial Ki coefficient
   * @param Kd - initial Kd coefficient
   * @param delta_Kp - initial delta Kp
   * @param delta_Ki - initial delta Ki
   * @param delta_Kd - initial delta Kd
   * @param max_steps -- number of maximum steps that a twiddle loop will consist of.
   *                     guess what: the number depends on the maximum speed value.
   * @param tolerance -- tolerance level to be reached
   */
  Twiddle(bool is_active,
          double Kp, double Ki, double Kd,
          double delta_Kp = 0.02,
          double delta_Ki = 0.0005,
          double delta_Kd = 0.3,
          unsigned int max_steps = 1100,
          double tolerance = 0.001);

  /*
   * Destructor.
   */
  virtual ~Twiddle();

  /**
   * Getter function for `is_active`.
   */
  bool isActive();

  /**
   * Start a whole new twiddle loop.
   * @param pid -- PID controller instance
   */
  void startLoop(PID &pid);

  /**
   * Doing minor administrative tasks at the end of a twiddle loop. 
   * @param pid -- PID controller instance
   */
  void endLoop(PID &pid);

  /**
   * Check twiddle status after the current update step.
   * @param pid -- PID controller instance
   * @param cte -- cross-track error
   */
  int check(PID &pid, double cte);

  /**
   * Processing the failure branch (or refactoring rulez).
   */
  void processFailure();

};

#endif /* TWIDDLE_H */
