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

  // Coefficients Kp, Ki and Kd
  std::vector<double> params;

  // Current delta values for Kp, Ki and Kd
  std::vector<double> delta_params;

  // Step size for parameter change (actually a multiplier when added to or subtracted from 1)
  constexpr double DELTA_DELTA_PARAMS = 0.02;

  // Nunber of runs / loops done by the algorithm
  unsigned int loop_count;

  // Step count in the current run
  unsigned int step_count;

  // As the name says
  time_t loop_start;

public:

  // Return status codes for the main twiddle-processing-step function (yeah, I know, how about enums.)
  static const int RESTART_LOOP = -1;
  static const int NOTHING_SPECIAL = 0;
  static const int FINISHED = 1;

  // If the CTE becomes ever bigger than the limit defined here, we'll break the current twiddle-session,
  // reset the simulator and start over the process.
  static constexpr double CTE_LIMIT = 3.0;

  /*
   * Constructor.
   * @param is_active -- flag signalizing whether the twiddle algoritm will be used
   * @param Kp - base/initial Kp coefficient
   * @param Ki - base/initial Ki coefficient
   * @param Kd - base/initial Kd coefficient
   * @param delta_Kp - initial delta Kp
   * @param delta_Ki - initial delta Ki
   * @param delta_Kd - initial delta Kd
   * @param max_steps -- number of maximum steps that a twiddle loop will consist of
   * @param tolerance -- tolerance level to be reached
   */
  Twiddle(bool is_active,
          double Kp, double Ki, double Kd,
          double delta_Kp = 0.01, double delta_Ki = 0.001, double delta_Kd = 0.5,
          unsigned int max_steps = 1100,
          double tolerance = 0.01);

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
  void Start(PID &pid);

  /**
   * Check twiddle status after the current update step.
   * @param pid -- PID controller instance
   * @param cte -- cross-track error
   */
  int Check(PID &pid, double cte);

  /**
   * Processing the failure branch (or refactoring rulez).
   */
  void ProcessFailure();


};

#endif /* TWIDDLE_H */
