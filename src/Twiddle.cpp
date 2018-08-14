#include "Twiddle.h"
#include <limits>
#include <numeric>
#include <iostream>

/*
 * Construct a Twiddle object
 * @param is_active - flag signalizing whether we have to do the twiddle process actively
 * @param Kp - base/initial Kp coefficient
 * @param Ki - base/initial Ki coefficient
 * @param Kd - base/initial Kd coefficient
 * @param delta_Kp - initial delta Kp
 * @param delta_Ki - initial delta Ki
 * @param delta_Kd - initial delta Kd
 * @param max_steps - maximum number of twiddle steps for a single run
 * @param tolerance - tolerance limit for stopping the twiddle process
 */
Twiddle::Twiddle(bool is_active,
                 double Kp, double Ki, double Kd,
                 double delta_Kp, double delta_Ki, double delta_Kd,
                 unsigned int max_steps,
                 double tolerance) {

  this->is_active = is_active;
  this->max_steps = max_steps;
  this->tolerance = tolerance;

  // Everything begins now
  loop_count = 0;
  loop_start = 0;
  loop_error = 0.0;

  // Initializing the parameters...
  params.push_back(Kp);  // initial Kp
  params.push_back(Ki);  // initial Ki
  params.push_back(Kd);  // initial Kd

  // ...and the delta values
  delta_params.push_back(delta_Kp);  // initial delta Kp
  delta_params.push_back(delta_Ki);  // initial delta Ki
  delta_params.push_back(delta_Kd);  // initial delta Kd


  // Initializing the object for a new loop
  if ( is_active ) {

    // Debug message
    std::cout << "Twiddle is active. Target for SUM(dp): " << tolerance << std::endl << std::endl;

    // Executing the first step of the twiddle loop, actually
    best_error_so_far = std::numeric_limits<double>::max();
    current_idx = 0;
    params[current_idx] += delta_params[current_idx];
    parameter_increased = true;

    // Registering start time
    time(&twiddle_start);
  }
}


Twiddle::~Twiddle() {}


/**
 * Getter function for `is_active`.
 */
bool Twiddle::isActive() {
  return is_active;
}


/**
 * Start a whole new twiddle loop.
 */
void Twiddle::start(PID &pid) {

  double max_cte = pid.getMaxCTE();

  // (Re)initializing the PID controller
  pid.init(params[0], params[1], params[2]);

  if ( is_active ) {

    // Timing
    if ( loop_start != 0 ) {

      time_t now;
      time(&now);
      double loop_time = difftime(now, loop_start);
      std::cout << "Loop " << loop_count << " took " << loop_time << " seconds. Accumulated error: " << loop_error <<
                   " Max. CTE: " << max_cte << std::endl;
    }

    // Entering the next loop
    step_count = 0;
    loop_count++;
    time(&loop_start);

    // Debug output
    double delta_params_sum = std::accumulate(delta_params.begin(), delta_params.end(), 0.0);
    std::cout << "Starting loop " << loop_count <<
                 ".   [Kp, Ki, Kd]: [" << params[0] << ", " << params[1] << ", " << params[2] <<
                 "]    SUM(dp): " << delta_params_sum <<
                 "    Best error: " << best_error_so_far << std::endl;
  }
}



/**
* Processing the failure branch (or refactoring rulez).
*/
void Twiddle::processFailure() {

  if ( parameter_increased ) {
    // Increasing the current parameter didn't work; let's try the other way: decreasing the current parameter
    params[current_idx] -= 2 * delta_params[current_idx];

  } else {
    // Neither increasing nor decreasing the parameter worked. Resetting the current parameter to its original value,
    // then decreasing the current delta_params for the next run...
    params[current_idx] += delta_params[current_idx];
    delta_params[current_idx] *= (1 - DELTA_PARAM_CHANGE);

    // ...and progressing to the next parameter
    current_idx = (current_idx + 1) % 3;
    params[current_idx] += delta_params[current_idx];
  }

  // switching to the 'other' part of the twiddle algorithm (parameter decrease after an increase and vice versa)
  parameter_increased = !parameter_increased;
}



/**
 * Check twiddle status after the current update step.
 */
int Twiddle::check(PID &pid, double cte) {

  // Default return value
  int status = NOTHING_SPECIAL;

  // Storing the important metric
  loop_error = pid.getTotalError();


  // Breaking the current twiddle loop in case the current CTE is simply too big
  // (=the vehicle probably has driven off-track).
  if ( std::abs(cte) > CTE_LIMIT ) {

    std::cout << " *** CTE (" << cte << ") exceeds allowed limit; breaking/restarting twiddle loop." << std::endl;
    processFailure();

    return RESTART_LOOP;
  }

  // Watching as time goes by
  step_count++;

  // Have we just arrived to the end of a loop?
  if ( step_count > max_steps ) {

    // Checking whether we've reached the desired tolerance level
    double delta_params_sum = std::accumulate(delta_params.begin(), delta_params.end(), 0.0);
    if ( delta_params_sum <= tolerance ) {

      // That was it. https://media.giphy.com/media/8g63zqQ5RPt60/giphy.gif
      time_t now;
      time(&now);
      double loop_time = difftime(now, loop_start);

      std::cout << std::endl << "Twiddle finished in " << loop_time << " seconds." << std::endl;
      std::cout << "Best parameters:\n\tKp: " << best_params[0] <<
                                   "\n\tKi: " << best_params[1] <<
                                   "\n\tKd: " << best_params[2] << std::endl << std::endl;
      return FINISHED;
    }

    // Let's examine the resulting total error
    if ( loop_error < best_error_so_far ) {

      // The run resulted in a better performance than what we've seen so far; increasing delta parameter value
      best_error_so_far = loop_error;
      best_params = params;
      delta_params[current_idx] *= (1 + DELTA_PARAM_CHANGE);

      // Resetting the algorithm to the 'increasing' part and progressing to the next parameter
      current_idx = (current_idx + 1) % 3;
      params[current_idx] += delta_params[current_idx];
      parameter_increased = true;

    } else {
      processFailure();
    }

    status = RESTART_LOOP;
  }

  return status;
}
