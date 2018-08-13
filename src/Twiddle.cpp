#include "Twiddle.h"
#include <limits>
#include <numeric>

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

  // Initializing the parameters...
  params.push_back(Kp);  // initial Kp
  params.push_back(Ki);  // initial Ki
  params.push_back(Kd);  // initial Kd

  // ...then finally the delta values
  delta_params.push_back(delta_Kp);  // initial delta Kp
  delta_params.push_back(delta_Ki);  // initial delta Ki
  delta_params.push_back(delta_Kd);  // initial delta Kd


  // Initializing the object for a new loop
  if ( is_active ) {

    // Executing the first step of the twiddle loop, actually
    step_count = 0;
    best_error_so_far = std::numeric_limits<double>::max();
    current_idx = 0;
    params[current_idx] += delta_params[current_idx];
    parameter_increased = true;
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
void Twiddle::Start(PID &pid) {

  // (Re)initializing the PID controller
  pid.Init(params[0], params[1], params[2]);

  if ( is_active ) {

    // Timing
    if ( loop_start != 0 ) {

      time_t now;
      time(&now);
      double loop_time = difftime(now, loop_start);
      std::cout << "Last loop took " << loop_time << " seconds." << std::endl;
    }

    // Entering the next loop
    loop_count++;
    time(&loop_start);

    // Debug output
    double delta_params_sum = std::accumulate(delta_params.begin(), delta_params.end(), 0.0);
    std::cout << "[" << loop_count << "] Starting loop. Kp: " << params[0] << ",  Ki: " << params[1] << "  Kd: " << params[2] << "   sum(dp): " << delta_params_sum << std::endl;
  }
}



/**
* Processing the failure branch (or refactoring rulez).
*/
void ProcessFailure() {

  if ( parameter_increased ) {
    // Increasing the current parameter didn't work; let's try the other way: decreasing the current parameter
    params[current_idx] -= 2 * delta_params[current_idx];

  } else {
    // Neither increasing nor decreasing the parameter worked. Resetting the current parameter to its original value,
    // then decreasing the current delta_params for the next run...
    params[current_idx] += delta_params[current_idx]
    delta_params[current_idx] *= (1 - DELTA_DELTA_PARAMS)

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
int Twiddle::Check(PID &pid, double cte) {

  // default return value
  int status = NOTHING_SPECIAL;


  // breaking the current twiddle loop in case the current CTE is simply too big (=the vehicle probably drove off-track)
  if ( std::abs(cte) > Twiddle::CTE_LIMIT ) {

    std::cout << " ** CTE (" << cte << ") over limit; finishing twiddle cycle." << std::endl;
    ProcessFailure();

    return RESTART_LOOP;
  }

  // Watching as time goes by
  step_count++;

  // Have we just arrived at the end of a loop?
  if ( step_count > max_steps ) {

    // Checking whether we've reached the desired tolerance level
    double delta_params_sum = std::accumulate(delta_params.begin(), delta_params.end(), 0.0);
    if ( delta_params_sum <= tolerance ) {

      // That was it. https://media.giphy.com/media/8g63zqQ5RPt60/giphy.gif
      std::cout << std::endl << "Twiddle finished in " << loop_count << " runs." << std::endl;
      std::cout << "Final parameters:\n\tKp: " << params[0] << "\n\tKi: " << params[1] << "\n\tKd: " << params[2] << std::endl << std::endl;

      return FINISHED;
    }

    // Let's examine the resulting total error
    double error = pid.TotalError();
    if ( error < best_error_so_far ) {

      // The run resulted in a better performance than what we've seen so far; increasing delta parameter value
      best_error_so_far = error;
      delta_params[current_idx] *= (1 + DELTA_DELTA_PARAMS);

      // Resetting the algorithm to the 'increasing' part and progressing to the next parameter
      current_idx = (current_idx + 1) % 3;
      params[current_idx] += delta_params[current_idx];
      parameter_increased = true;

    } else {
      ProcessFailure();
    }

    status = RESTART_LOOP;
  }

  return status;
}
