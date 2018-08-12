#include "Twiddle.h"
#include <limits>

/*
 * Construct a Twiddle object
 * @param is_active - flag signalizing whether we have to do the twiddle process actively
 * @param Kp - base/initial Kp coefficient
 * @param Ki - base/initial Ki coefficient
 * @param Kd - base/initial Kd coefficient
 * @param max_steps - maximum number of twiddle steps for a single run
 */
Twiddle::Twiddle(bool is_active, double Kp, double Ki, double Kd, unsigned int max_steps) {

  this->is_active = is_active;
  this->max_steps = max_steps;

  // Initializing the parameters...
  params.push_back(Kp);  // initial Kp
  params.push_back(Ki);  // initial Ki
  params.push_back(Kd);  // initial Kd

  // ...then finally the delta values
  dp.push_back(0.005);  // initial delta Kp
  dp.push_back(0.001);  // initial delta Ki
  dp.push_back(0.5);    // initial delta Kd
}


Twiddle::~Twiddle() {}


/**
 * Wrapper function for `is_active`
 */
bool Twiddle::isActive() {

  return is_active;
}


/**
 * Start a whole new twiddle session.
 */
void Twiddle::Init(PID &pid) {

  step_count = 0;
  current_param_idx = 0;

  best_error_so_far = std::numeric_limits<double>::max();

  pid.Init(params[0], params[1], params[2]);
}


/**
 * Execute a twiddle step
 */
unsigned int Twiddle::doOneStep(PID &pid) {

  step_count++;
  if ( step_count > max_steps ) {

    double error = pid.TotalError();
    if ( error < best_error_so_far ) {
      
    }
  }

  // TO DO
  return DONE;
}
