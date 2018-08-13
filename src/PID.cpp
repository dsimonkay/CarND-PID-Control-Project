#include "PID.h"

#include <algorithm>
#include <iostream>
#include <math.h>


PID::PID() {}

PID::~PID() {}


void PID::Init(double Kp, double Ki, double Kd) {

  // Initializing pretty much everything
  total_error = 0.0;

  // Error terms
  p_error = i_error = d_error = 0.0;

  // The coefficients
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  // Current steering value
  steering = 0.0;
  first_update = true;
}


void PID::UpdateError(double cte) {

  // We set the differential error exceptionally to 0 for the first measurement.
  // For all other cases: p_error still holds the previous cross track error value at this point.
  d_error = first_update ? 0.0 : cte - p_error;

  // Now we can overwrite its value
  p_error = cte;

  // Simple case for the integral error
  i_error += cte;

  // Increasing total error
  total_error += std::abs(cte);

  // Never again 'first'
  first_update = false;
}


double PID::TotalError() {

  return total_error;
}



double PID::CalculateSteering(double cte) {

  // Calculating the raw steering angle in radians
  double angle = -Kp * p_error - Ki * i_error - Kd * d_error;

  // Constraining the angle between [-max_steering_angle, max_steering_angle]
  angle = std::max(angle, -max_angle);
  angle = std::min(angle, max_angle);

  // Normalizing the steering angle into the range [-1, 1] where the range limits correspond to -/+ 25 degrees.
  // As our steering value is still in radians, we use radians for the normalization as well.
  const double range_limit = M_PI * 25 / 180;
  double steering = angle / range_limit;

  // std::cout << "[speed: " << speed << "  angle: " << angle << " pE: " << p_error << "  iE: " << i_error << " dE: " << d_error << "] -- " << steering << std::endl;

  // Normalizing between [-1, 1]
  // steering_value /= max_steering_angle;

  // The higher the speed of the vehicle is, the more careful we do sudden movements on the steering wheel
  // steering_value *= (100 - speed) / 100.0;

  // double normalized_angle = angle / 25.0;
  // steering = (normalized_angle + steering) / 2.0;

  // pfff
  // steering = (this->steering + steering) / 2.0;

  this->steering = steering;

  return steering;
}


double PID::CalculateThrottle(double cte) {

  // Above this cte we choose `min_speed`
  const double limit = 2.0;
  const double cropped_cte = std::min(std::abs(cte), limit);

  // Simple linear proportionality
  double speed = min_speed + (max_speed - min_speed) * (1 - (cropped_cte / limit));

  // Normalizing between [0, 1]
  double throttle = speed / 100.0;

  return throttle;
}
