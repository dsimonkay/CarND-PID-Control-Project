#include "PID.h"

#include <algorithm>
#include <iostream>
#include <math.h>


using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}


void PID::Init(double Kp, double Ki, double Kd) {

  // initializing pretty much everything
  total_squared_error = 0.0;

  // error terms
  p_error = i_error = d_error = 0.0;

  // the coefficients
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  max_angle = M_PI * 25.0 / 180.0;   // 25 degrees
  max_speed = 30.0;
  steering = 0.0;
  first_update = true;
}


void PID::UpdateError(double cte) {

  // we set the differential error exceptionally to 0 for the first measurement.
  // for the other cases: p_error still holds the previous cross track error value at this point.
  d_error = first_update ? 0.0 : cte - p_error;

  // now we can overwrite its value
  p_error = cte;

  // simple case for the integral error
  i_error += cte;

  // increasing total error
  total_squared_error += cte * cte;

  // never again 'first'
  first_update = false;
}


double PID::TotalError() {

  return total_squared_error;
}



double PID::CalculateSteering(double speed, double angle, double cte) {

  // calculating the raw steering angle in radians
  double steering = -Kp * p_error - Ki * i_error - Kd * d_error;

  // constraining the angle between [-max_steering_angle, max_steering_angle]
  steering = std::max(steering, -max_angle);
  steering = std::min(steering, max_angle);

  // normalizing the steering value into the range [-1, 1] where the range limits correspond to -/+ 25 degrees.
  // as our steering value is still in radians, we use radians for the normalization as well.
  const double range_limit = M_PI * 25 / 180;
  steering /= range_limit;

  // std::cout << "[speed: " << speed << "  angle: " << angle << " pE: " << p_error << "  iE: " << i_error << " dE: " << d_error << "] -- " << steering << std::endl;

  // normalizing between [-1, 1]
  // steering_value /= max_steering_angle;

  // the higher the speed of the vehicle is, the more careful we do sudden movements on the steering wheel
  // steering_value *= (100 - speed) / 100.0;

  // double normalized_angle = angle / 25.0;
  // steering = (normalized_angle + steering) / 2.0;

  // pfff
  // steering = (this->steering + steering) / 2.0;

  this->steering = steering;

  return steering;
}


double PID::CalculateThrottle(double speed, double angle, double cte) {

  return 0.3;

  const double cte_limit = 2.5;
  // normalizing between [0, 1]
  double cte_index = std::min(std::abs(cte), cte_limit) / cte_limit;

  double throttle = ((1 - cte_index) * max_speed + speed) / 200.0;
  std::cout << "[throttle] cte: " << cte << "   cte_idx: " << cte_index << "  throt: " << throttle << std::endl;
  return throttle;
}
