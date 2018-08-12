#include "PID.h"

using namespace std;

Twiddle::Twiddle(unsigned int number_of_steps, unsigned double tolerance) {

  this->number_of_steps = number_of_steps;
  this->tolerance = tolerance;
}

Twiddle::~Twiddle() {}


void Twiddle::Init(double Kp, double Ki, double Kd) {


}
