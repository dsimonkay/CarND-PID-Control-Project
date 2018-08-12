#ifndef PID_H
#define PID_H

// for portability of M_PI (Vis Studio, MinGW, etc.)
#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif

class PID {

  // This flag shows whether the current update step is the first one in the history of the controller.
  bool first_update;

  // Maximal allowed steering angle in radian and speed (in MPH).
  double max_angle;
  double max_speed;
  double min_speed;

  // Total squared error
  double total_squared_error;

  // Ss the name says: the value of the last calculated steering angle (in radians)
  double steering;

public:
  /*
   * Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /*
   * Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * Constructor
   */
  PID();

  /*
   * Destructor.
   */
  virtual ~PID();

  /*
   * Initialize PID.
   */
  void Init(double Kp, double Ki, double Kd);

  /*
   * Update the PID error variables given cross track error.
   */
  void UpdateError(double cte);

  /*
   * Calculate the total PID error.
   */
  double TotalError();

  /*
   * Calculate the steering value.
   * @param angle Current cross track error value
   */
  double CalculateSteering(double cte);

  /*
   * Calculate throttle.
   * @param angle Current cross track error value
   */
  double CalculateThrottle(double cte);
};

#endif /* PID_H */
