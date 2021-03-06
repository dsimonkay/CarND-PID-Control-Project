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
  const double max_angle = M_PI * 24.56 / 180.0;
  const double max_speed = 40;
  const double min_speed = 10;

  // Total (absolute) error
  double total_error;

  // As the name says: the value of the last calculated steering angle (in radians) and throttle
  double steering;
  double throttle;

  // Storing the CTE with the biggest magnitude
  double max_cte;

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
  void init(double Kp, double Ki, double Kd);

  /*
   * Update the PID error variables given cross track error.
   */
  void updateError(double cte);

  /*
   * Calculate the total PID error.
   */
  double getTotalError();

  /*
   * Getter function for `max_cte`
   */
  double getMaxCTE();

  /*
   * Calculate the steering value.
   * @param angle Current cross track error value
   */
  double calculateSteering(double cte);

  /*
   * Calculate throttle.
   * @param angle Current cross track error value
   */
  double calculateThrottle(double cte);
};

#endif /* PID_H */
