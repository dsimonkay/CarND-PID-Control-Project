#ifndef PID_H
#define PID_H

class Twiddle {

  /**
   * An instance of this class is responsible for tuning/determining the parameters Kp, Ki and Kd
   * for the self driving car of the Udacity Term 2 simulator.
   */

  unsigned int number_of_steps;
  unsigned double tolerance;


public:

  const unsigned int DONE = 0;
  const unsigned int FINISHED = 1;

  /*
   * Constructor
   */
  Twiddle(unsigned int number_of_steps = 1100, unsigned double tolerance = 0.001);

  /*
   * Destructor.
   */
  virtual ~Twiddle();

  unsigned int DoTheTwist(double cte);
};

#endif /* PID_H */
