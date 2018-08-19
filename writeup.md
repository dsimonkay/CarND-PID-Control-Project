# PID Control Project

The goal of this project was to build a PID controller and tune its hyperparameters so that it can be applied to steer / drive the the vehicle of the Term 2 simulator around the lake.

The simulator delivers information over a local websocket cross-track error, providing the following values to the program:
- cross-track error (CTE),
- current steering angle (in degrees),
- current speed (in MPH).

The simulator expects a steering angle and a throttle value. Both values have to be normalized into the [-1, 1] range.


[//]: # (Image References)
[video1]: ./project_video_output.mp4

## [Rubric](https://review.udacity.com/#!/rubrics/824/view) Points:


### Compilation

#### Your code should compile.
The code compiles without errors or warnings. I've modified the originally provided `CMakeLists.txt`: I've added `src/Twiddle.cpp`  to the source set.


### Implementation

#### The PID procedure follows what was taught in the lessons.
The basic PID controller functions have been implemented in `PID.cpp`. Beside the functions `updateError()` and `totalError()` I have also implemented the functions `calculateSteering()` and `calculateThrottle()` to move the control logic from `main.cpp` into the PID object instance.


### Reflection

#### Describe the effect each of the P, I, D components had in your implementation. 
- The **P** component gives a response that is proportional to the current error. This component tries to steer the vehicle instantly towards the center line in proportion to the error. Using this component  only leads to an increasingly oscillating behaviour of the vehicle.

- The **I** component accounts for past values of the error and integrates them over time. It helps to counterweight the bias of error. The simulator vehicle has no built-in "bias" (drifting) but the curves present a sort of bias that has to be compensated.

- The **D** component tries to counterweight the effect of the **P** component, based on its current rate of change. As a result, the vehicle will get to the center line in a much smoother line and without oscillation (in the optimal case).


#### Describe how the final hyperparameters were chosen.
I have implemented the twiddle algorithm that has been presented in the lessons. As a "cycle" that ultimately yields a PID total error requires the vehicle to drive a full lap (which might take up to 100 seconds), running the whole process will take a lot of time.

In order not to begin the search for the parameters from scratch, I tried to find approximate values for `Kp`, `Ki` and `Kd` first.
At the beginning I set `Ki` and `Kd` to zero and then tried to find a value manually for `Kp` to get a more or less stable oscillation of the car (0.04). Then I tried to find -- also manually -- a value for `Kd` that made the car drive the full lap without leaving the track (1.4).

I set then `Ki` to a reasonably small value (0.001) and started the twiddle algorithm. The running time of the algorithm was really long, at least 12 hours. At the end I set the best found parameter values as the new base values and restarted the whole process with smaller parameter changes (e.g. I used the multiplier 1.01 instead of 1.1 when increasing a delta parameter value) and a smaller tolerance limit. I did this a few times.

Finally I checked the result (which wasn't too great) and tried to improve vehicle performance by manually tweaking the parameters. At the end I settled down at 0.0655369, 0.00199 and 1.29544 for `Kp`, `Ki` and `Kd`.


### Simulation

#### The vehicle must successfully drive a lap around the track.
It does so but it still oscillates too much. I suspect that some of "best" parameter values that I got happened to work by pure chance in the twiddle phase. I re-run the simulator later with the exact same parameters but I didn't receive the same "maxCTE" value that these parameters yielded during the twiddle process.

A possible solution would be to let the simulator run with the same parameters for a few laps (=a batch) and then use the averaged total error value for comparision. 