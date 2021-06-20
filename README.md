# Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

[pid]: ./doc/pid.png "piddiag"
[pideqn]: ./doc/eqn.svg "piddiageq"

## Design

### Controller
The PID controller is a very general purpose implementation of a controller which is used to implement controllers without knowing much of the plant dynamics.  
![p][pid]
The above diagram shows the control system containing a typical PID controller. The three components being `proportional`, `integral` and `derivative` controls. `Proportional` control does the work of acting on the error, whereas, `integral` accounts for plant non linearity and tries to minimize steady state error. `Derivative` applies control that requires quick maneuvers which helps in reducing the rise time and also compensating for overshoots.  

The project uses two PID controllers one for controlling the `throttle` and `brake` and one for controlling the `steering`.  

here is the mathematical expression showing the various elements.
![p][pideqn]  

As an initial setting the following parameters were used:  

1. Steering controller

    |Kp|Ki | Kd|
    |:---:|:---:|:---:|
    |0.135| 0.0008|1.0|  
2. Throttle valve controller  
    |Kp|Ki | Kd|
    |:---:|:---:|:---:|
    |0.6|0.0|1.0|  

### Twiddle Algorithm
Twiddle algorithm is a method of finding a better Kp Ki and Kd when the PID initial tuning values are not suitable to effectively complete the track. each iteration when the cross track error gets out of hand the twiddle algorithm is called which increments or decrements the gains of the controller in order to reduce the mean squared error which is computed by squaring the cross track error. This is actually the `reference - actual value` term that is squared.    

### Integrator design
While developing for an embedded platform it is important to have an integrator that is limited and also works effectively. 

#### moving window integrator
It is of limited size and the implementation here uses `std::deque`, a circular buffer to add at the back and remove elements at the front effectively creating a moving / sliding window. using `std::accumulate` the integral is calculated.
see `integrator.h` for more details.  

#### Integral windup 
If the integrator keeps on adding errors when the car is stuck/ takes time to return to the center, the integrator error would be very high by the time the car comes back to the center of the lane causing overshoots or worse loss of control. A limited integrator prevents `integral windup` in some rare cases where the aforementioned can happen.  

### Speed Control
the steering wheel angle is used to compute the desired speed. this is done by proportionally increasing or decreasing the car desired speed using the angle and the maximum angle at which the vehicle should brake heavily to restore control which is 25 degrees. refer to `main.h` for the empirical computation. 
