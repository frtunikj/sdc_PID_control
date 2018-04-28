# PID Control Project

[//]: # (Image References)
[image0]: ./docs/PID.png

This project implementes a PID controler in C++. The Udacity code basis for the project can be find [here](https://github.com/udacity/CarND-PID-Control-Project). Only the `PID.cpp` and `main.cpp` files have been modified. For each time step we are able to sense the speed, steering angle, and cross track error. The cross track error (CTE) is the error of the ego vehicle from the desired position on the track. The final  parameters of the PID controller i.e. Kp, Kd and Ki were chosen manually through a trial and error process and observation of the effects. In case of too much oscillation of the ego vehicle either the proportional gain was reduced or the derivative gain was increased until it behaves well. The integral term was set to close to zero since it seems that the Udacity simulator does not have any systematic bias. Below is an explanation of the P, I and D parameters of an PID controller and their effect:

* P (proportional control)

    The Proportional control gain generates a control signal in direct proportion to the cross-track-error (CTE), in such a way that the CTE is reduced. If this parameter is too large the tendency is to overshoot and miss the desired output. If it's too small, it will take a longer time to get to the desired set point. Using a P-control gain of Kp = 1.0 at the beginning leads to the car rapidly oscillating about the center of the road. Thus, gradually the P-control gain was reduced down to 0.1 such that the car manages to stay on the road much longer.

* D (derivative control)

    The Derivative control gain considers the rate of change in the error. If the error is rapidly approaching zero, this parameter will attempt to slow things down to avoid overshooting. If this is too large, then there will be overdamping which means it will take longer to reach the desired goal. The D-control gain is helpfull in the situations when the ego vehicle starts turning. As the vehicle turns, the CTE increases and as a result P-controll command incerases as well. However, the D-control gain damps the P-control command as the car gets closer to the reference trajectory i.e. center of the road.

* I (integral control)

    The Integral control gain is usually used for compensation of systematic bias in the system. An examaple for a systematic bias is e.g. when the front wheels are not oriented exactly straight for a steering angle of 0 degrees. It seems that the Udacity simulator does not have a systematic bias and thus this paramter was set close to zero.
    
NOTE: I experienced problems with the simulator when recording videos thus no example videos are provided.

The result of the localization is depected on the figure below.

![alt text][image0]

## Running the Code

### Dependencies

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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

### Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

### Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).



