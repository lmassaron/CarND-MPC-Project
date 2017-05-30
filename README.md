# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## The Model
Model Predictive Control (MPC) is a controlling method that can anticipate future events and elaborate control actions accordingly, showing more responsivity and stability than a PID controller.

As reflections, after implementing this model for a car running at 40 mph in a round circuit around a lake, I can summarize:

* MPC can handle latency very well, incorporating it into its predictions, something that a PID can only answer in a reactive fashion, not an anticipatory one.
* MPC is computationally intensive depending on the steps in the future you need to look forward and the actuation reactivity. Higher speeds require mtheore computations and that can cause problems if the hardware is not suitable.
* MPC can control both steering and throttle, no need of anything else (Using PID that would require two PIDs)
* I used the standard cost function taken from the lectures but the cost function can be easily customized weighting some parts of it more of others, thus making a MPC suitable for specific routes or situation. Thus I found that increasing certain costs helped the car stay on tack even at higher speeds as described below in "An empirical solution" section.

### State
The vehicle state is described by the following variables:

    * x: position of the vehicle in the forward direction
    * y: position of the vehicle in the lateral direction
    * psi: yaw angle or orientation of the vehicle
    * v: speed of the vehicle

The state is completed by its evaluation expressed by:
    * cross track error (cte) which is the distance between the car's position and the lane center
    * heading error (epsi) which is the radians difference between the tangent of the closest point of the lane center and the car forward direction

### Actuators
The actuators of the vehicle are:

    * delta: steering angle expressed in radians
    * alpha: throttle input, determining acceleration

### MPC algorithm

Setup:

    1. Define the length of the trajectory, N, and duration of each timestep, dt.
    2. Define vehicle dynamics and actuator limitations along with other constraints.
    3. Define the cost function.

Loop:

    4. We pass the current state as the initial state to the model predictive controller.
    5. We call the optimization solver. Given the initial state, the solver will return the vector of control inputs that minimizes the cost function. The solver we'll use is called [Ipopt](https://projects.coin-or.org/Ipopt).
    6. We apply the first control input to the vehicle.
    7. Back to 4.

When solving for control inputs I've found that a third degree polynomial is enough and it provides realistic trajectories.

In order to represent the vehicle, the simplified kinematic bicycle model is used in this implementation of MPC:
In such model, next state is calculated as:
 - x_t1   = x_t + v_t * cos(psi_t) * dt
 - y_t1   = y_t + v_t * sin(psi_t) * dt
 - psi_t1 = psi_t * v_t / Lf * delta * dt
 - v_t1   = v_t + a_t * dt


## Timestep lenght (N) and Elapsed Duration (dt)

The prediction horizon is the duration over which future predictions are made. We’ll refer to this as T.
T is the product of two other variables, Time step (N parameter) and elapsed duration (dt parameter).
N is the number of timesteps in the horizon. dt is how much time elapses between actuations.
N and dt are inversely related and general guidelines indicate that T should be as large as possible, while dt should be as small as possible.

In fact, T should be a few seconds, at most since beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future. 

### N
N is the number of variables optimized by MPC, the more, the more precise the forecast yet the heavier the computations will be.

### dt
dt points out the frequency of actuations. Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory.

### An empirical solution
A good practice is to first determine a reasonable range for T and then tune dt and N appropriately, keeping the effect of each in mind. By various experiments I found out that it is better to look adhead for about 0.75 seconds, which can be splitted in N=15 (still a fair computational effort) and dt = 0.05 (50 milliseconds) which allows a good steering reaction, even at as high speed as 90 mph. After fixing N and dt, I found that I could have the car deal with higher speeds by putting some extra penalties in the cost function:

* a penalty on the steering and steering sequence which is calculated by a polynomial function on the basis of the actual speed of the car. The polynomial formula has been found by figuring out the best value at different speeds (40, 50, 60, 70, 80, 90 mph) and then interpolating the resulting penalties. Using a function, instead of a branching of if/then, helps speeding up computations.

* a penalty in using the brake/throttle which proved very useful in avoiding the car to brake too much at lower speeds.

## Waypoints, Vehicle State, Actuators Preprocessing

The only preprocessing done is transforming the waypoints coordinates in respect of the car's coordinate system (which is based on the car itself, having the car as the origin).

## Dealing with Latency
In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds. 

A latency of 100 milliseconds has been implemented in the code of `main.cpp`. Such a small delay causes the first, immediate, prediction made by the solver in the MPC to be already superseeded when actually implemented. In fact, the first prediction is expected to be actual after 50 milliseconds, but, due to the latency, is actuated after 150 milliseconds.

The suggested action on steering and throttle could then prove unsuitable for the present situation because based on a state situation which is not actual. That could increase the CTE and orientation error, calling for furthermore actions that, as the previous ones cannot be timely, causing more and more errors. In the end, that will make the vehicle oscillating around the intended trajectory causing it to get off-road at higher speeds.

As a solution I combined:

* Using the actual speed and car's position, I try to predict its position after the latency in order to feed into the MPC solver a more likely position to be evaluated. This results in an anticipation of the latency effects, making the first prediction of the solver as actual for the car's state.
 
* Since there are N predictions, we can take a few ones, for instance the first five, average them, and return a solution for MPC steering and throttle actuaction values which is incorporating a future state well beyond the 100 milliseconds latency (five predictions, with dt=0.05, contain information on about 250 milliseconds)

Please note that in the current implementation actuator dynamics have not been taken into account (because they have not been measured), but they could be easily dealt with because of the averaging of the first three solver's predictions.


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * This [version](https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip) works out of the box,
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.7`.
    * In case errors are reported while loading shared libraries, just run this command before the ipopt.sh: `sudo ldconfig`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
  * Linux tip for running the simulator as an executable. Run this command in the simulator's directory: `chmod +x term2_sim.x86*`
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Websocket Data

The JSON object send back from the simulator command server consists of the following fields:

* `ptsx` (Array<float>) - The global x positions of the waypoints.
* `ptsy` (Array<float>) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity
since y is the up-down direction.
* `psi` (float) - The orientation of the vehicle in **radians** converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
* `psi_unity` (float) - The orientation of the vehicle in **radians**. This is an orientation commonly used in [navigation](https://en.wikipedia.org/wiki/Polar_coordinate_system#Position_and_navigation).
* `x` (float) - The global x position of the vehicle.
* `y` (float) - The global y position of the vehicle.
* `steering_angle` (float) - The current steering angle in **radians**.
* `throttle` (float) - The current throttle value [-1, 1].
* `speed` (float) - The current velocity in **mph**.


### `psi` and `psi_unity` representations

`psi`

```
//            90
//
//  180                   0/360
//
//            270
```


`psi_unity`

```
//            0/360
//
//  270                   90
//
//            180


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
5. Start the simulator on autonomous mode for mpc

or simply:

1. ./build.sh
2. Start the simulator on autonomous mode for mpc

