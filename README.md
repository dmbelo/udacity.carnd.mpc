# udacity.carnd.mpc
Udacity Self-Driving Car Nanodegree Term 2 Project 5: Model Predictive Control

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Model Description 

The controller implemented relies on a kinematic model that contains the following states and actuator commands

|State|Units|Min|Max|Description|
|---|---|---|---|---|
|x_car|m|-inf|inf|Vehicle x-position in global coordinate system|
|y_car|m|-inf|inf|Vehicle y-position in global coordinate system|
|psi_car|rad|-inf|inf|Vehichle heading angle|
|v_car|m/s|-inf|inf|Vehicle velocity in heading direction|
|cte_car|m|-inf|inf|Cross tracking error|
|err_psi_car|rad|-inf|inf|Heading angle error|

|Actuator|Units|Min|Max|Description|
|---|---|---|---|---|
|a_steer|rad|-0.436|0.436|Steering wheel angle command|
|r_throttle|-|-1|1|Non-dimensional control signal for forward/reverse acceleration command|

The kinematic equations that govern the relationships of the states and actuators are implemented as optimization constraints that need to be satisfied and thus are taken into account when solving for the optimum actuator commands for a given scenario.

The controller considers a discretized future time horizon of finite length. This is the domain over which the optimization problem is cast. Therefore, the time horizon should be long enough to take into account a suitable amount of features (upcoming turns in this example) that are desirable to be considered for the optimization of steering wheel and acceleration demand controls. If the horizon is too long, the computational expense will increase proportionally while the controller improvements will be increasingly diminished. It was found that a good practical length was 1 second.

The number of discrete points used for the optimization problem is another design parameter. The number of points is directly proportional to computational expense and related to simulation accuracy. Therefore we'd like to choose a high enough number of points that doesn't results in discritezation error, but not too high that it slows down the processing demands of the controller for the target hardware. In this case 10 points were found to be sufficient.

### Waypoints

At each step, the simulator provides the controller with desired trajectory waypoints. These are coordinates that defined the desired trajectory that might be calculated by the path planning system on board. To turn the set of finite points into a continuous function to be used by the optimization process, a 3rd order polynomial is fit to the waypoints to described the trajectory with a continuous and differentiable function. Note that before this is done, the waypoints are first converted from the map coordinate system into the vehicle's coordinate system. 

## Latency

In pratice, the the latency in the autonomy stack will significantly affect the control performance of the vehicle. To improve the instability induced by this latency, the expected time delay is taken into account in the optimization problem. This is accomplished by simulating the vehicle's trajectory during the delay before the optimization. By doing this, we take into account the fact that by the time the vehicle experiences the optimized control inputs it will have been in a different state as compared to when the controller read them.
