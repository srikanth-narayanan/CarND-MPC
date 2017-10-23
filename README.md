# Model Predictive Control

This project involves in implementing a model predictive control to steer and accelerate the car around the track. The project incudes using the IPOPT and CPPAD libraries to find the optmised cost and minimise error. The solution will be robust to include a 100ms latency, so as to mimic the real world scenario.

The following are the steps involved in the bulding a MPC for controlling the vehicle.

- Choose appropriate N and dt.
- Evaluate the way points from the simulator and fit a polynomial.
- Define the cost functions using the actuators and state.
- Define the model state equations.
- Define the constraints for the model.
- Optmise the cost function with appropriate weights.
- Return back the optmised steering and throttle value.

## Vehicle Model

The vehicle mode consists of definition of the states and actuators. The following parameters define the vehicle model.

- vehicle's x position
- vehicle's y position
- vehicle's orientation psi
- vehicle's speed v
- distance from the desired position or the cross track error, cte
- orientation error to desired orientation, epsi

The model combines state and acutuations from the previous timestep to calculate the current timestep based on the equations below.

- x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
- y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
- psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
- v_[t+1] = v[t] + a[t] * dt
- cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
- epsi[t+1] = psi[t] - psides[t] + v[t] / Lf * delta[t] * dt

## Timestep and Elapsed Duration

The value of N and dt are choosen from the original MPC-line project 10 and 0.1 respectively. This value means the optmiser uses a one second worth of way points to correct the trajectory. The values were tried to be modified to 20 and 0.05 or 8 and 0.125. These mostly resulted in too many waypoints with a given time and resulted in erratic behaviour of the vehicle.

## Polynomial fitting and MPC preprocessing

The polynomial was fitted using the poyval function. A 3rd order poynomial is fitted to the waypoints. Prior fitting the polynomial, in order to simplyfy the process the waypoints are transformed to the vehicle co-ordinate system. This transformation involves centering the co-ordinates around car position x = 0, y = 0 and rotating counter clockwise by -psi as done line lines 109 to 128 in `main.cpp`.

## Model Predictive Control with Latency

The project recommended to consider a 100 millisecond latency to account for the real world scenarios. The acutal kinematic model depends on the acuations from the previous state, but with a delay of 100ms. In order to account for this delay, the equations are altered in lines 125 to 129 of `MPC.cpp`, where a previous state of acutator is considered at the end of the every 1 seonds cycle. Additional all the cost are penalised appopriately with an addition of combined velocity and steer in line 83 of `MPC.cpp`. This enables smooth steering and cornering of the vehicle.

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
