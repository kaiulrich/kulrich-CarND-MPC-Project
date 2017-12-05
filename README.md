# CarND-Controls-MPC

The purpose of the project was to implement a MPC (Model, Predictive and Control) controller and tune it so that the vehicle drives safely around the track.

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Rubric Tasks


### The Model

Model Predictive Control : Model predictive control is a process control technique which relies on process models that approximate the behavior and constraints of the process closely. The major advantage of MPC over PID is that it optimizes control at current timestep taking into account the future timesteps. To control the self-driving car we have to model the motion of the car. There are multiple models which define the movement of the car taking into account various factors that impact the cars motion. The more realistic the model, the higher is its complexity. To simplify the model, we introduce various approximations.

* This Kinematic model ignores parameters like tires forces, mass, gravity etc. 
* The kinematic model defines the vehicle state with its position (x, y), its velocity(v) and orientation(psi). 
* The next state of the car can be calculated using the following equations where 'delta' is the steering angle and 'a' is the throttle.
* MPC uses waypoints received from the simulator to calculate the expected trajectory. 
* The model uses actuator controls to generate the actual trajectory such that the crosstrack error(cte) and orientation error(epsi) of the car are minimized. These errors are also tracked as part of the vehicle's state.

~~~
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t-1] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t-1] / Lf * dt
~~~

### Timestep Length and Elapsed Duration (N & dt)

* The values chosen for N and dt originally were 10 and 0.1 respectively to first give stable performance at lower speeds.
* Once a benchmark performance was reached dt was reduced to 0.05 to allow for greater speeds. 
* A N value of 12 was chosen so as to keep the horizon T to an OK level, the values selected results in a T of 0.6s. 
* As the vehicle is travelling at higher speeds a lower T is more acceptable. It was found that when trying to increase N any further resulted in deteriorating performance.

### Model Predictive Control with Latency

The latency was mainly handled by predicting the next state based on the latency applied to the kinematic model as per the equations below. Note here that px, py and psi are all 0s as we have transformed about the center point of the car.

