# MPC Control
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Directories
The directory structure of this project is as follows:

```
.
|-- CMakeLists.txt
|-- README.md   
|-- src
    |-- json.hpp
    |-- main.cpp
    |-- MPC.cpp
    |-- MPC.h
```

## Project Instructions
The objective of this project is the implementation of a Model Predictive Control (MPC) in C++ that can drive a simulated car around the track using specific waypoints.

The simulated car's actuators have a 100ms latency (delay) that must be accounted for as well as part of the MPC calculation.

### Project Steps
* Fitting a line based on road waypoints and evaluating the current state based on that polynomial line.
* Implementing the MPC calculation, including setting variables and constraints
* Calculating actuator values from the MPC calc based on current state
* Accounting for latency (I used a predicted state 100ms in the future to replace the actual current state in the calculation)
* Calculating steering angle & throttle/brake based on the actuator values
* Setting timestep length and duration
* Testing/tuning of above implementations on Udacity simulator

## Discussion/Reflection
### The Model
My MPC model starts out by taking in certain information from the simulator:
* ptsx (x-position of waypoints ahead on the track in global coordinates)
* ptsy (y-position of waypoints ahead on the track in global coordinates)
* px   (current x-position of the vehicle's position in global coordinates)
* py   (current y-position of the vehicle's position in global coordinates)
* psi (current orientation angle of the vehicle, converted from the simulator's format to that expected in mathematical formulas)
* v (current velocity)
* delta (current steering angle of the car, i.e. where the wheels are turned, as opposed to the actual orientation of the car in the simulator at that point [psi])
* a (current throttle)

Update equations for the model:

```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t-1] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t-1] / Lf * dt
```

#### Polynomial Fitting & Preprocessing
I transform the points from the simulator's global coordinates into the vehicle's coordinates. First, each of the waypoints are adjusted by subtracting out px and py accordingly such that they are based on the vehicle's position. Next, the waypoint coordinates are changed using standard 2d vector transformation equations to be in vehicle coordinates:
```
MatrixXd tGlobalToLocal(double px, double py, double psi, const vector<double> & ptsx, const vector<double> & ptsy) {
  assert(ptsx.size() == ptsy.size());
  double x, y;
  auto points_l = MatrixXd(2, ptsx.size());
  for (unsigned int i = 0; i < ptsx.size() ; i++){
    x = ptsx[i] - px;
    y = ptsy[i] - py;
    points_l(0,i) = x * cos(-psi) - y * sin(-psi);
    points_l(1,i) = x * sin(-psi) + y * cos(-psi);
  }
  return points_l;
}
```

Next, we use the `polyfit ()` function to create a third-degree line that matches the transformed waypoints. This line indicates the trajectory of the vehicle.

Px, py and psi will be equal to zero since the center of the coordinate system is the vehicle itself, and it is always pointing to a zero orientation. The transverse error can be calculated by evaluating the polynomial function (`polyeval ()`) in px. The psi error, or epsi, is calculated from the derivative of the polynomial adjustment line, it is the negative arc tangent of the second coefficient.

```
auto coeffs = polyfit(Ptsx, Ptsy, 3);
double cte = polyeval(coeffs, 0);
double epsi = -atan(coeffs[1]);
```

## Timestep Length and Elapsed Duration (`N` & `dt`)
The prediction horizon `T` is the product of the timestep length `N` and elapsed duration `dt`. Timestep length refers to the number of timesteps in the horizon and elapsed duration is how much time elapses between each actuation.  
The prediction horizon I settled on was one second, with `N = 10` and `dt = .1`.  
With higher `N` value, if the vehicle "overshot" the reference trajectory, it would begin to oscillate wildly and drive off the track. With lower value of `N`, the vehicle may drive straight off the track.

### Back to The Simulator
Back in `main.cpp`, the first variable back from the `MPC.Solve()` function is delta. This value needs to be divided by deg2rad(25) to normalize it, as well as being multiplied by Lf in order to account for the vehicle's turning radius. The second value, "a", can be used directly as the throttle value. These are then sent back to the json model for the simulator to use.

```
auto vars = mpc.Solve(state, coeffs);
// Send values to the simulator
json msgJson;
msgJson["steering_angle"] = vars[0] / (deg2rad(25) * Lf);
msgJson["throttle"] = vars[1];
```

---

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
