# CarND-Model Predictive Controller

Self-Driving Car Engineer Nanodegree Program

---

## Vehicle Model

A kinematic bicycle model is used in this project. All dynamic effects (tire forces, gravity, mass, etc) are neglected. The model is non-linear as changes in heading direction are taken into consideration.

Position (x,y), heading (ψ) and velocity (v) form the vehicle state vector: [x, y, ψ, v]

There are two actuators. Stearing angle (δ) is the first one, it should be in range [-25,25] deg. For simplicity the throttle and brake represented as a singular actuator (a), with negative values signifying braking and positive values signifying acceleration. It should be in range [-1,1]: [δ,a]

The following equations sestup the model used:

```text
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

x and y denote the position of the car, psi the heading, v its velocity, cte the cross-track error and epsi the orientation error. Lf is the distance between the center of mass of the vehicle and the front wheels. The vehicle model can be found in the class FG_eval.

## Timestep Length and Elapsed Duration (N & dt)

The time T=N dt defines the prediction horizon. Short prediction horizons lead to more responsive controllers, but are less accurate and can suffer from instabilities when chosen too short. Long prediction horizons generally lead to smoother controls. For a given prediction horizon shorter time steps dt imply more accurate controls but also require a larger MPC problem to be solved, thus increasing latency.

Here values of N and dt were chosen such that the car drives smoothly around the track for slow velocities of about 25mph all the way up to about 100mph. The values are N=15 and dt=0.1. This means the model predicts for 1.5 seconds.

## Polynomial Fitting and MPC Preprocessing

The waypoints from the simulator are transformed into the vehicle coordinate system. 

```math
ptsx_car[i] = (ptsx[i] - px) * cos(-psi) - (ptsy[i] - py) * sin(-psi);
ptsy_car[i] = (ptsx[i] - px) * sin(-psi) + (ptsy[i] - py) * cos(-psi);
```

px and py are subtracted from the waypoints to be relative to vehicle position.

Using the polyfit() function, a third-degree polynomial is fitted to the transformed waypoints, representing the path the vehicle should try to travel. Here, px, py and psi are zero as the vehicle is the center of the coordinate system, and it is always pointing to a zero orientation. The cross-track error is then calculated by evaluating the polynomial function (polyeval()) at px. The psi error, or epsi, which is calculated from the derivative of polynomial fit line, is therefore simpler to calculate, as polynomials above the first order in the original equation are all eliminated since x is zero.

## Latency

A 100 ms latency exists between the actuator calculation (when the model tells the car to perform a steering or acceleration/braking change) and when the simulator actually performs instructed action.

To implement this, a step to predict where the vehicle would be after 100ms was added. This accounts for the motion during the latency period, instead of the one in reaction to an old situation. The "dt" value here (not same as in MPC.cpp) is set equal to the latency. Then, using the same update equations as those used in the actual MPC model, the state is predicted and fed into the model. Note that these equations were able to be simplified again because of the coordinate system transformation - using x, y and psi all of zero made these equations a little simpler, as lots of the values end up being zero or one. This new predicted state, along with the coefficients, are then fed into the mpc.Solve() function in MPC.cpp.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((<https://developer.apple.com/xcode/features/)>
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.

    ```text
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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps (not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## References

* <https://medium.com/self-driving-cars/five-different-udacity-student-controllers-4b13cc8f2d0>
* <https://medium.com/@NickHortovanyi/carnd-controls-mpc-2f456ce658f>
* <https://towardsdatascience.com/how-self-driving-cars-steer-c8e4b5b55d7f>
* <https://github.com/mvirgo/MPC-Project>