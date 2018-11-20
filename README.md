# CarND-Controls-MPC
This is Chuan's writeup report for Udacity self-driving car nano degree program term 2 project 4 Model Predictive Controller (MPC)

---
[//]: # (Image References)

[image1]: ./SystemStates.png "SystemStates"
[image2]: ./VehicleModelEquations.png "vehiclemodel"
[image3]: ./CTEEquations.png "CTEEquation"
[image4]: ./EpsiEquations.png "epsiEquation"

## MPC Introduction

* As explained in Udacity class, model predictive control is one kind of optimal control used in self-driving car to control the car to follow a reference trajectory.

* MPC mainly simulates different actuator inputs, predicts the resulting trajectory based on more accurate vehicle dynamic model, and then selects the optimized trajectory with a minimum cost function.

* MPC optimizes actuator inputs at each step in time to minimize the cost of predicted trajectory. Once the least cost trajectory is found, only the first set of actuation commands are implemented with the rest throwed away. Then new states are updated based on this optimized actuator inputs and used for next MPC control cycle. So MPC control inputs are constantly calculated and optimized over a future horizon, that is why MPC is also called **Receding Horizon Control**.


## MPC Implementation

The implementation of MPC in this project involves these parts:

* Define system states and actuator inputs
  
  The system states are decided based on critical states in vehicle dynamic models and also critical states for trajectory following. So in this project, vehicle x and y position, vehicle heading angle, vehicle speed from vehicle model and cross track error heading angle angle from trajctory following are considered a good combination of system states for MPC:
  
  ![alt text][image1]
  
  The actuator inputs are selected as vehicle steering angle input **delta** and vehicle accleration/deceleration **a**.
  
* Define system state equations

  After system states and actuator inputs are defined, then we need to relate them with system state equations for control problem. The system state equations contain also two part, one is from original vehicle dynamic model (bicycle model), another is for updating cross track error and heading angle error:
  
    ![alt text][image2]
    ![alt text][image3]
    ![alt text][image4]
    
  In MPC.cpp, the code for this part is identified as equality constrants as shown below:
  ```sh
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
  ```
  
* Define cost function components and actuator inequality constraints

  Cost function is the criteria used by MPC optimizer to select the optimized control and trajectory, the goal is to minimize the cost function with optimized control. Cost function component selection is very flexible and can be designed pretty well for realizing different control purposes. For example in this project, the main goal is to minize the cross track and heading angle error between real trajectory and reference one. So the main part of cost function should be that. And also in order to ensure smooth and stable vehicle control, actuator transition smoothness is also incorporated into cost function. This can shown in code in MPC.cpp here:

  ```sh
    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += 1500 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 1500 * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += 10 * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 100 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 10 * CppAD::pow(vars[a_start + t], 2);

      //This is to reduce speed at turns and increase on straight road
      fg[0] += 1000*CppAD::pow((vars[t + v_start] * vars[t + delta_start]), 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 50 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 50 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
  ```
  
* Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
