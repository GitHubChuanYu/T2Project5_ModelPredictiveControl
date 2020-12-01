# CarND-Controls-MPC
This is Chuan's writeup report for Udacity self-driving car nano degree program term 2 project 5 Model Predictive Controller (MPC)

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
  
  The system states are decided based on critical states in vehicle dynamic models and also critical states for trajectory following. So in this project, vehicle x and y position, vehicle heading angle, vehicle speed from vehicle model, cross track error, and vehicle orientation error are considered a good combination of system states for MPC:
  
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
  
* Calling MPC solver (CppAD::IPOPT)  with above defined inputs to get optimized solution including states and actuator inputs:

```sh
  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
```

## MPC Preprocessing and Polynomial Fitting

As mentioned above, MPC is used to control car to follow a reference trajectory. So MPC requires a reference trajectory to calculate the cross track error and heading angle error for cost function. However, in this project, the simulator only sends waypoints along the reference trajectory. Some some preprocessing of waypoints are needed for MPC. 

1. As suggested in **Tips and Tricks** part of the MPC project, I did a transform of waypoints from map's coordinate system to car's coordinate system so that they are easier for displaying and also calculating CTE and Epsi values for MPC. The detailed theory for this transform can be found in this [link](https://www.miniphysics.com/coordinate-transformation-under-rotation.html). And the code for doing this transformation is:
```sh
          //Remap waypoints from map's coordinate system to car's coordinate system
          for (int i = 0; i < ptsx.size(); i++) {
              //x and y difference in map's coordinate system
              double xdiff = ptsx[i] - px_new;
              double ydiff = ptsy[i] - py_new;
              
              //Remap assuming car always start from origin (0,0) in car's coordinate system
              ptsx_car[i] = xdiff * cos(-psi_new) - ydiff * sin(-psi_new);
              ptsy_car[i] = xdiff * sin(-psi_new) + ydiff * cos(-psi_new);
          } 
```
2. After waypoints are transformed to car's coordinate system, then the initial MPC state of x, y, psi should be 0 since we assume car always start from origin with 0 heading angle as you can see from code in MPC.cpp here:
```sh
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v_new, cte, epsi;
          
          //Calling MPC controller and upate control outputs
          std::vector<double> solution = mpc.Solve(state, coeffs);
```
3. Lastly, those newly transformed waypoints are polynomial fitted to generate reference trajectory for calculating CTE and Epsi for MPC using two functions defined in main.cpp: **polyfit** and **polyeval**. **polyfit** is used to fit waypoints with a 3rd order polynomial, and **polyeval** is used to find the reference y_ref=f(x) and heading angle psi_ref = arctan(df/dx(x)). This is shown in code in MPC.cpp here:
```sh
      AD<double> f0 = coeffs[0] + (coeffs[1] * x0) + (coeffs[2] * x0 * x0) + (coeffs[3] * x0 * x0 * x0);
      AD<double> psides0 = CppAD::atan((3 * coeffs[3] * x0 * x0) + (2 * coeffs[2] * x0) + coeffs[1]);
```

## MPC dealing with actuator latency

One advantage of MPC compared with PID control is that it can easily deal with actuator latency by incorporting this latency into state equations or model. As suggested in the class, the way to deal with this actuator latency is to run a simulation using the vehicle model starting from the current state for the duration of latency. And then use the resulting state from the simulation as the new initial state. In this project this is realized in the code in main.cpp like this:
```sh
          //Add 100ms actuator latency, create a new initial state for MPC using the vehicle model
          //starting from the current state for the duration of latency
          double latency = 0.1;
          std::vector<double> current_state = {px, py, psi, v, acceleration, delta};
          
          //Predict new initial state with consideration of 100ms actuator latency
          std::vector<double> new_state = mpc.LatencyPredict(current_state, latency);
          double px_new = new_state[0];
          double py_new = new_state[1];
          double psi_new = new_state[2];
          double v_new = new_state[3];
```

And the LatencyPredict function is defined in MPC.cpp like this:
```sh
//Predict new vehicle state using vehicle model and latency time
std::vector<double> MPC::LatencyPredict(std::vector<double> state, double latency) {
  double px = state[0];
  double py = state[1];
  double psi = state[2];
  double v = state[3];
  double acceleration = state[4];
  double delta = state[5];

  //Calculate new initial states after actuator latency based on original initial state
  //using vehicle kinematic equations
  double px_new = px + (v * cos(psi) * latency);
  double py_new = py + (v * sin(psi) * latency);
  double psi_new = psi - ((v * delta * latency)/Lf);
  double v_new = v + (acceleration * latency);

  std::vector<double> result = {px_new, py_new, psi_new, v_new};

  return result;
}
```
## MPC Tuning

### N and dt tuning
One important tuning of MPC is to tune number of steps in horizon **N** and time elapses between actuations. As suggested in class, first is to pick a reasonable **T** which is called prediction horize T = N * dt. For driving car, T should be a few seconds at most. Otherwise the car is not predictable. So I first pick T = 1s with two combinations of N and dt:

* N = 20, dt = 0.05
The test result is shown in [video1](https://github.com/GitHubChuanYu/T2Project5_ModelPredictiveControl/blob/master/N_20_dT_005_MPC.mp4)
As you can see from the video, the car is pretty good at straight line and smooth corner driving, however it failed at second sharp corner. I think the reason is that the prediction horizon or number of steps N are relative large so the prediction is not accurate anymore in sharp corner for the car to drive throught it smoothly.

* N = 10, dt = 0.1
The test result is shown in [video2](https://github.com/GitHubChuanYu/T2Project5_ModelPredictiveControl/blob/master/N_10_dT_01_MPC.mp4)
From the video, we can also see that MPC drives the car off the track at the second sharp corner. This time N is small, but the problem is dt is not small enough to accurate predicte the fast change in corner, this is related to the so-called "discretization error".

So after the above try, I am thinking to reduce T to 0.5s and also tried two combinations:

* N = 20, dt = 0.025
The test result is shown in [video3](https://github.com/GitHubChuanYu/T2Project5_ModelPredictiveControl/blob/master/N_20_dT_0025_MPC.mp4)
Now the car can drive through the track without leaving the track successfully. I think this is mainly because dt is further reduced to 0.025 compared with dt = 0.05 with N kept the same at 20, so each prediction and control is more accurate so that the car can drive through sharp corner successfully.

* N = 10, dt = 0.05
The test result is shown in [video4](https://github.com/GitHubChuanYu/T2Project5_ModelPredictiveControl/blob/master/N_10_dT_005_MPC.mp4)
With the comination, the car can also drives through track without leaving the lane successfully, however compared with N = 20, dt = 0.025, this tune of MPC does not have more accurate prediction at each step but less prediction steps, so it can also drive through sharp corner but just with less smoothness and higher speed compared with N = 20 and dt = 0.025.

Besides above tuning sets, I have also tried other cominations as shown below:

* N = 5, dt = 0.05 (T = 0.25s): [video5](https://github.com/GitHubChuanYu/T2Project5_ModelPredictiveControl/blob/master/N_5_dT_005_MPC.mp4)
The number of steps for predictive N is too short, the car does not follow the track from the beginning.

* N = 5, dt = 0.1 (T = 0.5s): [video6](https://github.com/GitHubChuanYu/T2Project5_ModelPredictiveControl/blob/master/N_5_dT_01_MPC.mp4)
The number of steps for predictive N is too short, the car does not follow the track from the beginning.

* N = 7, dt = 0.07 (T = 0.49s): [video7](https://github.com/GitHubChuanYu/T2Project5_ModelPredictiveControl/blob/master/N_7_dT_007_MPC.mp4)
Even the car follows the track, but it is off the lane center lane too much, so the performance is not as good as N = 20, dt = 0.025 or N = 10, dt = 0.05.

Eventually, I pick **= 20, dt = 0.025** as my final tuning. Even if it reduces the speed a little bit during curve compared with **N = 10, dt = 0.05**, however it has smooth and more accurate trajectory following during curve.
