# Extended Kalman Filter Project

Self-Driving Car Engineer Nanodegree Program

[image1]: ./Docs/complete_1.png "dataset_1"
[image2]: ./Docs/complete_2.png "dataset_2"
[image3]: ./Docs/ekf_flow.jpg "ekf_flow"
[image4]: ./Docs/ekf_kf_compare.png "ekf_kf"

In this project a kalman filter is used to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

The code consists of the following files:

- src
  - FusionEKF.cpp
  - FusionEKF.h
  - kalman_filter.cpp
  - kalman_filter.h
  - tools.cpp
  - tools.h
- .clang-format

## Measurement Flow

- **first measurement**: the filter will receive initial measurements of an object's position relative to the car. These measurements will come from a radar or lidar sensor.
- **initialize state and covariance matrices**: the filter will initialize the object's position based on the first measurement then the car will receive another sensor measurement after a time period Δt.
- **predict**: the algorithm will predict where the bicycle will be after time Δt. One basic way to predict the bicycle location after Δt is to assume the object's velocity is constant; thus the object will have moved velocity * Δt.
- **update**: the filter compares the "predicted" location with what the sensor measurement says. The predicted location and the measured location are combined to give an updated location. The Kalman filter will put more weight on either the predicted location or the measured location depending on the uncertainty of each value then the car will receive another sensor measurement after a time period Δt. The algorithm then does another predict and update step.

![][image3]

The extended Kalman filter is almost the same as a basic Kalman filter except the H, and F are different. However, in this case, since I‘m assuming a constant velocity model which is linear, the F is the same.

## Compare EKF and KF

![][image4]

## Results

### Dataset 1

![][image1]

RMSE Values:

- X: 0.09
- Y: 0.09
- Vx: 0.47
- Vy: 0.44

### Dataset 2

![][image2]

RMSE Values:

- X: 0.08
- Y: 0.09
- Vx: 0.46
- Vy: 0.49

### main.cpp file Explaination

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Code Style

A modified version of the [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) has been used. Clang format was used to make sure of the style format.

## Generating Additional Data

No additional data was generated for this project.

To generate extra radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## References

[Blog](https://medium.com/@mithi/sensor-fusion-and-object-tracking-using-an-extended-kalman-filter-algorithm-part-2-cd20801fbeff)
