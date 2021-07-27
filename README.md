# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

[//]: # (Image References)

[image1]: ./results/MeasurementProcess.PNG "1.EKF measurement process.png"
[image2]: ./results/MeasurementProcess.PNG "2.EKF block diagram.png"
[image3]: ./results/EKF_dataset_1.png "3.Result with dataset 1"
[image4]: ./results/EKF_dataset_2.png "4.Result with dataset 2"


Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `
5. Run: `./<folder_path>/term2_sim.x86_64`
6. Select Dataset 1 or 2, then Start

## EKF block diagram and measurement process
1. EKF block diagram
  ![EKF block diagram][image1]
2. EKF measure and update process
  ![EKF measure and update process][image2]
## EKF results with term2 simulation

  ![Result with dataset 1][image3]
  
  ["estimate_marker",{"estimate_x":-7.23245979391494,"estimate_y":10.8959189178428,"rmse_vx":0.407104786526373,"rmse_vy":0.46816549241109,"rmse_x":0.0982753802861895,"rmse_y":0.0851595418973946}]

  ![Result with dataset 2][image4]
  
  42["estimate_marker",{"estimate_x":-7.23245979391494,"estimate_y":10.8959189178428,"rmse_vx":0.407104786526373,"rmse_vy":0.46816549241109,"rmse_x":0.0982753802861895,"rmse_y":0.0851595418973946}]


