# **Extended Kalman Filter**

## Project scope
The goals of this project are:
* Utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 
* Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric
* Data provided live by a simulator

## Writeup / README

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

```
mkdir build
cd build
cmake ..
make
./ExtendedKF
```

Most of this project's code was already provided, with a lot of TODO's.
My contribution was mostly replacing those TODOs with the code developed throughout the course and marking them as DONE to keep track of the status.

The only issue I have encountered was with handling the sign of the theta angle when updating the model with measurements from the radar.
Kepping the angle in the -pi/pi range solved the issue.

The only files that were changed from the given template were src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h


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
