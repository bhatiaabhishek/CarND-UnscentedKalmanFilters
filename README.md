# Unscented Kalman Filter Project

---
This project implements an Unscented Kalman filter using LIDAR and RADAR sensor inputs. The code is based on [Udacity's started code](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project).

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

## Results

The following RMSE values are acheived on the data "obj_pose-laser-radar-synthetic-input.txt"

Accuracy - RMSE:
RMSE
0.0661727
0.0871531
0.1772
0.278968

## Visualization results

I plotted the NIS values for LIDAR and RADAR seperately as shown below.

<img src="https://github.com/bhatiaabhishek/CarND-UnscentedKalmanFilters/blob/master/NIS_LASER.png" width="60%"> 

<img src="https://github.com/bhatiaabhishek/CarND-UnscentedKalmanFilters/blob/master/NIS_RADAR.png" width="60%"> 
