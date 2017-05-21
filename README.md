# Extended Kalman Filter Project

---
This project implements an extended Kalman filter using LIDAR and RADAR sensor inputs. The code is based on [Udacity's started code](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).

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
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

## Results

The following RMSE values are acheived on the data "obj_pose-laser-radar-synthetic-input.txt"

Accuracy - RMSE:
0.0972256 (px)
0.0853761 (py)
0.450855  (vx)
0.439588  (vy)

## Visualization results

The compiled code was tested in a kalman filter visualizer. The vehicle drives in pattern 8. The triangles represent the position estimates from this kalman filter.

<img src="https://github.com/bhatiaabhishek/CarND-ExtendedKalmanFilters/blob/master/Visualize_KF.png" width="60%"> 
