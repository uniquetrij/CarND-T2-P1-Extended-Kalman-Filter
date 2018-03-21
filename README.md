[//]: # (Image References)

[image1]: ./images/lidar_radar_1.png
[image2]: ./images/lidar_only_1.png
[image3]: ./images/radar_only_1.png
[image4]: ./images/lidar_radar_2.png
[image5]: ./images/lidar_only_2.png
[image6]: ./images/radar_only_2.png


# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


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

## Result

The program was run on both datasets and the final state of the simulator is presented in the following screenshots.

### Dataset 1:

|Using LIDAR in conjunction to RADAR|Using LIDAR Only    |Using RADAR Only    |
|:---------------------------------:|:------------------:|:------------------:|
|![alt text][image1]                |![alt text][image2] |![alt text][image3] |

|RMSE   |Using LIDAR in conjunction to RADAR|Using LIDAR Only    |Using RADAR Only    |
|:-----:|:---------------------------------:|:------------------:|:------------------:|
|X      |0.0974                             |0.1474              |0.2304              |
|Y      |0.0855                             |0.1154              |0.3467              |
|Vx     |0.4517                             |0.6390              |0.5840              |
|Vy     |0.4404                             |0.5351              |0.8048              |


### Dataset 2:

|Using LIDAR in conjunction to RADAR|Using LIDAR Only    |Using RADAR Only    |
|:---------------------------------:|:------------------:|:------------------:|
|![alt text][image4]                |![alt text][image5] |![alt text][image6] |

|RMSE   |Using LIDAR in conjunction to RADAR|Using LIDAR Only    |Using RADAR Only    |
|:-----:|:---------------------------------:|:------------------:|:------------------:|
|X      |0.0726                             |0.1169              |0.2709              |
|Y      |0.0965                             |0.1262              |0.3857              |
|Vx     |0.4216                             |0.6231              |0.6530              |
|Vy     |0.4932                             |0.6030              |0.9227              |


## Conclusion

The RMSE in the results clearly shows that using LIDAR in conjunction with RADAR produces much lesser error than using either LIDAR or RADAR alone. In fact, using only RADAR produces far worse errors than using LIDAR alone. 