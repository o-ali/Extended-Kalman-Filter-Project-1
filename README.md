# Term 2 Project 1 - Extended Kalman Filters
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Modifications
The files that have been modified to make the kalman filter reach the desired RMSE thresholds are:
	1. FushionEKF.cpp
	2. FusionEKF.h
	3. kalman_filter.cpp
	4. kalman_filter.h
	5. main.cpp
	6. tools.cpp
	7. tools.h
All of these can be found in the src/ folder.

Example output matrices is in the file results/output matrices.txt and a screenshot of the result for dataset 1 in the term2 sim is below.
![Screenshot](/results/Dataset1_Results.jpg)