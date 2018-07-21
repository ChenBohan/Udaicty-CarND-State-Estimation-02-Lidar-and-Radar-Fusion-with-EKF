# Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion
Udacity Self-Driving Car Engineer Nanodegree:  Lidar and Radar Fusion with Kalman Filters in C++.
The task is to track a prdestrain moving in front of our autonomous vehicle.

## Content of this repository
- src
  - `Eigen` A C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
  - `kalman_filter_1d.cpp` Kalman Filter for 1D tracking problem in C++. 
  

## Overview of the Kalman Filter Algorithm Map
The Kalman Filter algorithm will go through the following steps:
- First measurement.
- initialize state and covariance matrices.
- Predict.
- Update.
- Do another predict and update step (when receive another sensor measurement).

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Overview%20of%20the%20Kalman%20Filter%20Algorithm%20Map.png" width = "70%" height = "70%" div align=center />

## Two-step estimation problem

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Estimation%20Problem%20Refresh.png" width = "70%" height = "70%" div align=center />

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Estimation%20Problem%20Refresh2.png" width = "70%" height = "70%" div align=center />

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Estimation%20Problem%20Refresh3.png" width = "70%" height = "70%" div align=center />

## State Prediction

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/State%20Prediction.png" width = "70%" height = "70%" div align=center />

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/State%20Prediction2.png" width = "70%" height = "70%" div align=center />

