# Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion
Udacity Self-Driving Car Engineer Nanodegree:  Lidar and Radar Fusion with Kalman Filters in C++.
The task is to track a prdestrain moving in front of our autonomous vehicle.

## Content of this repository
- src
  - `Eigen` A C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
  - `kalman_filter_1d.cpp` Kalman Filter for 1D tracking problem in C++. 
  

## Overview
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

Because our state vector only tracks position and velocity, we are modeling acceleration as a random noise. 

## Measurements

### Laser & Radar synthetic input (object position)

```
L	4.632272e-01	6.074152e-01	1477010443000000	6.000000e-01	6.000000e-01	2.199937e+00	0	0	6.911322e-03
R	8.986584e-01	6.176736e-01	1.798602e+00	1477010443050000	7.099968e-01	6.000190e-01	2.199747e+00	7.601581e-04	3.455661e-04	1.382155e-02
L	9.685213e-01	4.054501e-01	1477010443100000	8.199842e-01	6.000950e-01	2.199431e+00	2.280027e-03	1.036644e-03	2.072960e-02
R	9.105743e-01	6.105369e-01	1.462326e+00	1477010443150000	9.299556e-01	6.002660e-01	2.198985e+00	4.558776e-03	2.073124e-03	2.763437e-02
L	9.477518e-01	6.368242e-01	1477010443200000	1.039905e+00	6.005699e-01	2.198410e+00	7.595190e-03	3.454842e-03	3.453479e-02
R	1.441585e+00	4.700585e-01	2.079419e+00	1477010443250000	1.149825e+00	6.010446e-01	2.197701e+00	1.138767e-02	5.181582e-03	4.142974e-02
......
......
```

### Laser Measurements



## Radar Measurements
