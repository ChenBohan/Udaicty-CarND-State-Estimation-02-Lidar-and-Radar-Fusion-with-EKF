# Robotics-Sensor-Fusion-02-EKF-Lidar-and-Radar-Fusion

Udacity Self-Driving Car Engineer Nanodegree:  Lidar and Radar Fusion with Kalman Filters in C++.

The task is to track a prdestrain moving in front of our autonomous vehicle.

This project can use multiple data sources originating from different sensors to estimate a more accurate object state.

## Overview
The Kalman Filter algorithm will go through the following steps:
- First measurement.
- initialize state and covariance matrices.
- Predict.
- Update.
- Do another predict and update step (when receive another sensor measurement).

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Overview%20of%20the%20Kalman%20Filter%20Algorithm%20Map.png" width = "70%" height = "70%" div align=center />

## Two-step estimation problem

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Estimation%20Problem%20Refresh.png" width = "60%" height = "60%" div align=center />

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Estimation%20Problem%20Refresh2.png" width = "60%" height = "60%" div align=center />

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Estimation%20Problem%20Refresh3.png" width = "60%" height = "60%" div align=center />

## State Prediction

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/State%20Prediction.png" width = "60%" height = "60%" div align=center />

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/State%20Prediction2.png" width = "60%" height = "60%" div align=center />

Because our state vector only tracks position and velocity, we are modeling acceleration as a random noise. 

```cpp
void KalmanFilter::Predict() {
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
```

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

- ``z``measurement vector
- ``x``state vector
- ``R``measurement covariance matrix
- ``H``measurement matrix: Find the right H matrix to project from a 4D state to a 2D observation space.

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Laser%20Measurements.png" width = "50%" height = "50%" div align=center />

```cpp
void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
```

#### Disadvantages

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Disadvantages.png" width = "50%" height = "50%" div align=center />

It works quite well when the pedestrian is moving along the straght line.

However, our linear motion model is not perfect, especially for the scenarios when the pedestrian is moving along a circular path.

To solve this problem, we can predict the state by using a more complex motion model such as the circular motion.

### Radar Measurements

The radar can directly measure the object ``range``, ``bearing``, ``radial velocity``.

We use the extended kalman filter in Radar Measurements for non-linear function.

What we change is we simply use non-linear function f(x) to predict the state, and h(X) to compute the measurement error.

So we first linearize the non-linear prediction and measurement functions, and then use the same mechanism to estimate the new state.

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Generalization.png" width = "50%" height = "50%" div align=center />

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Radar%20Measurements.png" width = "50%" height = "50%" div align=center />

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Radar%20Measurements2.png" width = "50%" height = "50%" div align=center />

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Radar%20Measurements3.png" width = "30%" height = "30%" div align=center />

Extended Kalman filter (EKF) is the nonlinear version of the Kalman filter which linearizes about an estimate of the current mean and covariance.

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Extended%20Kalman%20Filter.png" width = "50%" height = "50%" div align=center />

Calculate the Jacobian matrix

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Jacobian%20Matrix.png" width = "50%" height = "50%" div align=center />

```cpp
//pre-compute a set of terms to avoid repeated calculation
float c1 = px*px+py*py;
float c2 = sqrt(c1);
float c3 = (c1*c2);

//check division by zero
if(fabs(c1) < 0.0001){
	cout << "CalculateJacobian () - Error - Division by Zero" << endl;
	return Hj;
}

//compute the Jacobian matrix
Hj << (px/c2), (py/c2), 0, 0,
	-(py/c1), (px/c1), 0, 0,
	py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
```

## Evaluating KF Performance

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Evaluating%20KF%20Performance.png" width = "50%" height = "50%" div align=center />

```cpp
VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size()
		|| estimations.size() == 0) {
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse / estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}
```
