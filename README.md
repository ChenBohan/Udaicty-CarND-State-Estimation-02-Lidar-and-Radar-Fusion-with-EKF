# CarND-11-Sensor-Fusion-Lidar-and-Radar-Fusion-with-Kalman-Filters

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

<img src="https://github.com/ChenBohan/CarND-13-Localization-Bayes-Markov-Localization/blob/master/readme_img/06-l-apply-bayes-rule-with-additional-conditions.00-02-12-10.still004.png" width = "70%" height = "70%" div align=center />

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
}
```

## Lidar Measurements

- ``x``state vector
- ``z``measurement vector
	- For a lidar sensor, the `z` vector contains the `position−x` and `position-y` measurements.
- ``H``measurement matrix
	- H is the matrix that projects your belief about the object's current state into the measurement space of the sensor.
	- For lidar, this is a fancy way of saying that we discard velocity information from the state variable since the lidar sensor only measures position
	- Find the right H matrix to project from a 4D state to a 2D observation space.
	```cpp
	kf_.H_ = MatrixXd(2, 4);
	kf_.H_ << 1, 0, 0, 0,
			  0, 1, 0, 0;
    ```
- ``R``Measurement Noise Covariance Matrix 
	- Because our state vector only tracks position and velocity, we are modeling acceleration as a random noise.
	- Generally, the parameters for the random noise measurement matrix will be **provided by the sensor manufacturer**.
	```cpp
	kf_.R_ = MatrixXd(2, 2);
	kf_.R_ << 0.0225, 0,
              0, 0.0225;
	```
<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Laser%20Measurements.png" width = "50%" height = "50%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-02-EKF-Lidar-and-Radar-Fusion/blob/master/readme_img/Process%20Covariance%20Matrix" width = "60%" height = "60%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-02-EKF-Lidar-and-Radar-Fusion/blob/master/readme_img/Process%20Covariance%20Matrix2.png" width = "60%" height = "60%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-02-EKF-Lidar-and-Radar-Fusion/blob/master/readme_img/Process%20Covariance%20Matrix3.png" width = "60%" height = "60%" div align=center />

<img src="https://github.com/ChenBohan/Robotics-Sensor-Fusion-02-EKF-Lidar-and-Radar-Fusion/blob/master/readme_img/Process%20Covariance%20Matrix4.png" width = "60%" height = "60%" div align=center />

[Details: Udacity Process Covariance Matrix](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/3612b91d-9c33-47ad-8067-a572a6c93837/concepts/1ac6e0ac-1809-4864-b58f-870d6bda9b25)

### Code
```cpp
	// compute the time elapsed between the current and previous measurements
	// dt - expressed in seconds
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;

	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	// 1. Update state transition matrix F, so that the time is integrated
	kf_.F_(0, 2) = dt;
	kf_.F_(1, 3) = dt;

	// 2. Update process covariance matrix Q based on new elapesd time
	kf_.Q_ = MatrixXd(4, 4);
	kf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
		      0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
		      dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
		      0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;

	// 3. Call the Kalman Filter predict() function
	kf_.Predict();

	// 4. Call the Kalman Filter update() function with the most recent raw measurements_
	kf_.Update(measurement_pack.raw_measurements_);
```

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
}
```

### Disadvantages

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Disadvantages.png" width = "50%" height = "50%" div align=center />

It works quite well when the pedestrian is moving along the straght line.

However, our linear motion model is not perfect, especially for the scenarios when the pedestrian is moving along a circular path.

To solve this problem, we can predict the state by using a more complex motion model such as the circular motion.

## Radar Measurements

### Overview

#### Difference

The main differences betweem EKF and KF are:
- the ``F`` matrix will be replaced by ``F_j`` when calculating ``P'.
- the ``H`` matrix in the Kalman filter will be replaced by the Jacobian matrix ``H_j`` when calculating ``S``, ``K``, and ``P``.
- to calculate ``x'``, the prediction update function, ``f``, is used instead of the ``F`` matrix.
- to calculate ``y``, the hhh function is used instead of the ``H`` matrix.

In the radar update step, the Jacobian matrix ``H_j`` is used to calculate ``S``, ``K`` and ``P``. 

To calculate ``y``, we use the equations that map the predicted location ``x'`` from Cartesian coordinates to polar coordinates

The radar sensor will output values in polar coordinates.

In order to calculate yyy for the radar sensor, we need to convert x′x'x′ to polar coordinates.



#### Predict

We are still using a linear model for the prediction step. 

So, for the prediction step, we can still use the regular Kalman filter equations and the F matrix rather than the extended Kalman filter equations. 

#### Measurement

The measurement update for the radar sensor will use the extended Kalman filter equations.

### Calculations

The radar can directly measure the object ``range``, ``bearing``, ``radial velocity``.

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Generalization.png" width = "50%" height = "50%" div align=center />

For radar, there is no ``H`` matrix that will map the state vector ``x`` into polar coordinates; instead, we need to calculate the mapping manually to convert from cartesian coordinates to polar coordinates. 

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Radar%20Measurements.png" width = "50%" height = "50%" div align=center />

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Radar%20Measurements2.png" width = "50%" height = "50%" div align=center />

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Radar%20Measurements3.png" width = "30%" height = "30%" div align=center />

We use the extended kalman filter in Radar Measurements for non-linear function.

What we change is we simply use non-linear function f(x) to predict the state, and h(X) to compute the measurement error.

So we first linearize the non-linear prediction and measurement functions, and then use the same mechanism to estimate the new state.

Extended Kalman filter (EKF) is the nonlinear version of the Kalman filter which linearizes about an estimate of the current mean and covariance.

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Extended%20Kalman%20Filter.png" width = "50%" height = "50%" div align=center />

To derive a linear approximation for the h function, we will only keep the expansion up to the **Jacobian matrix Df(a)**. 

We will **ignore the Hessian matrix D^2f(a) and other higher order terms**. 

Assuming (x - a) is small, (x - a)^2 or the multi-dimensional equivalent will be even smaller;

[Details: Calculate the Jacobian matrix](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/3612b91d-9c33-47ad-8067-a572a6c93837/concepts/ad446f45-d757-41b0-be5d-3f0a28d3dfd1)

<img src="https://github.com/ChenBohan/Auto-Car-Sensor-Fusion-02-Lidar-and-Radar-Fusion/blob/master/readme_img/Jacobian%20Matrix.png" width = "50%" height = "50%" div align=center />

### Code
```cpp
MatrixXd CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
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
