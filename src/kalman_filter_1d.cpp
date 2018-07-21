// Write a function 'filter()' that implements a multi-
// dimensional Kalman Filter for the example given
//============================================================================
#include <iostream>
#include "Dense"
#include <vector>

using namespace std;
using namespace Eigen;

//Kalman Filter variables
VectorXd x;	// object state
MatrixXd P;	// object covariance matrix
VectorXd u;	// external motion
MatrixXd F; // state transition matrix
MatrixXd H;	// measurement matrix
MatrixXd R;	// measurement covariance matrix
MatrixXd I; // Identity matrix
MatrixXd Q;	// process covariance matrix

vector<VectorXd> measurements;
void filter(VectorXd &x, MatrixXd &P);


int main() {
	/**
	* Code used as example to work with Eigen matrices
	*/
	//	//you can create a  vertical vector of two elements with a command like this
	//	VectorXd my_vector(2);
	//	//you can use the so called comma initializer to set all the coefficients to some values
	//	my_vector << 10, 20;
	//
	//
	//	//and you can use the cout command to print out the vector
	//	cout << my_vector << endl;
	//
	//
	//	//the matrices can be created in the same way.
	//	//For example, This is an initialization of a 2 by 2 matrix
	//	//with the values 1, 2, 3, and 4
	//	MatrixXd my_matrix(2,2);
	//	my_matrix << 1, 2,
	//			3, 4;
	//	cout << my_matrix << endl;
	//
	//
	//	//you can use the same comma initializer or you can set each matrix value explicitly
	//	// For example that's how we can change the matrix elements in the second row
	//	my_matrix(1,0) = 11;    //second row, first column
	//	my_matrix(1,1) = 12;    //second row, second column
	//	cout << my_matrix << endl;
	//
	//
	//	//Also, you can compute the transpose of a matrix with the following command
	//	MatrixXd my_matrix_t = my_matrix.transpose();
	//	cout << my_matrix_t << endl;
	//
	//
	//	//And here is how you can get the matrix inverse
	//	MatrixXd my_matrix_i = my_matrix.inverse();
	//	cout << my_matrix_i << endl;
	//
	//
	//	//For multiplying the matrix m with the vector b you can write this in one line as let¡¯s say matrix c equals m times v.
	//	//
	//	MatrixXd another_matrix;
	//	another_matrix = my_matrix*my_vector;
	//	cout << another_matrix << endl;


	//design the KF with 1D motion
	x = VectorXd(2);
	x << 0, 0;

	P = MatrixXd(2, 2);
	P << 1000, 0, 0, 1000;

	u = VectorXd(2);
	u << 0, 0;

	F = MatrixXd(2, 2);
	F << 1, 1, 0, 1;

	H = MatrixXd(1, 2);
	H << 1, 0;

	R = MatrixXd(1, 1);
	R << 1;

	I = MatrixXd::Identity(2, 2);

	Q = MatrixXd(2, 2);
	Q << 0, 0, 0, 0;

	//create a list of measurements
	VectorXd single_meas(1);
	single_meas << 1;
	measurements.push_back(single_meas);
	single_meas << 2;
	measurements.push_back(single_meas);
	single_meas << 3;
	measurements.push_back(single_meas);

	//call Kalman filter algorithm
	filter(x, P);
	getchar();
	return 0;

}


void filter(VectorXd &x, MatrixXd &P) {

	for (unsigned int n = 0; n < measurements.size(); ++n) {

		VectorXd z = measurements[n];
		//YOUR CODE HERE

		// KF Measurement update step
		VectorXd y = z - H * x;
		MatrixXd Ht = H.transpose();
		MatrixXd S = H * P * Ht + R;
		MatrixXd Si = S.inverse();
		MatrixXd K = P * Ht * Si;
		// new state
		x = x + (K * y);
		P = (I - K * H) * P;

		// KF Prediction step
		x = F * x + u;
		MatrixXd Ft = F.transpose();
		P = F * P *Ft + Q;

		std::cout << "x=" << std::endl << x << std::endl;
		std::cout << "P=" << std::endl << P << std::endl;


	}
}