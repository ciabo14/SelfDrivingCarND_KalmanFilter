#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);

	H_laser_ << 1, 0, 0, 0,
				0, 1, 0, 0;

	Hj_ = MatrixXd(3, 4);

	//measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
		0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
		0, 0.0009, 0,
		0, 0, 0.09;

	//state covariance matrix P
	P_ = MatrixXd(4, 4);
	P_ <<	1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1000, 0,
			0, 0, 0, 1000;

	// State transiction matrix
	F_ = MatrixXd(4,4);

	// Process Covariance Matrix
	Q_ = MatrixXd(4,4);
	
	//process noises
	noise_ax_ = 9;
	noise_ay_ = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


	/*****************************************************************************
	*  Initialization
	****************************************************************************/
	if (!is_initialized_) {

		// first measurement
		cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4);
		ekf_.x_ << 1, 1, 1, 1;

		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			
			double rho =  measurement_pack.raw_measurements_[0];
			double phi =  measurement_pack.raw_measurements_[1];
			double px = rho * cos(phi);
			double py = rho * sin(phi);
			
			//@TODO: Undestand what happend when the state is firstly initialized with px and py close to zero.
			ekf_.x_ << px, py, 0 , 0;

		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

			//@TODO: Undestand what happend when the state is firstly initialized with px and py close to zero.
			ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0 , 0;
		}
		// Also the timestamp need to be initialized to the first timestamp.
		previous_timestamp_ = measurement_pack.timestamp_ / 1000000.0;
		

		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/*****************************************************************************
	*  Prediction
	****************************************************************************/

	/**
	TODO:
		* Update the state transition matrix F according to the new elapsed time.
		- Time is measured in seconds.
		* Update the process noise covariance matrix.
		* Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	*/
	
	// Compute the timestamp in seconds
	double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
	UpdateMatricesForPrediction(dt);
	ekf_.Predict();

	/*****************************************************************************
	*  Update
	****************************************************************************/

	/**
	TODO:
		* Use the sensor type to perform the update step.
		* Update the state and covariance matrices.
	*/

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar updates
		ekf_.Update(measurement_pack.raw_measurements_);
	} else {
		// Laser updates
		Hj_ = Tools::CalculateJacobian(ekf_.x_);
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	}

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
void FusionEKF::UpdateMatricesForPrediction(const double dt) {
	  
	float dt44 = pow(dt,4)/4;
	float dt32 = pow(dt,3)/2;
	float dt2 = pow(dt,2);
	
	Q_ <<	dt44*noise_ax_,	0,				dt32*noise_ax_,	0,
			0,				dt44*noise_ay_,	0,				dt32*noise_ay_,
			dt44*noise_ax_,	0,				dt2*noise_ax_,	0,
			0,				dt44*noise_ay_,	0,				dt2*noise_ay_;

	ekf_.Set_Q_(Q_);
	/*
	F_ <<	1,0,dt,0,
			0,1,0,dt,
			0,0,1,0,
			0,0,0,1;
	*/
	//ekf_.Set_F_(F_);
	ekf_.F_(0,2) = dt;
	ekf_.F_(1,3) = dt;

  }

void FusionEKF::UpdateMatricesForUpdate(const double dt, const VectorXd prediction) {
	  
	/**
	Computation of the JACOBIAN Matrix for the update step and update of this matrix in the EKF object
	**/

	//predicted state parameters
	float px = prediction(0);
	float py = prediction(1);
	float vx = prediction(2);
	float vy = prediction(3);

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//check division by zero
	if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
	}

	//compute the Jacobian matrix
	
	ekf_.H_

}