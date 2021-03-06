#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
	/**
	* Constructor.
	*/
	FusionEKF();

	/**
	* Destructor.
	*/
	virtual ~FusionEKF();

	/**
	* Run the whole flow of the Kalman Filter from here.
	*/
	void ProcessMeasurement(const MeasurementPackage &measurement_pack);

	/**
	* Kalman Filter update and prediction math lives in here.
	*/
	KalmanFilter ekf_;

	/**
	* Function used to update the Transition matrix and the Process covariance matrix with the current Dt
	*/
	void UpdateMatricesForPrediction(double dt);
	void UpdateMatricesForUpdate(double dt, const Eigen::VectorXd measurement);

private:
	// check whether the tracking toolbox was initiallized or not (first measurement)
	bool is_initialized_;

	// previous timestamp
	double previous_timestamp_;

	// tool object used to compute Jacobian and RMSE
	Tools tools;
	Eigen::MatrixXd R_laser_;
	Eigen::MatrixXd R_radar_;
	Eigen::MatrixXd H_laser_;
	Eigen::MatrixXd P_;
	Eigen::MatrixXd F_; 
	Eigen::MatrixXd Q_;
	Eigen::VectorXd x_;
	double noise_ax_;
	double noise_ay_;
};

#endif /* FusionEKF_H_ */