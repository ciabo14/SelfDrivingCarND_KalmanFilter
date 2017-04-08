#include "kalman_filter.h"

#include <iostream>

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {

	//State transiction matrix F_ assumes a constant speed motion with dt dependancy
	/*	1	0	dt	0
		0	1	0	dt
		0	0	1	0
		0	0	0	1
	*/
	//Process covariance matrix assumes to have some acceleration noise Q_

	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
/*
* update the state by using Kalman Filter equations
*/

	VectorXd y = z - H_ * x_;
	
	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose() * S.inverse();
	
	//new estimate
	x_ = x_ + (K * y);

	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

	VectorXd z_pred(3);
	z_pred = ComputeMeasurementFunction();
	VectorXd y = z - z_pred;
	
	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose() * S.inverse();

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}


void KalmanFilter::Set_Q_(const MatrixXd &Q_in) {
	Q_ = Q_in;
}
void KalmanFilter::Set_F_(const MatrixXd &F_in) {
	F_ = F_in;
}
void KalmanFilter::Set_H_(const MatrixXd &H_in) {
	H_ = H_in;
}

VectorXd KalmanFilter::ComputeMeasurementFunction(){
	
	//Apply the following measurement function to the state for measurement conversion
	/*	sqrt(px^2+py^2),
		arctan(px/py),
		(px*vx+py*vy)/sqrt(px^2+py^2)
	*/
	double px = x_[0];
    double py = x_[1];
    double vx = x_[2];
    double vy = x_[3];
	
	if(fabs(px) < 0.0001)
        px = 0.0001;
    
	double rho = sqrt(pow(px,2)+pow(py,2));
	
	if(fabs(rho) < 0.0001)
        rho = 0.0001;

	double phi = atan(py/px);
	double rhoDot = (px*vx+py*vy)/rho;
	
	VectorXd measFormatState(3);
	measFormatState << 	rho, phi, rhoDot;
	
	return measFormatState;
}