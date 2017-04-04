#include "kalman_filter.h"

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

  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd y = z - H_ * x_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}


void KalmanFilter::Set_Q_(const MatrixXd &Q_in) {
	Q_ = Q_in;
}
void KalmanFilter::Set_F_(const MatrixXd &F_in) {
	F_ = F_in;
}