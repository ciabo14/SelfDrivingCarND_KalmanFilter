#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;


Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the inputs 
    if(estimations.size() == 0 || ground_truth.size() == 0 || estimations.size() != ground_truth.size())
    {
        cout << "Error with estimation and ground_truth size" << endl;
        return rmse;
    }
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i)
		rmse = rmse.array() + (estimations[i]-ground_truth[i]).array()*(estimations[i]-ground_truth[i]).array();

	//calculate the mean and the squared root
    rmse /= estimations.size();
    rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  	
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float pSum = pow(px,2) + pow(py,2);
    float pSqrt = sqrt(pSum);
    float ps32 = sqrt(pow(pSum,3));
	
	//check division by zero
	
    if(fabs(pSum) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}
	
	//compute the Jacobian matrix
    else
        Hj << px/pSqrt,                 py/pSqrt,               0,          0,
              -py/pSum,                 px/pSum,                0,          0,
              (px*(vx*py-vy*px))/ps32,  (py*(vy*px-vx*py))/ps32,px/pSqrt,   py/pSqrt;
    
	return Hj;

}
