#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	if (estimations.size() == 0 ||
			estimations.size()!=ground_truth.size())
	{
		cout << "Invalid estimation or ground truth data" << endl;
		cout << "estimate size" << estimations.size() << endl;
		return rmse;
	}

	for (int i = 0; i < estimations.size(); i++)
	{
		VectorXd residual = estimations[i] - ground_truth[i];
		//co-efficient wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}
	rmse = rmse/estimations.size();
	rmse = rmse.array().sqrt();
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = c1*c2;

	if (fabs(c1) < 0.0001) {
		cout << "Error - divison by zero in CalculateJacobian()" << endl;
	}
	if (fabs(c2) < 0.0001) {
		cout << "Error - divison by zero in CalculateJacobian()" << endl;
	}
	MatrixXd Hj(3,4);
	Hj << (px/c2), (py/c2), 0, 0,
			(-py/c1), (px/c1), 0, 0,
			py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
	return Hj;
}
