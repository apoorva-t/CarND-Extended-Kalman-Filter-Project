#include "math.h"
#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  /**
  TODO:
    * predict the state
  */

  x_ = F_ * x_ ;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - (H_ * x_);

  computeGainAndUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  double theta = atan2(x_(1), x_(0));
  double rhodot = (x_(0)*x_(2)+x_(1)*x_(3));

  //avoid divide by zero
  if (fabs(rho) > 0.0001) {
	  rhodot = rhodot/rho;
  }
  else rhodot = 0;

  VectorXd h(3);
  h << rho, theta, rhodot;

  VectorXd y = z - h;

  //normalize angle in y vector to make it between (-pi,pi)
  if (y(1) > M_PI)
	  y(1) -= 2*M_PI;
  else if (y(1) < -2*M_PI)
	  y(1) += 2*M_PI;

  computeGainAndUpdate(y);
}

void KalmanFilter::computeGainAndUpdate(const VectorXd &y)
{
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd K = P_ * Ht * S.inverse();

	//update measurement
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

	P_ = (I - K * H_) * P_;
}
