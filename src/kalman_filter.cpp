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
  R_laser_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
		x = F * x + u;
		MatrixXd Ft = F.transpose();
		P = F * P * Ft + Q;
  */
  x_ = F_ * x_;
  MatrixXd Ft_ = F_.transpose();
  P_ = F_ * P_ * Ft_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  /*
   * KF Measurement update step
   */
  VectorXd y_ = z - H_ * x_;
  MatrixXd Ht_ = H_.transpose();
  MatrixXd S_ = H_ * P_ * Ht_ + R_laser_;
  MatrixXd Si_ = S_.inverse();
  MatrixXd K_ =  P_ * Ht_ * Si_;

  //new state
  x_ = x_ + (K_ * y_);
  P_ = (I_ - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  //float pie = 3.14159;

  Eigen::VectorXd polar;
  polar = VectorXd(3);
  polar(0) = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  polar(1) = atan2(x_(1), x_(0));
  polar(2) = (x_(0)*x_(2) + x_(1)*x_(3)) / polar(0);
  //cout << "polar =\n"<<polar<<"\n";

  Eigen::VectorXd y_;
  y_ = VectorXd(3);
  y_ = z - polar;

  while (y_(1) > M_PI || y_(1) < -M_PI) {
    if (y_(1) > M_PI) y_(1) = y_(1) - (2*M_PI);
    if (y_(1) < -M_PI) y_(1) = y_(1) + (2*M_PI);
  }
  MatrixXd Hjt_ = Hj_.transpose();

  MatrixXd S_ = Hj_ * P_ * Hjt_ + R_radar_;

  MatrixXd Si_ = S_.inverse();

  MatrixXd K_ =  P_ * Hjt_ * Si_;


  //new state
  x_ = x_ + (K_ * y_);

  P_ = (I_ - K_ * Hj_) * P_;


}
