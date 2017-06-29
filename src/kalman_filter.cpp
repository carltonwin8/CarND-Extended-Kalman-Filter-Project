#include <iostream>
#include "kalman_filter.h"
// debug remove iostream above

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std; // debug remove

KalmanFilter::KalmanFilter() {
    I_ = MatrixXd::Identity(4, 4);
    u_ = VectorXd(4);
    u_ << 0, 0, 0, 0;
}

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
  x_ = F_*x_ + u_;
  P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_*x_;
  MatrixXd s = H_*P_*H_.transpose()+R_;
  MatrixXd K = P_*H_.transpose()*s.inverse();
  x_ = x_ + K*y;
  P_ = (I_ - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd h;
  h = VectorXd(3);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float px2py2 = px*px + py*py;
  h(0) = sqrt(px2py2);
  h(1) = atan2(py, px);
  h(2) = (px*vx + py*vy)/h(0);
  VectorXd y = z - h;
  float ypre = y(1);
  //y(1) = fmod(y(1), M_PI);
  // y(1) = (y(1) > M_PI) ? y(1) - 2*M_PI : (y(1) < -M_PI) ? y(1) + 2*M_PI : y(1);
  if (ypre > M_PI) {
      cout << ypre << " > " << M_PI << endl;
      y(1) = y(1) - 2*M_PI;
  } else if (y(1) < -M_PI) {
      cout << ypre << " > " << -M_PI << endl;
      y(1) = y(1) + 2*M_PI;
  } else {
      cout << -M_PI << " < " << ypre << " < " << M_PI << endl;
  }

  MatrixXd s = H_*P_*H_.transpose()+R_;
  MatrixXd K = P_*H_.transpose()*s.inverse();
  x_ = x_ + K*y;
  int age;
  /*if (x_(0) < -14) */
  { cout << K << endl << M_PI << endl <<
            "z " << z(0) << " " << z(1) << " " << z(2) << endl <<
      "h " << h(0) << " " << h(1) << " " << h(2) << endl <<
      "y " << y(0) << " " << y(1) << " " << y(2) << " y-pre " << ypre << endl;
      /* << "enter to continue" << endl; cin >> age; */ }
  P_ = (I_ - K*H_)*P_;
}
